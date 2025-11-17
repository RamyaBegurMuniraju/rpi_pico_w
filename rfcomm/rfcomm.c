// main.c - I2C slave + DMA → RFCOMM (no padding, no ISR wakeups)
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "btstack.h"
#include "classic/rfcomm.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"

// ------------------- CONFIG -------------------
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_SLAVE_ADDR  0x17

#define RING_BYTES          (192 * 1024)
#define DMA_BLOCK_BYTES     8192
#define TOTAL_BYTES         (2 * 1024 * 1024)   // 2,097,152 bytes

// ------------------- RING / DMA STATE -------------------
static uint8_t  ring_buf[RING_BYTES];
static volatile uint32_t rd_offset = 0;
static volatile uint32_t wr_off    = 0;
static uint32_t          next_dst  = 0;

static int dma_ch = -1;

static volatile bool dma_paused  = false;
static volatile bool dma_pending = false;

// how many bytes we have actually received from I2C
static volatile uint32_t total_rx = 0;

// last DMA length (because we do variable-length DMA)
static volatile uint32_t dma_last_len = DMA_BLOCK_BYTES;

// ------------------- RFCOMM STATE -------------------
#define RFCOMM_CHANNEL 1
static uint16_t rfcomm_cid = 0;
static uint16_t rfcomm_mtu = 512;    // keep this modest to avoid large frames
static uint8_t  tx_buf[512];
static uint8_t  spp_service_buf[200];
static btstack_packet_callback_registration_t hci_cb;

// we will request send-now from the MAIN LOOP only
static volatile bool need_rfcomm_send = false;
static volatile bool rfcomm_send_in_flight = false;

// =====================================================
// RING HELPERS
// =====================================================
static inline uint32_t ring_avail(void){
    uint32_t w = wr_off, r = rd_offset;
    return (w >= r) ? (w - r) : (RING_BYTES - (r - w));
}

static inline uint32_t ring_free(void){
    // leave 1 byte gap
    return (RING_BYTES - 1) - ring_avail();
}

static inline void ring_consume(uint32_t n){
    rd_offset = (rd_offset + n) % RING_BYTES;
}

// =====================================================
// DMA HELPERS
// =====================================================
static void start_dma_transfer_len(int ch, uint32_t dst_off, uint32_t len){
    volatile uint32_t *src = &i2c_get_hw(I2C_PORT)->data_cmd;

    dma_last_len = len;

    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

    dma_channel_configure(ch, &c,
                          &ring_buf[dst_off],
                          src,
                          len,
                          false);
    dma_channel_set_irq0_enabled(ch, true);
    dma_start_channel_mask(1u << ch);
}

static void __isr dma_irq_handler(void){
    if (!dma_channel_get_irq0_status(dma_ch)) return;
    dma_channel_acknowledge_irq0(dma_ch);

    // 1) commit what DMA just wrote
    wr_off   = (wr_off   + dma_last_len) % RING_BYTES;
    next_dst = (next_dst + dma_last_len) % RING_BYTES;
    total_rx += dma_last_len;
    if (total_rx > TOTAL_BYTES)
        total_rx = TOTAL_BYTES;

    // 2) decide next DMA
    if (total_rx >= TOTAL_BYTES) {
        dma_paused  = true;
        dma_pending = false;
    } else {
        uint32_t free_bytes   = ring_free();
        uint32_t remain_bytes = TOTAL_BYTES - total_rx;

        if (free_bytes == 0) {
            dma_paused  = true;
            dma_pending = true;
        } else {
            uint32_t next_len = DMA_BLOCK_BYTES;
            if (next_len > free_bytes)   next_len = free_bytes;
            if (next_len > remain_bytes) next_len = remain_bytes;
            start_dma_transfer_len(dma_ch, next_dst, next_len);
        }
    }

    // 3) DO NOT call rfcomm_request_can_send_now_event() here.
    // just mark that there's data
    if (ring_avail())
        need_rfcomm_send = true;
}

static void dma_init_and_start(void){
    dma_ch = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_ch, true);

    rd_offset   = 0;
    wr_off      = 0;
    next_dst    = 0;
    total_rx    = 0;
    dma_paused  = false;
    dma_pending = false;
    dma_last_len = DMA_BLOCK_BYTES;

    start_dma_transfer_len(dma_ch, next_dst, DMA_BLOCK_BYTES);
}

// =====================================================
// I2C SLAVE
// =====================================================
static void i2c_slave_init_dma(void){
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    i2c_init(I2C_PORT, 3400000);
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDR);

    i2c_hw_t *hw = i2c_get_hw(I2C_PORT);
    hw->dma_rdlr = 0;
    hw_set_bits(&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);
    hw_clear_bits(&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS);

    while (i2c_get_read_available(I2C_PORT)) (void)hw->data_cmd;

    dma_init_and_start();
}

// =====================================================
// RFCOMM SEND (no padding)
// =====================================================
static void rfcomm_send_one_chunk(void){
    if (!rfcomm_cid) {
        rfcomm_send_in_flight = false;
        return;
    }

    uint32_t avail = ring_avail();
    if (!avail) {
        // nothing to send now
        rfcomm_send_in_flight = false;
        return;
    }

    uint32_t to_send = avail;
    if (to_send > rfcomm_mtu)     to_send = rfcomm_mtu;
    if (to_send > sizeof(tx_buf)) to_send = sizeof(tx_buf);

    // handle ring wrap
    uint32_t rd    = rd_offset;
    uint32_t first = to_send;
    uint32_t tail  = 0;
    if (rd + to_send > RING_BYTES) {
        first = RING_BYTES - rd;
        tail  = to_send - first;
    }

    memcpy(tx_buf, &ring_buf[rd], first);
    if (tail) memcpy(tx_buf + first, &ring_buf[0], tail);

    if (rfcomm_send(rfcomm_cid, tx_buf, (uint16_t)to_send) == 0) {
        ring_consume(to_send);

        // if DMA was paused because ring was full, try to resume
        if (dma_paused && dma_pending && (total_rx < TOTAL_BYTES)) {
            uint32_t free_bytes   = ring_free();
            uint32_t remain_bytes = TOTAL_BYTES - total_rx;
            if (free_bytes > 0) {
                uint32_t next_len = DMA_BLOCK_BYTES;
                if (next_len > free_bytes)   next_len = free_bytes;
                if (next_len > remain_bytes) next_len = remain_bytes;
                if (next_len > 0) {
                    dma_paused  = false;
                    dma_pending = false;
                    start_dma_transfer_len(dma_ch, next_dst, next_len);
                }
            }
        }
    }

    // after we send one chunk, we clear the in-flight flag;
    // the main loop will request another one if there is more
    rfcomm_send_in_flight = false;
}

// =====================================================
// BT / RFCOMM EVENTS
// =====================================================
static void packet_handler(uint8_t type, uint16_t ch, uint8_t *packet, uint16_t size){
    (void)ch; (void)size;
    if (type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
            gap_set_local_name("PicoW I2C→SPP");
            gap_set_class_of_device(0x2c010c);
            gap_discoverable_control(1);
            gap_connectable_control(1);
        }
        break;

    case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t addr;
        hci_event_pin_code_request_get_bd_addr(packet, addr);
        gap_pin_code_response(addr, "0000");
    } break;

    case RFCOMM_EVENT_INCOMING_CONNECTION: {
        rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
        rfcomm_accept_connection(rfcomm_cid);
    } break;

    case RFCOMM_EVENT_CHANNEL_OPENED: {
        uint8_t status = rfcomm_event_channel_opened_get_status(packet);
        if (status){
            printf("RFCOMM open failed 0x%02x\n", status);
            rfcomm_cid = 0;
            break;
        }

        rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
        rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
        // we cap it to 512 above to ease cyw43
        if (rfcomm_mtu > sizeof(tx_buf))
            rfcomm_mtu = sizeof(tx_buf);

        printf("RFCOMM up: cid=%u mtu=%u\n", rfcomm_cid, rfcomm_mtu);

        gap_discoverable_control(0);
        gap_connectable_control(0);

        // main loop will trigger first send if data is there
    } break;

    case RFCOMM_EVENT_CAN_SEND_NOW:
        rfcomm_send_one_chunk();
        break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
        printf("BT disconnected\n");
        rfcomm_cid = 0;
        rfcomm_send_in_flight = false;
        gap_discoverable_control(1);
        gap_connectable_control(1);
        break;

    default:
        break;
    }
}

static void bt_setup(void){
    l2cap_init();
    rfcomm_init();

    sdp_init();
    memset(spp_service_buf, 0, sizeof(spp_service_buf));
    uint32_t rec = sdp_create_service_record_handle();
    spp_create_sdp_record(spp_service_buf, rec, RFCOMM_CHANNEL, "SPP Streamer");
    sdp_register_service(spp_service_buf);

    rfcomm_register_service(packet_handler, RFCOMM_CHANNEL, 0xffff);

    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);

    hci_power_control(HCI_POWER_ON);
}

// =====================================================
// MAIN
// =====================================================
int main(void){
    stdio_init_all();
    sleep_ms(500);
    printf("Pico W I2C Slave + DMA → RFCOMM (ISR quiet)\n");

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }
    //cyw43_arch_enable_btstack();

    i2c_slave_init_dma();
    bt_setup();

    while (true){
        // let Wi-Fi/BT driver run
      //  cyw43_arch_poll();
        // user space “pump”:
        if (rfcomm_cid &&
            !rfcomm_send_in_flight &&
            (ring_avail() > 0 || (total_rx >= TOTAL_BYTES && ring_avail() > 0)))
        {
            // request one send-now
            rfcomm_send_in_flight = true;
            rfcomm_request_can_send_now_event(rfcomm_cid);
        }
        sleep_ms(1);
    }
    return 0;
}

