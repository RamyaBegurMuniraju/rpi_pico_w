// pico_i2c_dma_to_rfcomm.c
#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "btstack.h"
#include "classic/rfcomm.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"

// ---------------- I2C SLAVE + DMA CONFIG ----------------
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_SLAVE_ADDR  0x17

#define RING_BYTES          (192 * 1024)      // 192 KiB
#define DMA_BLOCK_BYTES     8192              // 8 KiB per DMA IRQ

static uint8_t  ring_buf[RING_BYTES];
static volatile uint32_t rd_offset = 0;

static int      dma_ch = -1;
static volatile uint32_t wr_off = 0;
static uint32_t next_dst = 0;
static volatile uint32_t dma_irq_count = 0;

// ---------------- RFCOMM / SPP ----------------
#define RFCOMM_CHANNEL 1
static uint16_t rfcomm_cid = 0;
//static uint16_t rfcomm_mtu = 1024;           // will be updated on open
//static uint8_t  tx_buf[1024];                // stage buffer (<= RFCOMM MTU)

static uint16_t rfcomm_mtu = 2048;           // will be updated on open
static uint8_t  tx_buf[2048];                // stage buffer (<= RFCOMM MTU)
static uint8_t  spp_service_buf[200];
static btstack_packet_callback_registration_t hci_cb;

// ---------------- Utils ----------------
static inline uint32_t ring_avail(void){
    uint32_t w = wr_off, r = rd_offset;
    return (w >= r) ? (w - r) : (RING_BYTES - (r - w));
}
static inline void ring_consume(uint32_t n){ rd_offset = (rd_offset + n) % RING_BYTES; }

// ---------------- DMA → ring ----------------
static void start_dma_transfer(int ch, uint32_t dst_off){
    volatile uint32_t *src = &i2c_get_hw(I2C_PORT)->data_cmd;

    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

    dma_channel_configure(ch, &c, &ring_buf[dst_off], src, DMA_BLOCK_BYTES, false);
    dma_channel_set_irq0_enabled(ch, true);
    dma_start_channel_mask(1u << ch);
}
static void __isr dma_irq_handler(void){
    if (!dma_channel_get_irq0_status(dma_ch)) return;
    dma_channel_acknowledge_irq0(dma_ch);

    wr_off   = (wr_off   + DMA_BLOCK_BYTES) % RING_BYTES;
    next_dst = (next_dst + DMA_BLOCK_BYTES) % RING_BYTES;
    dma_irq_count++;

    start_dma_transfer(dma_ch, next_dst);

    if (rfcomm_cid && ring_avail()) {
        rfcomm_request_can_send_now_event(rfcomm_cid);
    }
}
static void dma_init_and_start(void){
    dma_ch = dma_claim_unused_channel(true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_ch, true);

    wr_off = rd_offset = next_dst = 0;
    start_dma_transfer(dma_ch, next_dst);
}

// ---------------- I2C SLAVE SETUP ----------------
static void i2c_slave_init_dma(void){
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    //i2c_init(I2C_PORT, 3.4 *1000 * 1000);
    i2c_init(I2C_PORT, 1000000);
    //i2c_init(I2C_PORT, 4 * 1000000);
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDR);

    i2c_hw_t *hw = i2c_get_hw(I2C_PORT);
    hw->dma_rdlr = 0;
    hw_set_bits(&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);
    hw_clear_bits(&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS);
    while (i2c_get_read_available(I2C_PORT)) (void)hw->data_cmd;

    dma_init_and_start();
}

// ---------------- RFCOMM SEND ----------------
static bool rfcomm_try_send_once(void){
    if (!rfcomm_cid) return false;
    uint32_t avail = ring_avail();
    if (!avail) return false;

    uint32_t to_send = avail;
    if (to_send > rfcomm_mtu)     to_send = rfcomm_mtu;
    if (to_send > sizeof(tx_buf)) to_send = sizeof(tx_buf);

    uint32_t rd = rd_offset;
    uint32_t first = to_send, tail = 0;
    if (rd + to_send > RING_BYTES){ first = RING_BYTES - rd; tail = to_send - first; }
    memcpy(tx_buf, &ring_buf[rd], first);
    if (tail) memcpy(tx_buf + first, &ring_buf[0], tail);

    if (rfcomm_send(rfcomm_cid, tx_buf, (uint16_t)to_send) == 0){
        ring_consume(to_send);
        if (ring_avail()) rfcomm_request_can_send_now_event(rfcomm_cid);
        return true;
    }
    return false;
}

// ---------------- BT events ----------------
static void packet_handler(uint8_t type, uint16_t ch, uint8_t *packet, uint16_t size){
    (void)ch; (void)size;
    if (type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                gap_set_local_name("PicoW I2C→SPP");
                gap_set_class_of_device(0x2c010c);
                gap_discoverable_control(1);
                gap_connectable_control(1);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t a; hci_event_pin_code_request_get_bd_addr(packet, a);
            gap_pin_code_response(a, "0000");
        } break;

        case RFCOMM_EVENT_INCOMING_CONNECTION: {
            rfcomm_cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
            rfcomm_accept_connection(rfcomm_cid);
        } break;

        case RFCOMM_EVENT_CHANNEL_OPENED: {
            uint8_t st = rfcomm_event_channel_opened_get_status(packet);
            if (st){ printf("RFCOMM open failed 0x%02x\n", st); break; }
            rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
            rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
            printf("RFCOMM up: cid=%u, mtu=%u\n", rfcomm_cid, rfcomm_mtu);
            gap_discoverable_control(0);
            gap_connectable_control(0);
            if (ring_avail()) rfcomm_request_can_send_now_event(rfcomm_cid);
        } break;

        case RFCOMM_EVENT_CAN_SEND_NOW:
            (void)rfcomm_try_send_once();
            break;

        case RFCOMM_EVENT_CHANNEL_CLOSED:
            printf("RFCOMM closed\n");
            rfcomm_cid = 0;
            gap_discoverable_control(1);
            gap_connectable_control(1);
            break;
    }
}

// ---------------- BT setup ----------------
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

int main(void){
    stdio_init_all();
    sleep_ms(500);
    printf("Pico W I2C Slave + DMA → RFCOMM\n");

    if (cyw43_arch_init()){ printf("CYW43 init failed\n"); return -1; }
    //cyw43_arch_enable_btstack();  // <-- important

    i2c_slave_init_dma();
    bt_setup();

    while (true){ tight_loop_contents(); }
    return 0;
}

