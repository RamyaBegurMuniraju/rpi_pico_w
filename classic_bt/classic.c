#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "btstack.h"

// ---------------- I2C SLAVE + DMA CONFIG ----------------
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_SLAVE_ADDR  0x17

// Ring buffer (no DMA address wrap; we manage wrap in software)
#define RING_BYTES          (192 * 1024)     // 96 KB
//#define DMA_BLOCK_BYTES     4096
#define DMA_BLOCK_BYTES     8192

static uint8_t ring_buf[RING_BYTES];
static volatile uint32_t rd_offset = 0;     // consumed by BT (bytes)

// Single RX-DMA channel and write offset in BYTES
static int dma_ch = -1;
static volatile uint32_t wr_off = 0;        // producer write offset (bytes)
static uint32_t next_dst = 0;               // next DMA dst offset
static volatile uint32_t dma_irq_count = 0; // debug

// ---------------- BT CLASSIC L2CAP CONFIG ----------------
#define L2CAP_PSM       0x1001
#define L2CAP_TX_MTU    4096              // safe Classic default
//#define L2CAP_TX_MTU    1024              // safe Classic default

static uint16_t l2cap_cid = 0;
static uint16_t remote_mtu = L2CAP_TX_MTU;
static uint8_t  tx_buf[L2CAP_TX_MTU];
static btstack_packet_callback_registration_t hci_cb;

// ---------------- Utils ----------------
static inline uint32_t min_u32(uint32_t a, uint32_t b){ return a < b ? a : b; }

static inline uint32_t ring_avail(void){
    uint32_t w = wr_off, r = rd_offset;
    return (w >= r) ? (w - r) : (RING_BYTES - (r - w));
}
static inline void ring_consume(uint32_t n){
    rd_offset = (rd_offset + n) % RING_BYTES;
}

// ---------------- DMA: one channel -> ring ----------------
static void start_dma_transfer(int ch, uint32_t dst_off_bytes){
    volatile uint32_t *src = &i2c_get_hw(I2C_PORT)->data_cmd; // pops RX FIFO

    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

    // IMPORTANT: do NOT enable DMA address wrap (ring is 96 KB, not a 2^N we chose)
    // channel_config_set_ring(&c, true, 17); // <-- leave disabled

    dma_channel_configure(
        ch, &c,
        &ring_buf[0] + dst_off_bytes,   // destination in ring
        src,                            // I2C RX FIFO
        DMA_BLOCK_BYTES,                // one full block per IRQ
        false
    );

    dma_channel_set_irq0_enabled(ch, true);
    dma_start_channel_mask(1u << ch);
}

static void __isr dma_irq_handler(void){
    if (dma_channel_get_irq0_status(dma_ch)){
        dma_channel_acknowledge_irq0(dma_ch);

        // one block landed at next_dst
        wr_off   = (wr_off   + DMA_BLOCK_BYTES) % RING_BYTES;
        next_dst = (next_dst + DMA_BLOCK_BYTES) % RING_BYTES;
        dma_irq_count++;

        start_dma_transfer(dma_ch, next_dst);
    }
}

static void dma_init_and_start(void){
    dma_ch = dma_claim_unused_channel(true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_ch, true);

    wr_off = 0;
    rd_offset = 0;
    next_dst = 0;

    start_dma_transfer(dma_ch, next_dst);
}

// ---------------- I2C SLAVE SETUP ----------------
static void i2c_slave_init_dma(void){
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // even in slave mode, i2c_init() powers/configures the block
    i2c_init(I2C_PORT, 1000000);                 // value irrelevant in slave mode
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDR);

    // Enable RX-DMA on the I2C block
    i2c_hw_t *hw = i2c_get_hw(I2C_PORT);
    hw->dma_rdlr = 0;                           // DREQ when >=1 byte in RX FIFO
    hw_set_bits(&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);
    hw_clear_bits(&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS); // TX-DMA off

    // Flush any stale RX bytes
    while (i2c_get_read_available(I2C_PORT)) (void)hw->data_cmd;

    dma_init_and_start();

}

// ---------------- L2CAP SEND PUMP ----------------
static bool l2cap_try_send_once(void){
    if (!l2cap_cid ) return false; //|| !l2cap_can_send_packet_now(l2cap_cid)) return false;

    uint32_t avail = ring_avail();
    if (!avail) return false;

    uint32_t to_send = avail;
    //printf("avail = %d\n", avail);
    if (to_send > remote_mtu)   to_send = remote_mtu;
    if (to_send > L2CAP_TX_MTU) to_send = L2CAP_TX_MTU;

    // copy from ring (handle wrap)
    uint32_t rd    = rd_offset % RING_BYTES;
    uint32_t first = to_send;
    uint32_t tail  = 0;
    if (rd + to_send > RING_BYTES){
        first = RING_BYTES - rd;
        tail  = to_send - first;
    }
    memcpy(tx_buf, &ring_buf[rd], first);
    if (tail) memcpy(tx_buf + first, &ring_buf[0], tail);

    if (l2cap_send(l2cap_cid, tx_buf, to_send) == 0){
        ring_consume(to_send);
	  // No ACL buffers — request another send slot and return.
            l2cap_request_can_send_now_event(l2cap_cid);
        return true;
    }
    return false;
}

// ---------------- BTSTACK HANDLERS ----------------
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel); UNUSED(size);
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch(event){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                printf("BT ready. Discoverable + connectable.\n");
                gap_set_class_of_device(0x2c010c);
                gap_discoverable_control(1);
                gap_connectable_control(1);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr; hci_event_pin_code_request_get_bd_addr(packet, addr);
            printf("PIN code request → 0000\n");
            hci_send_cmd(&hci_pin_code_request_reply, addr, 4, '0','0','0','0');
        } break;

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t cid = l2cap_event_incoming_connection_get_local_cid(packet);
            printf("Incoming L2CAP: PSM=0x%04x, cid=%u\n", psm, cid);
            l2cap_accept_connection(cid);
        } break;

        case L2CAP_EVENT_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_channel_opened_get_status(packet);
            if (status){
                printf("L2CAP open failed: 0x%02x\n", status);
                l2cap_cid = 0; break;
            }
            l2cap_cid  = l2cap_event_channel_opened_get_local_cid(packet);
            remote_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
            if (remote_mtu > L2CAP_TX_MTU) remote_mtu = L2CAP_TX_MTU;
            printf("L2CAP open ok: cid=%u, remote_mtu=%u\n", l2cap_cid, remote_mtu);
            l2cap_request_can_send_now_event(l2cap_cid);
        } break;

        case L2CAP_EVENT_CHANNEL_CLOSED:
            printf("L2CAP closed\n");
            l2cap_cid = 0;
            break;

        case L2CAP_EVENT_CAN_SEND_NOW:
	    //printf("Send now\n");
            (void) l2cap_try_send_once();               // send exactly one SDU
            //l2cap_request_can_send_now_event(l2cap_cid); // ALWAYS re-arm
            break;

        default: break;
    }
}

int main(void){
    stdio_init_all();
    sleep_ms(1000);
    printf("Pico W I2C Slave + DMA → BT Classic L2CAP bridge @ 0x%02X\n", I2C_SLAVE_ADDR);

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }

    // BTstack bring-up
    l2cap_init();
    l2cap_register_service(packet_handler, L2CAP_PSM, L2CAP_TX_MTU, LEVEL_0);
    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);
    gap_set_local_name("PicoW I2C-DMA Bridge");
    hci_power_control(HCI_POWER_ON);

    // Start I2C slave + DMA capture
    i2c_slave_init_dma();

    // Light telemetry (no prints in ISR)
    absolute_time_t last = get_absolute_time();
    while (true){
        tight_loop_contents();
	//sleep_ms(1000);
        // (optional) gentle nudge to keep pump hot
        if (l2cap_cid && ring_avail() > 0){
            l2cap_request_can_send_now_event(l2cap_cid);
        }

        if (absolute_time_diff_us(last, get_absolute_time()) > 1000000){
            last = get_absolute_time();
            uint32_t avail = ring_avail();
            uint32_t prev  = (wr_off + RING_BYTES - DMA_BLOCK_BYTES) % RING_BYTES;
         /*   printf("DMA_IRQ=%lu  avail=%lu  rd=%lu  last:%02x %02x %02x %02x\n",
                (unsigned long)dma_irq_count,
                (unsigned long)avail,
                (unsigned long)rd_offset,
                ring_buf[prev+0], ring_buf[prev+1], ring_buf[prev+2], ring_buf[prev+3]);*/
        }
    }
}

