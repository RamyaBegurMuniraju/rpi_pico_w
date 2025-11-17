// Pico W: I2C slave + DMA -> L2CAP ERTM (no-drop) bridge
// Goal: accept 2 MB from I2C emulator and deliver ALL of it to remote L2CAP peer

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "btstack.h"
#include "btstack_config.h"

// ======================= I2C + DMA CONFIG =========================
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_SLAVE_ADDR  0x17

// 192 KB ring — big enough to buffer bursts, but smaller than 2 MB
#define RING_BYTES          (192 * 1024)
#define DMA_BLOCK_BYTES     1024    // one DMA block per interrupt

static uint8_t ring_buf[RING_BYTES];
static volatile uint32_t rd_offset = 0;     // consumed by BT
static volatile uint32_t wr_off    = 0;     // produced by DMA
static uint32_t          next_dst  = 0;

static int  dma_ch = -1;
static volatile bool dma_running = false;
static volatile uint32_t dma_irq_count = 0;

// ======================= L2CAP / ERTM CONFIG ======================
#define L2CAP_PSM       0x1001
// Pico W / CYW43 is happy around 1017 (fits ACL 1021)
#define L2CAP_TX_MTU    1017

static uint16_t l2cap_cid  = 0;
static uint16_t remote_mtu = L2CAP_TX_MTU;

static btstack_packet_callback_registration_t hci_cb;

// old-style BTstack ERTM struct (the one you showed)
static l2cap_ertm_config_t ertm_config = {
    .ertm_mandatory            = 0,
    .max_transmit              = 10,
    .retransmission_timeout_ms = 2000,
    .monitor_timeout_ms        = 12000,
    .local_mtu                 = L2CAP_TX_MTU,
    .num_tx_buffers            = 10,
    .num_rx_buffers            = 10,
    .fcs_option                = 0,
};

// ERTM needs working memory
static uint8_t ertm_buffer[24 * 1024];

// ---- send-side "pending" buffer: prevents drops on l2cap_send() busy
static uint8_t  pending_buf[L2CAP_TX_MTU];
static uint16_t pending_len = 0;
static bool     has_pending = false;

// ======================= RING HELPERS =============================
static inline uint32_t ring_avail(void) {
    uint32_t w = wr_off, r = rd_offset;
    return (w >= r) ? (w - r) : (RING_BYTES - (r - w));
}

static inline uint32_t ring_free(void) {
    return RING_BYTES - ring_avail();
}

static inline void ring_consume(uint32_t n) {
    rd_offset = (rd_offset + n) % RING_BYTES;
}

// ======================= DMA HANDLING =============================
static void start_dma_transfer(int ch, uint32_t dst_off_bytes) {
    volatile uint32_t *src = &i2c_get_hw(I2C_PORT)->data_cmd; // pops RX FIFO

    dma_channel_config c = dma_channel_get_default_config(ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_read_increment(&c, false);
    channel_config_set_write_increment(&c, true);
    channel_config_set_dreq(&c, I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

    dma_channel_configure(
        ch, &c,
        &ring_buf[0] + dst_off_bytes,   // destination
        src,                            // source (I2C RX FIFO)
        DMA_BLOCK_BYTES,                // length
        false
    );

    dma_channel_set_irq0_enabled(ch, true);
    dma_start_channel_mask(1u << ch);
    dma_running = true;
}

static void __isr dma_irq_handler(void) {
    if (dma_channel_get_irq0_status(dma_ch)) {
        dma_channel_acknowledge_irq0(dma_ch);

        // advance write pointer and next dst
        wr_off   = (wr_off   + DMA_BLOCK_BYTES) % RING_BYTES;
        next_dst = (next_dst + DMA_BLOCK_BYTES) % RING_BYTES;
        dma_irq_count++;

        // check for space — if no space, stop DMA (backpressure)
        if (ring_free() < DMA_BLOCK_BYTES) {
            // don't restart here
            dma_running = false;
        } else {
            start_dma_transfer(dma_ch, next_dst);
        }
    }
}

static void dma_init_and_start(void) {
    dma_ch = dma_claim_unused_channel(true);

    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);
    dma_channel_set_irq0_enabled(dma_ch, true);

    wr_off   = 0;
    rd_offset = 0;
    next_dst  = 0;
    dma_running = false;

    start_dma_transfer(dma_ch, next_dst);
}

// ======================= I2C SLAVE INIT ===========================
static void i2c_slave_init_dma(void) {
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);

    // in slave mode, this clock doesn't matter
    i2c_init(I2C_PORT, 3.4 * 1000 * 1000);
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDR);

    i2c_hw_t *hw = i2c_get_hw(I2C_PORT);
    hw->dma_rdlr = 0;
    hw_set_bits(&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);
    hw_clear_bits(&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS);

    while (i2c_get_read_available(I2C_PORT)) (void)hw->data_cmd;

    dma_init_and_start();
}

// ======================= L2CAP SEND PUMP ==========================
//
// Always send this way:
//  1. if we have a pending packet, flush it
//  2. else pull from ring
//  3. if can't send, keep it as pending
//
static bool l2cap_try_send_once(void) {
    if (!l2cap_cid) return false;
    if (!l2cap_can_send_packet_now(l2cap_cid)) return false;

    // 1) flush pending first
    if (has_pending) {
        if (l2cap_send(l2cap_cid, pending_buf, pending_len) == 0) {
            has_pending = false;
            // if still data, re-arm
            if (ring_avail() > 0) {
                l2cap_request_can_send_now_event(l2cap_cid);
            }
            return true;
        } else {
            // still can't send
            return false;
        }
    }

    // 2) pull from ring
    uint32_t avail = ring_avail();
    if (!avail) return false;

    uint32_t to_send = avail;
    if (to_send > remote_mtu)   to_send = remote_mtu;
    if (to_send > L2CAP_TX_MTU) to_send = L2CAP_TX_MTU;

    uint32_t rd = rd_offset;
    uint32_t first = to_send;
    uint32_t tail  = 0;
    if (rd + to_send > RING_BYTES) {
        first = RING_BYTES - rd;
        tail  = to_send - first;
    }

    memcpy(pending_buf, &ring_buf[rd], first);
    if (tail) memcpy(pending_buf + first, &ring_buf[0], tail);

    // try to send immediately
    if (l2cap_send(l2cap_cid, pending_buf, to_send) == 0) {
        // success -> now we can consume from ring
        ring_consume(to_send);
        if (ring_avail() > 0) {
            l2cap_request_can_send_now_event(l2cap_cid);
        }
    } else {
        // can't send now -> remember it
        pending_len = to_send;
        has_pending = true;
    }

    return true;
}

// ======================= BTSTACK HANDLER ==========================
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);

    switch (event) {

    case BTSTACK_EVENT_STATE:
        if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
            printf("BT ready. Discoverable + connectable.\n");
            gap_set_class_of_device(0x2c010c);
            gap_discoverable_control(1);
            gap_connectable_control(1);
        }
        break;

    case HCI_EVENT_PIN_CODE_REQUEST: {
        bd_addr_t addr;
        hci_event_pin_code_request_get_bd_addr(packet, addr);
        printf("PIN code request → 0000\n");
        hci_send_cmd(&hci_pin_code_request_reply, addr, 4, '0', '0', '0', '0');
        break;
    }

    case L2CAP_EVENT_INCOMING_CONNECTION: {
        uint16_t psm = l2cap_event_incoming_connection_get_psm(packet);
        uint16_t cid = l2cap_event_incoming_connection_get_local_cid(packet);
        printf("Incoming L2CAP: PSM=0x%04x, cid=%u\n", psm, cid);

        // accept as ERTM with our config
        uint8_t status = l2cap_ertm_accept_connection(
            cid,
            &ertm_config,
            ertm_buffer,
            sizeof(ertm_buffer)
        );
        printf("ERTM accept -> 0x%02x\n", status);
        break;
    }

    case L2CAP_EVENT_CHANNEL_OPENED: {
        uint8_t status = l2cap_event_channel_opened_get_status(packet);
        if (status) {
            printf("L2CAP open failed: 0x%02x\n", status);
            l2cap_cid = 0;
            break;
        }

        l2cap_cid  = l2cap_event_channel_opened_get_local_cid(packet);
        remote_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
        if (remote_mtu > L2CAP_TX_MTU) remote_mtu = L2CAP_TX_MTU;

        printf("L2CAP open OK: cid=%u, remote_mtu=%u\n", l2cap_cid, remote_mtu);

        // optional: stop advertising when connected
        gap_discoverable_control(0);
        gap_connectable_control(0);

        // kick the send pump
        l2cap_request_can_send_now_event(l2cap_cid);
        break;
    }

    case L2CAP_EVENT_CHANNEL_CLOSED:
        printf("L2CAP closed\n");
        l2cap_cid = 0;
        has_pending = false;
        gap_set_class_of_device(0x2c010c);
        gap_discoverable_control(1);
        gap_connectable_control(1);
        break;

    case L2CAP_EVENT_CAN_SEND_NOW:
        (void) l2cap_try_send_once();
        break;

    default:
        break;
    }
}

// ======================= MAIN ====================================
int main(void) {
    stdio_init_all();
    sleep_ms(1000);
    printf("Pico W I2C Slave + DMA -> L2CAP ERTM bridge @0x%02X\n", I2C_SLAVE_ADDR);

    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }

    // BTstack bring-up
    l2cap_init();
    // register service on our PSM
    l2cap_register_service(packet_handler, L2CAP_PSM, L2CAP_TX_MTU, LEVEL_0);

    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);

    gap_set_local_name("PicoW I2C-DMA Bridge");
    hci_power_control(HCI_POWER_ON);

    // start I2C slave + DMA capture
    i2c_slave_init_dma();

    absolute_time_t last = get_absolute_time();

    while (true) {
        tight_loop_contents();

        // keep L2CAP pump hot
        if (l2cap_cid && (has_pending || ring_avail() > 0)) {
            l2cap_request_can_send_now_event(l2cap_cid);
        }

        // restart DMA if it was stopped because ring was full
        if (!dma_running && ring_free() >= DMA_BLOCK_BYTES) {
            start_dma_transfer(dma_ch, next_dst);
        }

        // debug once per second
        if (absolute_time_diff_us(last, get_absolute_time()) > 1000000) {
            last = get_absolute_time();
            printf("DMA_IRQ=%lu avail=%lu free=%lu rd=%lu wr=%lu\n",
                   (unsigned long)dma_irq_count,
                   (unsigned long)ring_avail(),
                   (unsigned long)ring_free(),
                   (unsigned long)rd_offset,
                   (unsigned long)wr_off);
        }
    }
    return 0;
}

