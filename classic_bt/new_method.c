#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "hardware/dma.h"
#include "pico/i2c_slave.h"

#include "btstack.h"
#include "btstack_run_loop_embedded.h"

// ================= I2C config =================
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     4
#define I2C_SCL_PIN     5
#define I2C_SLAVE_ADDR  0x17

// ============== BT Classic (L2CAP) ===========
#define L2CAP_PSM           0x1001
#define LOCAL_L2CAP_MTU     65535        // advertise big; controller will fragment
#define BLOCK_SIZE          4096         // 4 KB SDU

// ---------- IRQ priority helper (RP2040 has 4 effective levels) ----------
#ifndef __NVIC_PRIO_BITS
#define __NVIC_PRIO_BITS 2
#endif
#define NVIC_PRIO(n) ((n) << (8 - __NVIC_PRIO_BITS))  // n=0..3 -> 0x00,0x40,0x80,0xC0

static btstack_packet_callback_registration_t hci_ev_reg;
static btstack_packet_callback_registration_t l2cap_ev_reg;

static uint16_t l2_cid   = 0;
static uint16_t peer_mtu = 0;
static bool     bt_connected = false;

// --------- I2C <-> BT block state (DMA ping-pong) -----------
static volatile uint8_t  status_flag = 0x01;       // 0x01=READY (at least one buffer free), 0x00=BUSY (both full)
static uint8_t           rxbuf[2][BLOCK_SIZE];     // DMA targets
static volatile bool     block_ready[2] = {false, false};
static volatile uint8_t  fill_idx = 0;             // DMA fills here next (0/1)
static volatile uint8_t  send_idx = 0;             // BT sends from here next (0/1)
static uint16_t          send_off = 0;             // offset inside rxbuf[send_idx]
static volatile bool     dma_paused = false;       // both buffers full -> paused
static volatile bool     kick_send  = false;       // poke TX path from main loop

// keep CAN_SEND_NOW "armed" without spamming
static volatile bool     send_req_out = false;

// ---------------- DMA ----------------
static int dma_ch = -1;

// ---------------- Protos ----------------
static void __isr dma_irq_handler(void);
static void dma_arm_for_buffer(uint8_t idx);
static void update_status_flag(void);
static void ensure_can_send(void);

// ---------------- Helpers ----------------
static inline void update_status_flag(void){
    // READY if at least one buffer is free; BUSY if both are full
    status_flag = (block_ready[0] && block_ready[1]) ? 0x00 : 0x01;
}

static void dma_arm_for_buffer(uint8_t idx){
    dma_channel_set_write_addr(dma_ch, rxbuf[idx], false);
    dma_channel_set_trans_count(dma_ch, BLOCK_SIZE, false);
    dma_channel_start(dma_ch);
}

static void dma_setup(void){
    dma_ch = dma_claim_unused_channel(true);
    dma_channel_config c = dma_channel_get_default_config(dma_ch);
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);
    channel_config_set_dreq(&c, (I2C_PORT == i2c0) ? DREQ_I2C0_RX : DREQ_I2C1_RX);
    channel_config_set_read_increment(&c, false);    // fixed RX FIFO
    channel_config_set_write_increment(&c, true);    // linear into buffer

    dma_channel_configure(dma_ch, &c,
        rxbuf[fill_idx],                      // dst (overridden by dma_arm_for_buffer)
        &I2C_PORT->hw->data_cmd,              // src: RX FIFO pop register
        BLOCK_SIZE,
        false
    );

    dma_channel_set_irq0_enabled(dma_ch, true);
    irq_set_priority(DMA_IRQ_0, NVIC_PRIO(3));       // lowest
    irq_set_exclusive_handler(DMA_IRQ_0, dma_irq_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    dma_arm_for_buffer(fill_idx);
}

static void __isr dma_irq_handler(void){
    uint32_t ints = dma_hw->ints0;
    if (ints & (1u << dma_ch)) {
        dma_hw->ints0 = (1u << dma_ch);   // ACK

        // The buffer that just completed is fill_idx
        block_ready[fill_idx] = true;
        kick_send = true;                  // ask main loop to poke TX
        update_status_flag();

        uint8_t other = fill_idx ^ 1;
        if (!block_ready[other]) {
            // continue ping-pong into the other buffer
            fill_idx = other;
            dma_arm_for_buffer(fill_idx);
            dma_paused = false;
        } else {
            // both full -> pause DMA until TX frees one
            dma_paused = true;
        }
    }
}

// ---------------- I2C slave: status byte only (RX via DMA) ---------------
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    (void)i2c;
    switch (event) {
        case I2C_SLAVE_REQUEST:
            i2c_write_byte_raw(I2C_PORT, status_flag); // READY/BUSY
            break;
        case I2C_SLAVE_RECEIVE:
            // RX handled by DMA; do NOT drain FIFO here
            break;
        case I2C_SLAVE_FINISH:
        default:
            break;
    }
}

static void setup_i2c_slave(void) {
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_init(I2C_PORT, 1000 * 1000);                 // 1 MHz safe default
    i2c_set_slave_mode(I2C_PORT, true, I2C_SLAVE_ADDR);

    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_handler);

    // Medium priority so CYW43/HCI can pre-empt
    irq_set_priority(I2C0_IRQ, NVIC_PRIO(2));        // try NVIC_PRIO(3) if needed
    irq_set_enabled(I2C0_IRQ, true);
}

// --------------- TX path helpers ---------------
static inline bool have_tx_data(void){
    return block_ready[0] || block_ready[1] || (send_off > 0);
}

static void ensure_can_send(void){
    if (bt_connected && l2_cid && have_tx_data() && !send_req_out){
        send_req_out = true;
        l2cap_request_can_send_now_event(l2_cid);
    }
}

// --------------- Send one slice per event ---------------
static void bt_send_block_if_possible(void) {
    if (!bt_connected || !l2_cid) return;

    // Select a ready buffer if current isn't ready
    if (!block_ready[send_idx]) {
        uint8_t other = send_idx ^ 1;
        if (block_ready[other]) send_idx = other;
        else return; // nothing ready
    }

    // Effective per-send limit: remote MTU clamped by our advertised local
    uint16_t eff_mtu = peer_mtu ? peer_mtu : LOCAL_L2CAP_MTU;
    if (eff_mtu > LOCAL_L2CAP_MTU) eff_mtu = LOCAL_L2CAP_MTU;
    if (eff_mtu == 0) eff_mtu = 1017;    // safe fallback for Classic

    uint16_t remaining = BLOCK_SIZE - send_off;
    uint16_t send_len  = remaining < eff_mtu ? remaining : eff_mtu;

    int err = l2cap_send(l2_cid, rxbuf[send_idx] + send_off, send_len);
    if (err) {
        // controller busy; will re-arm via ensure_can_send()
        return;
    }

    send_off += send_len;

    if (send_off >= BLOCK_SIZE) {
        // finished this 4KB buffer
        block_ready[send_idx] = false;
        send_off = 0;
        update_status_flag();

        // If DMA was paused (both full), resume into the freed buffer
        if (dma_paused) {
            dma_paused = false;
            fill_idx = send_idx;
            dma_arm_for_buffer(fill_idx);
        }

        // If the other buffer is ready, keep pipeline hot
        if (block_ready[send_idx ^ 1]) {
            send_idx ^= 1;
        }
    }
}

// ------------- BT event handler ---------------
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                gap_set_local_name("PicoW I2C->L2CAP");
                gap_set_class_of_device(0x200404);
                gap_discoverable_control(1);
                gap_connectable_control(1);
                gap_ssp_set_enable(1);
                gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
                gap_set_allow_role_switch(true);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t a;
            hci_event_pin_code_request_get_bd_addr(packet, a);
            gap_pin_code_response(a, "0000");
        } break;

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm       = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t local_cid = l2cap_event_incoming_connection_get_local_cid(packet);
            if (psm == L2CAP_PSM) l2cap_accept_connection(local_cid);
            else                  l2cap_decline_connection(local_cid);
        } break;

        case L2CAP_EVENT_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_channel_opened_get_status(packet);
            if (status) break;
            l2_cid   = l2cap_event_channel_opened_get_local_cid(packet);
            peer_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
            bt_connected = true;

            // If data is already available, arm sending
            ensure_can_send();
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            // one send slot granted — clear in-flight flag, send, then re-arm if needed
            send_req_out = false;
            bt_send_block_if_possible();
            ensure_can_send();
            break;

        case L2CAP_EVENT_CHANNEL_CLOSED:
            bt_connected = false;
            l2_cid = 0;
            break;
    }
}

// -------------------- main --------------------
int main(void) {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(300);

    if (cyw43_arch_init()) {
        return -1;
    }

    // Turn Wi-Fi off (reduce shared-bus contention)
    cyw43_arch_disable_sta_mode();
    cyw43_arch_disable_ap_mode();

    // I2C target (slave) + DMA ping-pong
    setup_i2c_slave();
    dma_setup();

    // BT L2CAP server
    l2cap_init();
    hci_ev_reg.callback   = &packet_handler;
    l2cap_ev_reg.callback = &packet_handler;
    hci_add_event_handler(&hci_ev_reg);
    l2cap_add_event_handler(&l2cap_ev_reg);

    if (l2cap_register_service(&packet_handler, L2CAP_PSM, LOCAL_L2CAP_MTU, LEVEL_0)) {
        return -2;
    }

    hci_power_control(HCI_POWER_ON);

    // Cooperative loop
    while (true) {
        // If DMA completed, poke the sender
        if (kick_send && bt_connected && l2_cid) {
            kick_send = false;
            ensure_can_send();
        }

        // Keep it armed whenever data exists
        ensure_can_send();

        async_context_poll(cyw43_arch_async_context());

        // tiny yield helps avoid HCI bus thrash without hurting throughput
        sleep_us(100);
    }
}

