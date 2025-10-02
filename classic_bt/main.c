#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
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
#define LOCAL_L2CAP_MTU     65535        // we can advertise big; peer will clamp
//#define LOCAL_L2CAP_MTU     1024        // we can advertise big; peer will clamp
#define BLOCK_SIZE          4096         // chosen I2C block size
//#define BLOCK_SIZE          5120         // testing chosen I2C block size
//#define BLOCK_SIZE          6144         // testing chosen I2C block size
//#define BLOCK_SIZE          1024         // testing chosen I2C block size

static btstack_packet_callback_registration_t hci_ev_reg;
static btstack_packet_callback_registration_t l2cap_ev_reg;

static uint16_t l2_cid   = 0;
static uint16_t peer_mtu = 0;
static bool     bt_connected = false;

// --------- I2C <-> BT block state -----------
static volatile uint8_t  status_flag = 0x01;       // 0x01 = READY, 0x00 = BUSY
static uint8_t           rxbuf[BLOCK_SIZE];
static volatile uint16_t rx_count = 0;             // bytes collected into rxbuf
static volatile bool     block_ready = false;      // true when rx_count==BLOCK_SIZE

// For BT sending progress:
static uint16_t send_off = 0;                      // how many bytes already sent from rxbuf

// ---------------- I2C slave ISR ----------------
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE: {
            // Drain FIFO quickly into rxbuf until BLOCK_SIZE reached
            while (i2c_get_read_available(i2c) && !block_ready) {
                uint8_t b = i2c_read_byte_raw(i2c);
                rxbuf[rx_count++] = b;

                if (rx_count >= BLOCK_SIZE) {
                    // Got one full block -> mark BUSY, freeze buffer, kick BT
                    block_ready = true;
                    status_flag = 0x00;   // BUSY
                    send_off = 0;
                    if (bt_connected && l2_cid) {
                        l2cap_request_can_send_now_event(l2_cid);
                    }
                }
            }
        } break;

        case I2C_SLAVE_REQUEST: {
            // Master polls our status
            i2c_write_byte_raw(i2c, status_flag);
        } break;

        case I2C_SLAVE_FINISH:
        default:
            break;
    }
}

// --------------- BT send pump (event-driven) ---------------
static void bt_send_block_if_possible(void) {
    if (!bt_connected || !l2_cid) return;
    if (!block_ready) return;                 // nothing to send

    // Effective MTU for a single l2cap_send
    uint16_t mtu = peer_mtu ? peer_mtu : LOCAL_L2CAP_MTU;
    if (mtu > LOCAL_L2CAP_MTU) mtu = LOCAL_L2CAP_MTU;
    if (mtu == 0) mtu = 1017;                 // safety default

    while (send_off < BLOCK_SIZE) {
        uint16_t remaining = BLOCK_SIZE - send_off;
        uint16_t n = (remaining > mtu) ? mtu : remaining;

        int err = l2cap_send(l2_cid, rxbuf + send_off, n);
        //int err = l2cap_send(l2_cid, rxbuf, 4096);
        if (err) {
            // No ACL buffers — request another send slot and return.
            l2cap_request_can_send_now_event(l2_cid);
            return;
        }

        send_off += n;
        // Ask again so we send the next slice ASAP
        if (send_off < BLOCK_SIZE) {
            l2cap_request_can_send_now_event(l2_cid);
            return;
        }
    }

    //printf("// Entire 4092-byte block sent successfully.\n");
    rx_count    = 0;
    block_ready = false;
    send_off    = 0;
    status_flag = 0x01;   // READY for next block
}

// ------------- BT event handler ---------------
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("BT up\n");
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
            if (status) { printf("L2CAP open failed, status=0x%02x\n", status); break; }
            l2_cid   = l2cap_event_channel_opened_get_local_cid(packet);
            peer_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
            bt_connected = true;
            printf("L2CAP open ok, cid=%u, remote_mtu=%u\n", l2_cid, peer_mtu);

            // If a block is already ready, kick sending
            if (block_ready) l2cap_request_can_send_now_event(l2_cid);
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            bt_send_block_if_possible();
            break;

        case L2CAP_EVENT_CHANNEL_CLOSED:
            bt_connected = false;
            l2_cid = 0;
            break;
    }
}

// --------------- Setup helpers ----------------
static void setup_i2c_slave(void) {
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_handler);

    irq_set_priority(I2C0_IRQ, 0x40);   // high on RP2040
    irq_set_enabled(I2C0_IRQ, true);
}

// -------------------- main --------------------
int main(void) {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(300);

    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }

    // BT L2CAP server
    l2cap_init();
    hci_ev_reg.callback   = &packet_handler;
    l2cap_ev_reg.callback = &packet_handler;
    hci_add_event_handler(&hci_ev_reg);
    l2cap_add_event_handler(&l2cap_ev_reg);

    if (l2cap_register_service(&packet_handler, L2CAP_PSM, LOCAL_L2CAP_MTU, LEVEL_0)) {
        printf("l2cap_register_service failed\n");
        return -2;
    }

    setup_i2c_slave();



    // Make sure both STA and AP are down, Turning off WiFi
    cyw43_arch_disable_sta_mode();
    cyw43_arch_disable_ap_mode();

    hci_power_control(HCI_POWER_ON);

    // Simple telemetry
    absolute_time_t last = get_absolute_time();
    while (true) {
        async_context_poll(cyw43_arch_async_context());
#if 0
        if (absolute_time_diff_us(last, get_absolute_time()) >= 1000000) {
            /*printf("blk_ready=%d rx_count=%u send_off=%u mtu=%u status=%u BT=%d\n",
                   block_ready, (unsigned)rx_count, (unsigned)send_off,
                   (unsigned)peer_mtu, (unsigned)status_flag, bt_connected);*/
            last = get_absolute_time();
        }
#endif
    }
}

