#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#define L2CAP_PSM       0x1001
#define LOCAL_MTU       1017
#define TOTAL_SIZE      (2 * 1024 * 1024)

static uint16_t l2cap_cid = 0;
static uint32_t bytes_sent = 0;

static void fill_pattern(uint8_t *buf, int len, uint32_t start) {
    for (int i = 0; i < len; i++) {
        buf[i] = (uint8_t)((start + i) & 0xFF);
    }
}

static void send_data(void) {
    if (!l2cap_cid || bytes_sent >= TOTAL_SIZE) return;

    uint8_t tx[LOCAL_MTU];
    uint16_t len = (TOTAL_SIZE - bytes_sent) > LOCAL_MTU ? LOCAL_MTU : (TOTAL_SIZE - bytes_sent);

    fill_pattern(tx, len, bytes_sent);

    int rc = l2cap_send(l2cap_cid, tx, len);
    if (rc == 0) {
        bytes_sent += len;
        if (bytes_sent % (256*1024) == 0) {
            printf("Progress: %u bytes\n", bytes_sent);
        }
    } else {
        // controller busy – retry later
        l2cap_request_can_send_now_event(l2cap_cid);
        return;
    }

    if (bytes_sent < TOTAL_SIZE) {
        l2cap_request_can_send_now_event(l2cap_cid);
    } else {
        printf("Done, sent %u bytes\n", bytes_sent);
    }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("Classic BT up\n");
                gap_set_local_name("PicoW Classic L2CAP");
                gap_discoverable_control(1);
                gap_connectable_control(1);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            gap_pin_code_response(addr, "0000");
        } break;

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm       = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t local_cid = l2cap_event_incoming_connection_get_local_cid(packet);
            if (psm == L2CAP_PSM) {
                l2cap_accept_connection(local_cid);
            } else {
                l2cap_decline_connection(local_cid);
            }
        } break;

        case L2CAP_EVENT_CHANNEL_OPENED: {
            if (l2cap_event_channel_opened_get_status(packet)) {
                printf("L2CAP open failed\n");
                break;
            }
            l2cap_cid = l2cap_event_channel_opened_get_local_cid(packet);
            uint16_t remote_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
            printf("Channel open, cid=%u, remote_mtu=%u\n", l2cap_cid, remote_mtu);
            l2cap_request_can_send_now_event(l2cap_cid);
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            send_data();
            break;

        case L2CAP_EVENT_CHANNEL_CLOSED:
            printf("Channel closed\n");
            l2cap_cid = 0;
            break;
    }
}

int main(void) {
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(2000);

    if (cyw43_arch_init()) {
        printf("CYW43 init failed\n");
        return -1;
    }

    l2cap_init();
    hci_add_event_handler(&(btstack_packet_callback_registration_t){ .callback = packet_handler });
    l2cap_add_event_handler(&(btstack_packet_callback_registration_t){ .callback = packet_handler });

    uint8_t st = l2cap_register_service(packet_handler, L2CAP_PSM, LOCAL_MTU, LEVEL_0);
    if (st) {
        printf("Failed to register service: %u\n", st);
        return -2;
    }

    hci_power_control(HCI_POWER_ON);
    btstack_run_loop_execute();
    return 0;
}

