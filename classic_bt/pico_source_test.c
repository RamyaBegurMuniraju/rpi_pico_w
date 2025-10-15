#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#define L2CAP_PSM     0x1001
#define LOCAL_MTU     4096u
#define TOTAL_SIZE    (2u * 1024u * 1024u)

static uint16_t l2cap_cid   = 0;
static uint16_t eff_mtu     = 672;         // negotiated later: min(LOCAL_MTU, remote_mtu)
static uint32_t bytes_sent  = 0;
static uint8_t  tx_buf[LOCAL_MTU];

static inline void fill_pattern(uint8_t *buf, uint16_t len, uint32_t start){
    for (uint16_t i=0;i<len;i++) buf[i] = (uint8_t)((start + i) & 0xFF);
}

static void pump_send(void){
    if (!l2cap_cid || bytes_sent >= TOTAL_SIZE) return;

    // Burst out as many SDUs as the stack allows right now
    while (l2cap_can_send_packet_now(l2cap_cid) && bytes_sent < TOTAL_SIZE){
        uint32_t remaining = TOTAL_SIZE - bytes_sent;
        uint16_t len = (remaining > eff_mtu) ? eff_mtu : (uint16_t)remaining;

        fill_pattern(tx_buf, len, bytes_sent);

        int rc = l2cap_send(l2cap_cid, tx_buf, len);
        if (rc) break;  // controller busy; stop this burst
        bytes_sent += len;
    }

    if (bytes_sent < TOTAL_SIZE){
        l2cap_request_can_send_now_event(l2cap_cid);
    } else {
        printf("Done, sent %u bytes\n", bytes_sent);
        // optional clean EOF:
        // l2cap_disconnect(l2cap_cid, 0);
    }
}

static void packet_handler(uint8_t type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void)channel; (void)size;
    if (type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
		gap_set_default_link_policy_settings(LM_LINK_POLICY_ENABLE_ROLE_SWITCH); // no sniff/hold/park
                gap_set_local_name("PicoW Classic L2CAP");
                gap_set_class_of_device(0x5A020C);
                gap_discoverable_control(1);
                gap_connectable_control(1);
                printf("Classic BT up (PSM 0x%04x)\n", L2CAP_PSM);
            }
            break;



    case HCI_EVENT_CONNECTION_COMPLETE: {
        if (hci_event_connection_complete_get_status(packet)) break;
        // 2) Enforce policy on this *specific* ACL link
        uint16_t handle = hci_event_connection_complete_get_connection_handle(packet);
        // Same setting: allow role switch only; disable hold/sniff/park
        hci_send_cmd(&hci_write_link_policy_settings, handle,
                     LM_LINK_POLICY_ENABLE_ROLE_SWITCH);
        break;
    }

    case HCI_EVENT_MODE_CHANGE: {
        // Optional: if remote forced us into sniff, exit it
        uint8_t mode = hci_event_mode_change_get_mode(packet);  // 0=active, 2=sniff
        uint16_t handle = hci_event_encryption_change_get_connection_handle(packet);
        if (mode == 2 /* sniff */){
            hci_send_cmd(&hci_exit_sniff_mode, handle);
        }
        break;
    }



        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t a; hci_event_pin_code_request_get_bd_addr(packet, a);
            gap_pin_code_response(a, "0000");
        } break;

        case L2CAP_EVENT_INCOMING_CONNECTION: {
            uint16_t psm       = l2cap_event_incoming_connection_get_psm(packet);
            uint16_t local_cid = l2cap_event_incoming_connection_get_local_cid(packet);
            if (psm == L2CAP_PSM){
                // IMPORTANT: accept with MTU so we advertise 4096
                l2cap_accept_connection(local_cid);
            } else {
                l2cap_decline_connection(local_cid);
            }
        } break;

        case L2CAP_EVENT_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_channel_opened_get_status(packet);
            if (status){ printf("L2CAP open failed, status=%u\n", status); break; }
            l2cap_cid = l2cap_event_channel_opened_get_local_cid(packet);
            uint16_t remote_mtu = l2cap_event_channel_opened_get_remote_mtu(packet);
            eff_mtu = remote_mtu < LOCAL_MTU ? remote_mtu : LOCAL_MTU;
            if (eff_mtu > sizeof(tx_buf)) eff_mtu = sizeof(tx_buf);
            bytes_sent = 0;
            printf("Channel open: cid=%u, negotiated MTU=%u\n", l2cap_cid, eff_mtu);
            gap_discoverable_control(0);
	    gap_connectable_control(0);
	    l2cap_request_can_send_now_event(l2cap_cid);
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            pump_send();
            break;

        case L2CAP_EVENT_CHANNEL_CLOSED:
            printf("Channel closed\n");
            l2cap_cid = 0;
            break;
    }
}

int main(void){
    stdio_init_all();
    sleep_ms(800);

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }
    //
    // REQUIRED on Pico W to let BTstack use CYW43:
    //cyw43_arch_enable_btstack();

    l2cap_init();

    // Register raw L2CAP service with offered MTU
    uint8_t st = l2cap_register_service(packet_handler, L2CAP_PSM, LOCAL_MTU, LEVEL_0);
    if (st){ printf("l2cap_register_service failed: %u\n", st); return -2; }

    static btstack_packet_callback_registration_t h;
    h.callback = &packet_handler;
    hci_add_event_handler(&h);
    l2cap_add_event_handler(&h);

    hci_power_control(HCI_POWER_ON);

    while (true){ tight_loop_contents(); }
}

