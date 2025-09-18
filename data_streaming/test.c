#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "btstack.h"

#define TSPX_LE_PSM 0x81          // Valid LE CBM PSM ≥ 0x0080, odd
#define TEST_PACKET_SIZE 1000
#define INITIAL_CREDITS L2CAP_LE_AUTOMATIC_CREDITS

static uint8_t data_channel_buffer[TEST_PACKET_SIZE];

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t l2cap_event_callback_registration;

static uint16_t connection_cid;
static hci_con_handle_t connection_handle;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);

    switch (packet_type){
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)){
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                        printf("BTstack is ready.\n");
                        printf("Server advertising LE CBM PSM 0x%02x\n", TSPX_LE_PSM);
                    }
                    break;

                case L2CAP_EVENT_CBM_INCOMING_CONNECTION:
                {
                    uint16_t psm = l2cap_event_cbm_incoming_connection_get_psm(packet);
                    uint16_t cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
                    printf("Incoming LE CBM connection request: PSM 0x%04x, CID 0x%04x\n", psm, cid);

                    if (psm != TSPX_LE_PSM){
                        printf("PSM mismatch, ignoring\n");
                        break;
                    }

                    // Accept the connection
                    connection_cid = cid;
                    l2cap_cbm_accept_connection(cid, data_channel_buffer, sizeof(data_channel_buffer), INITIAL_CREDITS);
                    printf("Accepted connection, CID 0x%04x\n", cid);
                    break;
                }

                case L2CAP_EVENT_CBM_CHANNEL_OPENED:
                {
                    uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
                    uint16_t cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
                    connection_handle = l2cap_event_cbm_channel_opened_get_handle(packet);
                    if (status == ERROR_CODE_SUCCESS){
                        printf("LE CBM channel opened, CID 0x%04x, handle 0x%04x\n", cid, connection_handle);
                        l2cap_request_can_send_now_event(cid);
                    } else {
                        printf("LE CBM channel failed to open, status 0x%02x\n", status);
                    }
                    break;
                }

                case L2CAP_EVENT_CHANNEL_CLOSED:
                    printf("LE CBM channel closed\n");
                    connection_cid = 0;
                    connection_handle = 0;
                    break;

                default:
                    break;
            }
            break;

        default:
            break;
    }
}

int main(void){
    stdio_init_all();
    printf("Starting Pico LE CBM server...\n");

    if (cyw43_arch_init()){
        printf("Failed to init CYW43\n");
        return -1;
    }

    // Register packet handler for HCI events
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Register packet handler for L2CAP events
    l2cap_event_callback_registration.callback = &packet_handler;
    l2cap_add_event_handler(&l2cap_event_callback_registration);

    // Initialize L2CAP & Security Manager
    l2cap_init();
    sm_init();

    // Power on controller first
    hci_power_control(HCI_POWER_ON);

    // Register LE CBM service AFTER HCI is ON
    int err = l2cap_cbm_register_service(&packet_handler, TSPX_LE_PSM, LEVEL_0);
    if (err){
        printf("l2cap_cbm_register_service failed: %d\n", err);
    }

    // Set up advertising
    uint8_t adv_data[] = {
        0x02, BLUETOOTH_DATA_TYPE_FLAGS, 0x06,
        0x0C, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME,
        'P','i','c','o',' ','S','e','r','v','e','r',
    };
    gap_advertisements_set_data(sizeof(adv_data), adv_data);
    gap_advertisements_enable(1);

    while (1){
        async_context_poll(cyw43_arch_async_context());
        sleep_ms(100);
    }

    cyw43_arch_deinit();
    return 0;
}

