#if 0
#include <stdio.h>
#include "btstack.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#define APP_AD_FLAGS 0x06

#define L2CAP_PSM 0x1001
static uint16_t l2cap_cid;

static uint8_t adv_data[] = {
	// Flags general discoverable
	0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
	// Complete Local Name
	0x0B, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', '_', 'A', 'D', 'V', 'E', 'R', 'T'
};
static const uint8_t adv_data_len = sizeof(adv_data);

static btstack_packet_callback_registration_t hci_event_callback_registration;

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
	UNUSED(size);
	UNUSED(channel);

	//if (packet_type != HCI_EVENT_PACKET) return;
	switch(packet_type) {
		case HCI_EVENT_PACKET:
			uint8_t event_type = hci_event_packet_get_type(packet);
			switch(event_type){
				case BTSTACK_EVENT_STATE:
					if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;

					printf("BLE is up, starting advertisement...\n");

					bd_addr_t null_addr = {0};
					gap_advertisements_set_params(800, 800, 2, 0, null_addr, 0x07, 0x00);
					gap_advertisements_set_data(adv_data_len, adv_data);
					gap_advertisements_enable(1);
					break;

				default:
					break;
			}
			break;
		case L2CAP_DATA_PACKET:
			// Echo the data back
			l2cap_send(channel, packet, size);
			break;

		default:
			break;
	}

}

int main() {
	stdio_init_all();

	if (cyw43_arch_init()) {
		printf("Failed to initialise CYW43\n");
		return -1;
	}

	printf("Starting BLE advertising example...\n");


	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	l2cap_init();
	l2cap_register_packet_handler(packet_handler);
	l2cap_register_service(packet_handler, L2CAP_PSM, 100, LEVEL_0);
	sm_init();
	// Turn on Bluetooth
	hci_power_control(HCI_POWER_ON);

	while (true) {
		async_context_poll(cyw43_arch_async_context());
		async_context_wait_for_work_until(cyw43_arch_async_context(), at_the_end_of_time);
	}

	return 0;
}

   #endif


#include <stdio.h>
#include <string.h>
#include "btstack.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#define APP_AD_FLAGS 0x06
#define L2CAP_COC_PSM 0x1001
#define L2CAP_MTU 512
#define REPORT_INTERVAL_MS 3000
#define CREDITS_PER_PACKET 5

// Advertisement Data
static uint8_t adv_data[] = {
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    0x0B, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', '_', 'C', 'O', 'C', '_', 'S', 'R', 'V'
};
static const uint8_t adv_data_len = sizeof(adv_data);

// Packet handler registration
static btstack_packet_callback_registration_t hci_event_callback_registration;

// Connection state
static uint16_t coc_cid = 0;
static uint32_t data_received = 0;
static uint32_t start_time_ms = 0;

static void test_reset(void) {
    data_received = 0;
    start_time_ms = btstack_run_loop_get_time_ms();
}

static void test_track_data(uint16_t size) {
    data_received += size;
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t elapsed = now - start_time_ms;
    if (elapsed >= REPORT_INTERVAL_MS) {
        uint32_t bps = (data_received * 1000) / elapsed;
        printf("Received %u bytes in %u ms -> %u.%03u kB/s\n", data_received, elapsed, bps / 1000, bps % 1000);
        test_reset();
    }
}

// LE CoC Packet Handler
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);

    switch (packet_type) {
        case HCI_EVENT_PACKET: {
            uint8_t event_type = hci_event_packet_get_type(packet);

            switch (event_type) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) return;

                    printf("BLE is up, starting advertising...\n");

                    bd_addr_t null_addr = {0};
                    gap_advertisements_set_params(800, 800, 0x02, 0x00, null_addr, 0x07, 0x00);
                    gap_advertisements_set_data(adv_data_len, adv_data);
                    gap_advertisements_enable(1);
                    break;

                case LE_CREDIT_BASED_CONNECTION_REQUEST: {
                    /*l2cap_event_le_credit_based_connection_request_t *req =
                        (l2cap_event_le_credit_based_connection_request_t *) packet;

                    printf("Incoming LE CoC request: handle 0x%04x, psm 0x%04x\n",
                        req->con_handle, req->psm);
					*/
                    l2cap_cbm_accept_connection(_handle, L2CAP_MTU, L2CAP_MTU, CREDITS_PER_PACKET);
                    break;
                }

                /*case LE_CREDIT_BASED_CONNECTION_COMPLETE: {
                    l2cap_event_le_credit_based_connection_complete_t *evt =
                        (l2cap_event_le_credit_based_connection_complete_t *) packet;

                    if (evt->status == 0) {
                        coc_cid = evt->local_cid;
                        printf("LE CoC channel opened! CID 0x%04x, MTU %u\n", coc_cid, evt->peer_mtu);
                        test_reset();
                    } else {
                        printf("LE CoC connection failed with status 0x%02x\n", evt->status);
                    }
                    break;
                }
				*/

                default:
                    break;
            }
            break;
        }

        default:
            break;
    }
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return -1;
    }

    printf("LE Credit-Based Server Starting...\n");

    // Init BTstack
    l2cap_init();
    l2cap_cbm_register_service(packet_handler, L2CAP_COC_PSM, L2CAP_MTU, LEVEL_0);

    sm_init();  // Security manager

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Power on controller
    hci_power_control(HCI_POWER_ON);

    while (true) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(cyw43_arch_async_context(), at_the_end_of_time);
    }

    return 0;
}
 
