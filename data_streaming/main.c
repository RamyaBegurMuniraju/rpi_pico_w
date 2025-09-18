/* EXAMPLE_START LE Credit-Based Flow-Control Mode Server - Receive data over L2CAP
 *
 */

#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#include "hci.h"
#include "hci_dump.h"
#include "hci_dump_embedded_stdout.h"
#include "btstack.h"


#define TEST_PACKET_SIZE 1000
#define TOTAL_FILE_SIZE (2 * 1024 * 1024)

#define REPORT_INTERVAL_MS 3000
#define MAX_NR_CONNECTIONS 3 

#define APP_AD_FLAGS 0x06

const uint16_t TSPX_le_psm = 0x0081;
static uint32_t bytes_received = 0;
static uint32_t start_time = 0;

static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0xC, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'P', 'i', 'c', 'o', ' ', 's', 'e', 'r', 'v', 'e', 'r',
    0x03, BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x1a, 0x18,
};

const uint8_t adv_data_len = sizeof(adv_data);

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t l2cap_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

// support for multiple clients
typedef struct {
    char name;
    hci_con_handle_t connection_handle;
    uint16_t cid;
    int  counter;
    char test_data[TEST_PACKET_SIZE];
    int  test_data_len;
    uint32_t test_data_sent;
    uint32_t test_data_start;
} le_cbm_connection_t;

static le_cbm_connection_t le_cbm_connection;

static uint16_t initial_credits = L2CAP_LE_AUTOMATIC_CREDITS;
static uint8_t data_channel_buffer[TEST_PACKET_SIZE];
static const uint8_t all_le_events_mask[] = { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

static void test_track_data(le_cbm_connection_t * context, int bytes_transferred){
    context->test_data_sent += bytes_transferred;
    // evaluate
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t time_passed = now - context->test_data_start;
	printf("time_passed = %ld\n", time_passed);
    if (time_passed < REPORT_INTERVAL_MS) {
		printf("returning from here \n");
		return;
	}
    // print speed
    int bytes_per_second = context->test_data_sent * 1000 / time_passed;
    printf("%c: %"PRIu32" bytes sent-> %u.%03u kB/s\n", context->name, context->test_data_sent, bytes_per_second / 1000, bytes_per_second % 1000);

    // restart
    context->test_data_start = now;
    context->test_data_sent  = 0;
}

 /* LISTING_START(streamer): Streaming code */



#define CHUNK_SIZE 128
static uint32_t bytes_sent = 0;
static uint8_t data_chunk[CHUNK_SIZE];

static void streamer(void) {
    if (bytes_sent >= TOTAL_FILE_SIZE) {
        printf("Transfer complete: %lu bytes sent\n", bytes_sent);
        return;
    }

    // Fill buffer with data
    for (int i = 0; i < CHUNK_SIZE; i++) {
        data_chunk[i] = (uint8_t)((bytes_sent + i) & 0xFF);  // Simulated file data
    }

    uint32_t remaining = TOTAL_FILE_SIZE - bytes_sent;
    uint32_t to_send = btstack_min(CHUNK_SIZE, remaining);

    int err = l2cap_send(le_cbm_connection.cid, data_chunk, to_send);
    if (err) {
        printf("Send error: %d\n", err);
        return;
    }

    bytes_sent += to_send;
    test_track_data(&le_cbm_connection, to_send);

    if (bytes_sent < TOTAL_FILE_SIZE) {
        l2cap_request_can_send_now_event(le_cbm_connection.cid);
    }
	printf("sent successfully\n");
}

/* LISTING_END */

void set_phy(uint16_t conn_handle) {
    // Request 2M PHY for both TX and RX
    uint8_t all_phys = 0x00;     // You want to specify both TX and RX
    uint8_t tx_phys  = 0x02;     // 2M PHY
    uint8_t rx_phys  = 0x02;     // 2M PHY
    uint16_t phy_opts = 0x0000;  // No PHY options
    printf ("hci_send returns = %d\n", hci_send_cmd(&hci_le_set_phy, conn_handle, all_phys, tx_phys, rx_phys, phy_opts));
}

void check_le_features(void) {
    //printf("Sending LE Read Local Supported Features command...\n");
    hci_send_cmd(&hci_le_read_local_supported_features);
}

/* 
 * @section HCI + L2CAP Packet Handler
 *
 * @text The packet handler is used to stop the notifications and reset the MTU on connect
 * It would also be a good place to request the connection parameter update as indicated 
 * in the commented code block.
 */

/* LISTING_START(packetHandler): Packet Handler */
static void packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
	UNUSED(channel);

	bd_addr_t event_address;
	uint16_t psm;
	uint16_t cid;
	uint16_t conn_interval;
	hci_con_handle_t handle;
	uint8_t status;


	switch (packet_type) {
		case HCI_EVENT_PACKET:
			switch (hci_event_packet_get_type(packet)) {
				case BTSTACK_EVENT_STATE:
					if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
						printf(" BTstack activated, get started\n");
						// setup advertisements
						uint16_t adv_int_min = 0x0030;
						uint16_t adv_int_max = 0x0030;
						uint8_t adv_type = 0;
						bd_addr_t null_addr;
						memset(null_addr, 0, 6);
						gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
						gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);
						gap_advertisements_enable(1);	

					} 
					break;
				case HCI_EVENT_DISCONNECTION_COMPLETE:
					printf("Disconnect, reason 0x%02x\n", hci_event_disconnection_complete_get_reason(packet));
					le_cbm_connection.connection_handle = HCI_CON_HANDLE_INVALID;
					break;
				case HCI_EVENT_COMMAND_COMPLETE:
					if (hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_READ_LOCAL_SUPPORTED_FEATURES) {

						const uint8_t *params = hci_event_command_complete_get_return_parameters(packet);
						// The payload starts with the status (1 byte)
						uint8_t status = params[0];
						if (status == 0) {
							// The LE Features bitmask starts after the status byte
							const uint8_t *features = &params[1];
							printf("LE Features: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
									features[7], features[6], features[5], features[4],
									features[3], features[2], features[1], features[0]);

							// The LE Features are defined in Core Spec v5.2, Vol 6, Part B, Section 4.6.1
							// The mask is Little Endian, so features[0] is the first byte.
							// Bit 1 (0x02) indicates LE 2M PHY support.
							if ((features[0] & 0x02) != 0) {
								printf("  - Supports LE 2M PHY\n");
							}
							// Bit 4 (0x10) indicates LE Coded PHY support.
							if ((features[0] & 0x10) != 0) {
								printf("  - Supports LE Coded PHY\n");
							}
						} else {
							printf("LE Read Local Supported Features command failed with status: 0x%02x\n", status);
						}
					} 
					if(hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_SET_EVENT_MASK) 
                    	printf("LE Event Mask set complete.\n");
					if (hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_CONNECTION_UPDATE)
						printf("hci_opcode_hci_le_connection_update \n");
					if (hci_event_command_complete_get_command_opcode(packet) == HCI_OPCODE_HCI_LE_SET_DEFAULT_PHY)
						printf("hci_opcode_hci_le_set_default_phy\n");
					break;
				case HCI_EVENT_META_GAP:
					switch (hci_event_gap_meta_get_subevent_code(packet)) {
						case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
							printf("************ Received event gap_subevent_le_connection_complete *************\n");
							bd_addr_t addr;
							gap_subevent_le_connection_complete_get_peer_address(packet, addr);
							uint8_t status = gap_subevent_le_connection_complete_get_status(packet);
							hci_role_t role = (hci_role_t) gap_subevent_le_connection_complete_get_role(packet);
							bd_addr_type_t addr_type = (bd_addr_type_t) gap_subevent_le_connection_complete_get_peer_address_type(packet);
							conn_interval = gap_subevent_le_connection_complete_get_conn_interval(packet);
							handle = gap_subevent_le_connection_complete_get_connection_handle(packet);

							printf("Con Int: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
							printf("Con Lat: %u\n", gap_subevent_le_connection_complete_get_conn_latency(packet));
							printf("Con handle = 0x%04x\n", handle);
							printf("LE Connection_complete (status = %u) type %u, %s", status, addr_type, bd_addr_to_str(addr)); 
							check_le_features();
							gap_request_connection_parameter_update(handle, 12, 12, 4, 0x0048);
							// Request 2M PHY
							gap_le_set_phy(handle, 0, 2, 2, 0);
							break;
						default:
							break;
					}
					break;
				case HCI_EVENT_LE_META:
					switch (hci_event_le_meta_get_subevent_code(packet)) {
						case HCI_SUBEVENT_LE_CONNECTION_UPDATE_COMPLETE:
							conn_interval = hci_subevent_le_connection_update_complete_get_conn_interval(packet);
							printf("New Connection Update:\n");
							printf("Conn Int: %u.%02u ms\n", le_cbm_connection.name, conn_interval * 125 / 100, 25 * (conn_interval & 3));
							printf("Conn Lat: %u\n", le_cbm_connection.name, hci_subevent_le_connection_update_complete_get_conn_latency(packet));
							break;
						case HCI_SUBEVENT_LE_CONNECTION_COMPLETE:
							uint16_t conn_handle = little_endian_read_16(packet, 4);
							printf("Connection complete, handle: 0x%04x\n", conn_handle);
							break;
						case HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE:
							printf("PHY updated: status %u, handle 0x%04x, TX PHY %u, RX PHY %u\n",
									packet[2],
									little_endian_read_16(packet, 3),
									packet[5],
									packet[6]);
							break;
						default:
							break;
					}
					break;  

				case L2CAP_EVENT_CONNECTION_PARAMETER_UPDATE_RESPONSE:
					printf("L2CAP Connection Parameter Update Complete, result: 0x%02x\n", l2cap_event_connection_parameter_update_response_get_result(packet));
					break;

					// LE Credit-based Flow-Control Mode

				case L2CAP_EVENT_CBM_INCOMING_CONNECTION: 
					psm = l2cap_event_cbm_incoming_connection_get_psm(packet);
					printf("requesting psm = %02x\n", psm);
					cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
					printf("requesting cid = %02x\n", cid);
					if (psm != TSPX_le_psm) break;
					printf("L2CAP: Accepting incoming connection request for 0x%02x, PSM %02x\n", cid, psm);
					l2cap_cbm_accept_connection(cid, data_channel_buffer, sizeof(data_channel_buffer), initial_credits);
					break;

				case L2CAP_EVENT_CBM_CHANNEL_OPENED:
					// inform about new l2cap connection
					l2cap_event_cbm_channel_opened_get_address(packet, event_address);
					psm = l2cap_event_cbm_channel_opened_get_psm(packet);
					cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
					handle = l2cap_event_cbm_channel_opened_get_handle(packet);
					status = l2cap_event_cbm_channel_opened_get_status(packet);
					if (status == ERROR_CODE_SUCCESS) {
						printf("L2CAP: Channel successfully opened: %s, handle 0x%04x, psm 0x%02x, local cid 0x%02x, remote cid 0x%02x\n",
								bd_addr_to_str(event_address), handle, psm, cid,  little_endian_read_16(packet, 15));
						// setup new 
						le_cbm_connection.counter = 'A';
						le_cbm_connection.cid = cid;
						le_cbm_connection.connection_handle = handle;
						le_cbm_connection.test_data_len = btstack_min(l2cap_event_cbm_channel_opened_get_remote_mtu(packet), sizeof(le_cbm_connection.test_data));
						l2cap_request_can_send_now_event(le_cbm_connection.cid);
					} else {
						printf("L2CAP: Connection to device %s failed, status 0x%02x\n", bd_addr_to_str(event_address), status);
					}
					break;

				case L2CAP_EVENT_CHANNEL_CLOSED:
					printf("L2CAP: Channel closed\n");
					le_cbm_connection.cid = 0;
					break;

				default:
					break;

			}
			break;
		case L2CAP_DATA_PACKET:
			if (bytes_received == 0) {
				start_time = btstack_run_loop_get_time_ms();
			}

			bytes_received += size;
			uint32_t elapsed = btstack_run_loop_get_time_ms() - start_time;

			if (elapsed >= REPORT_INTERVAL_MS || bytes_received >= TOTAL_FILE_SIZE) {
				double seconds = elapsed / 1000.0;
				double kbps = (bytes_received / 1024.0) / seconds;
				//printf("Throughput: %.2f kB/s | Total: %u bytes in %.2f s\n", kbps, bytes_received, seconds);

				// Reset if not complete
				if (bytes_received < TOTAL_FILE_SIZE) {
					start_time = btstack_run_loop_get_time_ms();
					bytes_received = 0;
				}
			}

			if (bytes_received >= TOTAL_FILE_SIZE) {
				printf("Received 2MB. Done.\n");
			}

			break;
		default:
			break;
	}
}

/*
 * @section SM Packet Handler
 *
 * @text The packet handler is used to handle pairing requests
 */
static void sm_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case SM_EVENT_JUST_WORKS_REQUEST:
            printf("Just Works requested\n");
            sm_just_works_confirm(sm_event_just_works_request_get_handle(packet));
            break;
        case SM_EVENT_NUMERIC_COMPARISON_REQUEST:
            printf("Confirming numeric comparison: %"PRIu32"\n", sm_event_numeric_comparison_request_get_passkey(packet));
            sm_numeric_comparison_confirm(sm_event_passkey_display_number_get_handle(packet));
            break;
        case SM_EVENT_PASSKEY_DISPLAY_NUMBER:
            printf("Display Passkey: %"PRIu32"\n", sm_event_passkey_display_number_get_passkey(packet));
            break;
        default:
            break;
    }
}


/* @section Main Application Setup
 *
 * @text Listing MainConfiguration shows main application code.
 * It initializes L2CAP, the Security Manager, and configures the ATT Server with the pre-compiled
 * ATT Database generated from $le_credit_based_flow_control_mode_server.gatt$. Finally, it configures the advertisements
 * and boots the Bluetooth stack.
 */

int main(void)
{

	absolute_time_t next_blink_time;
	bool led_on = false;

	stdio_init_all();
	stdio_usb_init();

    if (cyw43_arch_init()) {
        printf("Failed to initialise CYW43\n");
        return -1;
    }

    printf("Starting BLE advertising example...\n");


    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);



    //sm_init();

    // register for SM events
   // sm_event_callback_registration.callback = &sm_packet_handler;
  //  sm_add_event_handler(&sm_event_callback_registration);

	// register for L2CAP events
    	l2cap_init();
	l2cap_event_callback_registration.callback = &packet_handler;
	l2cap_add_event_handler(&l2cap_event_callback_registration);

	// le data channel setup
	//l2cap_cbm_register_service(&packet_handler, TSPX_le_psm, LEVEL_0);
	printf("Registering LE CBM service with PSM 0x%04x\n", TSPX_le_psm);
int err = l2cap_cbm_register_service(&packet_handler, TSPX_le_psm, LEVEL_0);
printf("l2cap_cbm_register_service returned 0x%02x\n", err);



	hci_power_control(HCI_POWER_ON);
	// turn on!

	next_blink_time = make_timeout_time_ms(1000);  // initial blink timeout

	while (true) {
		async_context_poll(cyw43_arch_async_context());

		/*if (absolute_time_diff_us(get_absolute_time(), next_blink_time) < 0) {
			led_on = !led_on;
			cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
			next_blink_time = make_timeout_time_ms(1000);  // every 1 sec
		}

		async_context_wait_for_work_until(cyw43_arch_async_context(), next_blink_time);
		*/
	}

	cyw43_arch_deinit();

    return 0;
}
/* EXAMPLE_END */
