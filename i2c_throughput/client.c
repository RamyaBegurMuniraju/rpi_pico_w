#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "pico/cyw43_arch.h"                                                                                                                                           
#include "pico/btstack_cyw43.h" 
#include "pico/stdlib.h"

#include "btstack.h"
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_slave.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_embedded.h"
#include "btstack_defines.h"
#include "btstack_event.h"
#include "btstack.h"






#define TEST_PACKET_SIZE 4096
#define TOTAL_TEST_DATA_SIZE (2 * 1024 * 1024)  // <-- MOD: 2MB total data



#define I2C_SLAVE_ADDRESS 0x17
#define I2C_BAUDRATE 100000
#define I2C_SLAVE_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SLAVE_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

#define BLE_L2CAP_PSM 0x0081
#define I2C_BUFFER_SIZE 240


static enum {
		TC_OFF,
		TC_IDLE,
		TC_W4_SCAN_RESULT,
		TC_W4_CONNECT,
		TC_W4_CHANNEL,
		TC_TEST_DATA
} state = TC_OFF;


const uint8_t adv_data[] = {
		// Flags general discoverable, BR/EDR not supported
		0x02, 0x01, 0x06,
		// Name
		0x0E, 0x09, 'L', 'E', ' ', 'C', 'B', 'M',  ' ', 'S', 'e', 'r', 'v', 'e', 'r',
};
const uint8_t adv_data_len = sizeof(adv_data);

const uint16_t TSPX_le_psm = 0x0081;

static bd_addr_t cmdline_addr;
static int cmdline_addr_found = 0;

// addr and type of device with correct name
static bd_addr_t      le_cbm_server_addr;
static bd_addr_type_t le_cbm_server_addr_type;

static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t l2cap_event_callback_registration;


static uint8_t cbm_receive_buffer[TEST_PACKET_SIZE];

static uint16_t initial_credits = L2CAP_LE_AUTOMATIC_CREDITS;
static uint8_t data_channel_buffer[TEST_PACKET_SIZE];



// random MAC address for the device, used if nothing else is available
static const bd_addr_t random_address = { 0xC1, 0x01, 0x01, 0x01, 0x01, 0x01 };



#define REPORT_INTERVAL_MS 3000

typedef struct {
		char name;
		hci_con_handle_t connection_handle;
		uint16_t cid;
		int  counter;
		char test_data[TEST_PACKET_SIZE];
		int  test_data_len;
		uint32_t test_data_sent;
		uint32_t test_data_start;
		uint32_t total_data_sent;       // <-- MOD: Track total sent
} le_cbm_connection_t;

static le_cbm_connection_t le_cbm_connection;



static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
static volatile uint16_t i2c_buf_index = 0;
static volatile bool i2c_data_ready = false;

static uint16_t l2cap_cid = 0;
static hci_con_handle_t connection_handle = 0;
static bool can_send = false;

static void send_i2c_data_over_l2cap(void) {
		if (!i2c_data_ready || !can_send || le_cbm_connection.cid == 0) {
				return;
		}

		//int err = l2cap_send(le_cbm_connection.cid, i2c_buffer, I2C_BUFFER_SIZE);
		int err = l2cap_send(le_cbm_connection.cid, (uint8_t *) i2c_buffer, le_cbm_connection.test_data_len);
		if (err) {
				printf("Could not send now; wait for next can_send_now event\n");
				return;
		}

		//printf("Sent %d bytes over L2CAP\n", I2C_BUFFER_SIZE);
		i2c_buf_index = 0;
		i2c_data_ready = false;
		can_send = false;

		printf("Request next can send event to continue sending when ready\n");
		l2cap_request_can_send_now_event(le_cbm_connection.cid);
}




static void test_reset(le_cbm_connection_t * context){
		context->test_data_start = btstack_run_loop_get_time_ms();
		context->test_data_sent = 0;
		context->total_data_sent = 0;   // <-- MOD: Reset total sent
}

static void test_track_data(le_cbm_connection_t * context, int bytes_transferred){
		context->test_data_sent += bytes_transferred;
		// evaluate
		uint32_t now = btstack_run_loop_get_time_ms();
		uint32_t time_passed = now - context->test_data_start;
		if (time_passed < REPORT_INTERVAL_MS) return;
		int bytes_per_second = context->test_data_sent * 1000 / time_passed;
		printf("%c: %"PRIu32" bytes -> %u.%03u kB/s\n", context->name, context->test_data_sent, bytes_per_second / 1000, bytes_per_second % 1000);
		context->test_data_start = now;
		context->test_data_sent  = 0;
}

static bool advertisement_report_contains_name(const char * name, uint8_t * advertisement_report){
		const uint8_t * adv_data = gap_event_advertising_report_get_data(advertisement_report);
		uint8_t         adv_len  = gap_event_advertising_report_get_data_length(advertisement_report);
		uint16_t        name_len = (uint16_t) strlen(name);

		ad_context_t context;
		for (ad_iterator_init(&context, adv_len, adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
				uint8_t data_type    = ad_iterator_get_data_type(&context);
				uint8_t data_size    = ad_iterator_get_data_len(&context);
				const uint8_t * data = ad_iterator_get_data(&context);
				switch (data_type){
						case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
						case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
								if (data_size < name_len) break;
								if (memcmp(data, name, name_len) == 0) return true;
								break;
						default:
								break;
				}
		}
		return 0;
}

static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
		switch (event) {
				case I2C_SLAVE_RECEIVE:
						if (i2c_buf_index < le_cbm_connection.test_data_len) {
								i2c_buffer[i2c_buf_index++] = i2c_read_byte_raw(i2c);
#if 1
								if (i2c_buf_index == le_cbm_connection.test_data_len) {
										i2c_data_ready = true;
										l2cap_request_can_send_now_event(le_cbm_connection.cid);  // Add this
								}

#endif
						} else {
								//printf("Buffer full, discard extra bytes for now\n");
								i2c_read_byte_raw(i2c);
								//i2c_buf_index = 0;
						}
						break;
				case I2C_SLAVE_REQUEST:
						// No data sent back on I2C here
						i2c_write_byte_raw(i2c, 0);
						break;
				case I2C_SLAVE_FINISH:
						if (i2c_buf_index > 0) {
								i2c_data_ready = true;
						}
						// Do nothing
						break;
				default:
						break;
		}
}

static void setup_i2c_slave() {
		printf("slave setup\n");
		gpio_init(I2C_SLAVE_SDA_PIN);
		gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
		gpio_pull_up(I2C_SLAVE_SDA_PIN);

		gpio_init(I2C_SLAVE_SCL_PIN);
		gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
		gpio_pull_up(I2C_SLAVE_SCL_PIN);

		i2c_init(i2c0, I2C_BAUDRATE);
		i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
		// Set IRQ handler for PIO0 or PIO1 depending on the i2c_slave implementation
		//irq_set_exclusive_handler(PIO0_IRQ_0, i2c_slave_handler); // replace with actual
		//irq_set_exclusive_handler(PIO0_IRQ_0, i2c_slave_irq_handler);
		irq_set_priority(PIO0_IRQ_0, 0);  // Set highest priority
		irq_set_enabled(PIO0_IRQ_0, true);
}
static void le_cbm_client_start(void){
		sscanf_bd_addr("F4:C8:8A:16:EA:DF", cmdline_addr);
		printf("Connect to %s\n", bd_addr_to_str(cmdline_addr));
		state = TC_W4_CONNECT;
		gap_connect(cmdline_addr, 0);
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
		UNUSED(channel);
		UNUSED(size);

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
												printf("BTstack up, enabling advertising\n");
												uint16_t adv_int_min = 0x0030;
												uint16_t adv_int_max = 0x0030;
												uint8_t adv_type = 0;
												bd_addr_t null_addr;
												memset(null_addr, 0, 6);
												gap_advertisements_set_params(0x0030, 0x0030, 0, 0, null_addr, 0x07, 0x00);
												gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
												gap_advertisements_enable(1);
												printf("BTstack is up, advertising now...\n");
												//gap_advertisements_enable(1);
										}
										break;

										/*
										   case HCI_EVENT_META_GAP:
										   if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
										   if (state != TC_W4_CONNECT) return;
										   connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
										   conn_interval = gap_subevent_le_connection_complete_get_conn_interval(packet);
										   printf("Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
										   printf("Connection Latency: %u\n", gap_subevent_le_connection_complete_get_conn_latency(packet));
										   printf("Connect to performance test service.\n");
										   state = TC_W4_CHANNEL;
										//l2cap_cbm_create_channel(&packet_handler, connection_handle, TSPX_le_psm, cbm_receive_buffer, sizeof(cbm_receive_buffer), L2CAP_LE_AUTOMATIC_CREDITS, LEVEL_0, &l2cap_cid);
										//l2cap_accept_connection(cid); // Accept CBM channel

										break;
										*/
								case HCI_EVENT_DISCONNECTION_COMPLETE:
										printf("Disconnected\n");
										gap_advertisements_enable(1);
										connection_handle = 0;
										l2cap_cid = 0;
										can_send = false;
										i2c_buf_index = 0;
										i2c_data_ready = false;
										//if (state != TC_OFF) le_cbm_client_start();
										break;



								case L2CAP_EVENT_CBM_INCOMING_CONNECTION:
										printf("incoming connection\n");
										psm = l2cap_event_cbm_incoming_connection_get_psm(packet);
										cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
										if (psm != TSPX_le_psm) break;
										printf("L2CAP: Accepting incoming connection request for 0x%02x, PSM %02x dat_channel_buffer = %d\n", cid, psm, sizeof(data_channel_buffer));
										l2cap_cbm_accept_connection(cid, data_channel_buffer, sizeof(data_channel_buffer), initial_credits);
										break;



								case L2CAP_EVENT_CBM_CHANNEL_OPENED:

										l2cap_event_cbm_channel_opened_get_address(packet, event_address);
										status = l2cap_event_cbm_channel_opened_get_status(packet);

										if (status == ERROR_CODE_SUCCESS) {
												psm = l2cap_event_cbm_channel_opened_get_psm(packet);
												le_cbm_connection.cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
												le_cbm_connection.connection_handle = l2cap_event_cbm_channel_opened_get_handle(packet);
												le_cbm_connection.test_data_len = btstack_min(l2cap_event_cbm_channel_opened_get_remote_mtu(packet), sizeof(le_cbm_connection.test_data));
												printf("L2CAP: CBM Channel successfully opened: %s, handle 0x%04x, psm 0x%02x, local cid 0x%02x, remote cid 0x%02x\n",
																bd_addr_to_str(event_address), le_cbm_connection.connection_handle, psm, le_cbm_connection.cid,  little_endian_read_16(packet, 15));
												state = TC_TEST_DATA;
												printf("Test packet size: %u\n", le_cbm_connection.test_data_len);
												test_reset(&le_cbm_connection);
												l2cap_request_can_send_now_event(le_cbm_connection.cid);
										} else {
												printf("L2CAP: Connection failed. status code 0x%02x\n", status);
										}

										break;
								case L2CAP_EVENT_CAN_SEND_NOW:
										can_send = true;
										printf("Can send now\n");
										send_i2c_data_over_l2cap();
										/*
										   if (i2c_data_ready) {
										   printf("Sending L2cap data ..\n");
										   cid = l2cap_event_can_send_now_get_local_cid(packet);
										   printf("cid = 0x%02x  le_cbm_connection.cid = 0x%02x\n", cid, le_cbm_connection.cid);
										   if (cid == le_cbm_connection.cid) {
										   can_send = true;
										   send_i2c_data_over_l2cap();
										   }
										   } 
										// request another packet
										l2cap_request_can_send_now_event(le_cbm_connection.cid);
										*/
										break;


								case L2CAP_EVENT_CHANNEL_CLOSED:
										cid = l2cap_event_channel_closed_get_local_cid(packet);
										printf("L2CAP: Channel closed 0x%02x\n", cid);
										break;
								default:
										break;
						}
						break;

				case L2CAP_DATA_PACKET:
						printf("l2cap data packet\n");
						test_track_data(&le_cbm_connection, size);
						break;

				default:
						break;
		}
}

// Core 1 main loop (for I2C)
void core1_entry() {

		setup_i2c_slave();
		printf("In core waiting for the data\n");
		while (true) {
				if (i2c_data_ready) {
						// Process I2C data on this core
						printf("Core 1 received I2C data: %s\n", i2c_buffer);
						i2c_data_ready = false;
				}
				// Add a short delay to prevent a tight loop
				sleep_ms(1);
		}
}



static bd_addr_t my_random_static_addr;

void generate_random_static_addr(void) {
		for (int i = 0; i < 6; i++) {
				my_random_static_addr[i] = (uint8_t)(rand() & 0xFF);
		}
		// Force static random: bits 7-6 of addr[5] = 1
		my_random_static_addr[5] |= 0xC0;

		printf("Using random static addr: %s\n", bd_addr_to_str(my_random_static_addr));
}

int main(int argc, const char * argv[]){
		int arg;
		bool led_on = false;

		stdio_init_all();
		stdio_usb_init();

		if (cyw43_arch_init()) {
				printf("Failed to initialise CYW43\n");                    
				return -1; 
		}
		sleep_ms(2000);
		printf("Starting BLE CBM client...\n");      
		//hci_dump_init(hci_dump_embedded_stdout_get_instance());


		l2cap_init();

		// le data channel setup
		l2cap_cbm_register_service(&packet_handler, TSPX_le_psm, LEVEL_0);

		hci_event_callback_registration.callback = &packet_handler;
		hci_add_event_handler(&hci_event_callback_registration);


		// L2CAP control events  <<< REQUIRED
		l2cap_event_callback_registration.callback = &packet_handler;
		l2cap_add_event_handler(&l2cap_event_callback_registration);

		//generate_random_static_addr();
		//gap_random_address_set(my_random_static_addr);
#if 0
		// setup advertisements
		uint16_t adv_int_min = 0x0030;
		uint16_t adv_int_max = 0x0030;
		uint8_t adv_type = 0;
		bd_addr_t null_addr;
		memset(null_addr, 0, 6);
		gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x01);
		gap_advertisements_set_data(adv_data_len, (uint8_t *) adv_data);
		//gap_advertisements_enable(1);

		// Launch core 1 for I2C handling before starting Bluetooth
		//multicore_launch_core1(core1_entry);
#endif

		setup_i2c_slave();
		hci_power_control(HCI_POWER_ON);

		while (true) {
				async_context_poll(cyw43_arch_async_context());
				if (i2c_data_ready && can_send) {
						send_i2c_data_over_l2cap();
				}
		}
		printf("program ending\n");
		cyw43_arch_deinit();
		return 0;
}

