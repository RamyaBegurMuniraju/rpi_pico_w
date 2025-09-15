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



#define TEST_STREAM_DATA
#define TEST_PACKET_SIZE 1000
#define TOTAL_TEST_DATA_SIZE (2 * 1024 * 1024)  // <-- MOD: 2MB total data



#define I2C_SLAVE_ADDRESS 0x17
#define I2C_BAUDRATE 100000
#define I2C_SLAVE_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SLAVE_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

#define BLE_L2CAP_PSM 0x25
#define I2C_BUFFER_SIZE 1024


static enum {
    TC_OFF,
    TC_IDLE,
    TC_W4_SCAN_RESULT,
    TC_W4_CONNECT,
    TC_W4_CHANNEL,
    TC_TEST_DATA
} state = TC_OFF;

const uint16_t TSPX_le_psm = 0x25;

static bd_addr_t cmdline_addr;
static int cmdline_addr_found = 0;

// addr and type of device with correct name
static bd_addr_t      le_cbm_server_addr;
static bd_addr_type_t le_cbm_server_addr_type;

static hci_con_handle_t connection_handle;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static btstack_packet_callback_registration_t sm_event_callback_registration;

static uint8_t cbm_receive_buffer[TEST_PACKET_SIZE];

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
static uint16_t i2c_buf_index = 0;
static bool i2c_data_ready = false;

static uint16_t l2cap_cid = 0;
static hci_con_handle_t connection_handle = 0;
static bool can_send = false;

static void send_i2c_data_over_l2cap(void) {
    if (!i2c_data_ready || !can_send || l2cap_cid == 0) {
        return;
    }

    int err = l2cap_send(l2cap_cid, i2c_buffer, I2C_BUFFER_SIZE);
    if (err) {
        // Could not send now; wait for next can_send_now event
        return;
    }

    printf("Sent %d bytes over L2CAP\n", I2C_BUFFER_SIZE);
    i2c_buf_index = 0;
    i2c_data_ready = false;
    can_send = false;

    // Request next can send event to continue sending when ready
    l2cap_request_can_send_now_event(l2cap_cid);
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

#ifdef TEST_STREAM_DATA
static void streamer(void){
    if (le_cbm_connection.total_data_sent >= TOTAL_TEST_DATA_SIZE) {
        uint32_t total_time_ms = btstack_run_loop_get_time_ms() - le_cbm_connection.test_data_start;
        int final_kbps = (le_cbm_connection.total_data_sent * 1000) / total_time_ms;

        printf("\n*** Transfer Complete ***\n");
        printf("Total bytes sent: %"PRIu32"\n", le_cbm_connection.total_data_sent);
        printf("Total time: %"PRIu32" ms\n", total_time_ms);
        printf("Final throughput: %d.%03d kB/s\n", final_kbps / 1000, final_kbps % 1000);

        printf("Disconnecting...\n");
        gap_disconnect(le_cbm_connection.connection_handle);  // <-- MOD: Disconnect
        return;
    }

    le_cbm_connection.counter++;
    if (le_cbm_connection.counter > 'Z') le_cbm_connection.counter = 'A';
    memset(le_cbm_connection.test_data, le_cbm_connection.counter, le_cbm_connection.test_data_len);

    l2cap_send(le_cbm_connection.cid, (uint8_t *) le_cbm_connection.test_data, le_cbm_connection.test_data_len);

    test_track_data(&le_cbm_connection, le_cbm_connection.test_data_len);
    le_cbm_connection.total_data_sent += le_cbm_connection.test_data_len; // <-- MOD: Track total sent

    l2cap_request_can_send_now_event(le_cbm_connection.cid);
}
#endif




static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            if (i2c_buf_index < I2C_BUFFER_SIZE) {
                i2c_buffer[i2c_buf_index++] = i2c_read_byte_raw(i2c);
                if (i2c_buf_index == I2C_BUFFER_SIZE) {
                    i2c_data_ready = true;
                    //send_i2c_data_over_l2cap();
                    //i2c_buf_index = 0;
                }
            } else {
                printf("Buffer full, discard extra bytes for now\n");
                i2c_read_byte_raw(i2c);
				i2c_buf_index = 0;
            }
            break;
        case I2C_SLAVE_REQUEST:
            // No data sent back on I2C here
            i2c_write_byte_raw(i2c, 0);
            break;
        case I2C_SLAVE_FINISH:
            // Do nothing
            break;
        default:
            break;
    }
}

static void setup_i2c_slave() {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
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
                        le_cbm_client_start();
                    } else {
                        state = TC_OFF;
                    }
                    break;
                case HCI_EVENT_META_GAP:
                    if (hci_event_gap_meta_get_subevent_code(packet) != GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
                    if (state != TC_W4_CONNECT) return;
                    connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                    conn_interval = gap_subevent_le_connection_complete_get_conn_interval(packet);
                    printf("Connection Interval: %u.%02u ms\n", conn_interval * 125 / 100, 25 * (conn_interval & 3));
                    printf("Connection Latency: %u\n", gap_subevent_le_connection_complete_get_conn_latency(packet));
                    printf("Connect to performance test service.\n");
                    state = TC_W4_CHANNEL;
                    l2cap_cbm_create_channel(&packet_handler, connection_handle, TSPX_le_psm, cbm_receive_buffer,
                                             sizeof(cbm_receive_buffer), L2CAP_LE_AUTOMATIC_CREDITS, LEVEL_0, &le_cbm_connection.cid);
					break;
				case HCI_EVENT_DISCONNECTION_COMPLETE:
					printf("Disconnected %s\n", bd_addr_to_str(cmdline_addr));
					printf("Disconnected\n");
					connection_handle = 0;
					l2cap_cid = 0;
					can_send = false;
					i2c_buf_index = 0;
					i2c_data_ready = false;
					break;

					if (state != TC_OFF) le_cbm_client_start();
                    break;
                case L2CAP_EVENT_CBM_CHANNEL_OPENED:
                    l2cap_event_cbm_channel_opened_get_address(packet, event_address);
                    psm = l2cap_event_cbm_channel_opened_get_psm(packet);
                    cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
                    handle = l2cap_event_cbm_channel_opened_get_handle(packet);
                    status = l2cap_event_cbm_channel_opened_get_status(packet);
                    if (status == ERROR_CODE_SUCCESS) {
                        printf("L2CAP: CBM Channel opened: %s, handle 0x%04x, psm 0x%02x, local cid 0x%02x\n",
                               bd_addr_to_str(event_address), handle, psm, cid);
                        le_cbm_connection.cid = cid;
                        le_cbm_connection.connection_handle = handle;
                        le_cbm_connection.test_data_len = btstack_min(l2cap_event_cbm_channel_opened_get_remote_mtu(packet), sizeof(le_cbm_connection.test_data));
                        state = TC_TEST_DATA;
                       // printf("Test packet size: %u\n", le_cbm_connection.test_data_len);
                        //test_reset(&le_cbm_connection);
#ifdef TEST_STREAM_DATA
                        l2cap_request_can_send_now_event(le_cbm_connection.cid);
#endif
                    } else {
                        printf("L2CAP: Connection failed. status code 0x%02x\n", status);
                    }
                    break;
#ifdef TEST_STREAM_DATA
				case L2CAP_EVENT_CAN_SEND_NOW:
					if (i2c_data_ready) {
						cid = l2cap_event_can_send_now_get_local_cid(packet);
						if (cid == l2cap_cid) {
							can_send = true;
							send_i2c_data_over_l2cap();
						}
					} else {
						l2cap_request_can_send_now_event(le_cbm_connection.cid);
					}
					break;

					//streamer();
				//	break;
#endif
                case L2CAP_EVENT_CHANNEL_CLOSED:
                    cid = l2cap_event_channel_closed_get_local_cid(packet);
                    printf("L2CAP: Channel closed 0x%02x\n", cid);
                    break;
                default:
                    break;
            }
            break;

        case L2CAP_DATA_PACKET:
            test_track_data(&le_cbm_connection, size);
            break;

        default:
            break;
    }
}

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

int main(int argc, const char * argv[]){
    int arg;
	absolute_time_t next_blink_time;
    bool led_on = false;

	stdio_init_all();
	stdio_usb_init();

	if (cyw43_arch_init()) {
		printf("Failed to initialise CYW43\n");                    
		return -1; 
	}
	printf("Starting BLE CBM client...\n");      

    setup_i2c_slave();

	l2cap_init();
	sm_init();
	sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);

	hci_event_callback_registration.callback = &packet_handler;
	hci_add_event_handler(&hci_event_callback_registration);

	sm_event_callback_registration.callback = &sm_packet_handler;
	sm_add_event_handler(&sm_event_callback_registration);

	hci_power_control(HCI_POWER_ON);
	next_blink_time = make_timeout_time_ms(1000);

    while (true) {
        async_context_poll(cyw43_arch_async_context());

        if (absolute_time_diff_us(get_absolute_time(), next_blink_time) < 0) {
            led_on = !led_on;
            cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
            next_blink_time = make_timeout_time_ms(1000);
        }

        async_context_wait_for_work_until(cyw43_arch_async_context(), next_blink_time);
    }

    cyw43_arch_deinit();
    return 0;
}

