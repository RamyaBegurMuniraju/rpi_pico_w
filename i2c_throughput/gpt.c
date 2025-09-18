#include <inttypes.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "btstack.h"
#include "btstack_run_loop.h"
#include "btstack_run_loop_embedded.h"
#include "btstack_defines.h"
#include "btstack_event.h"

#include "hardware/i2c.h"
#include "i2c_slave.h"
#include "hardware/sync.h"   // <-- FIX: use Pico SDK sync primitives

// ---------------- Config ----------------
#define I2C_SLAVE_ADDRESS   0x17
#define I2C_BAUDRATE        100000
#define I2C_SLAVE_SDA_PIN   PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SLAVE_SCL_PIN   PICO_DEFAULT_I2C_SCL_PIN

#define I2C_BUFFER_SIZE     256
#define BLE_L2CAP_PSM       0x25
#define REPORT_INTERVAL_MS  3000
// ----------------------------------------

static enum {
    TC_OFF,
    TC_IDLE,
    TC_W4_CHANNEL,
    TC_TEST_DATA
} state = TC_OFF;

typedef struct {
    char name;
    hci_con_handle_t connection_handle;
    uint16_t cid;
    int counter;
    char test_data[I2C_BUFFER_SIZE];
    int test_data_len;
    uint32_t test_data_sent;
    uint32_t test_data_start;
    uint32_t total_data_sent;
} le_cbm_connection_t;

static le_cbm_connection_t le_cbm_connection;
static btstack_packet_callback_registration_t hci_event_callback_registration;

static uint8_t cbm_receive_buffer[I2C_BUFFER_SIZE];
static uint8_t data_channel_buffer[I2C_BUFFER_SIZE];

static uint16_t initial_credits = L2CAP_LE_AUTOMATIC_CREDITS;

static uint8_t i2c_buf_a[I2C_BUFFER_SIZE];
static uint8_t i2c_buf_b[I2C_BUFFER_SIZE];
static uint8_t *write_buf = i2c_buf_a;
static uint8_t *send_buf  = i2c_buf_b;
static volatile uint16_t write_index = 0;
static volatile uint16_t send_len = 0;
static volatile bool send_buf_ready = false;

static bool can_send = false;

// ---------------- I2C ----------------
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            if (write_index < I2C_BUFFER_SIZE) {
                write_buf[write_index++] = i2c_read_byte_raw(i2c);
            } else {
                i2c_read_byte_raw(i2c); // discard
            }
            break;

        case I2C_SLAVE_REQUEST:
            i2c_write_byte_raw(i2c, 0); // nothing to send back
            break;

        case I2C_SLAVE_FINISH:
            if (write_index > 0) {
                uint32_t flags = save_and_disable_interrupts();
                uint8_t *tmp = write_buf;
                write_buf = send_buf;
                send_buf = tmp;
                send_len = write_index;
                write_index = 0;
                send_buf_ready = true;
                restore_interrupts(flags);

                if (le_cbm_connection.cid) {
                    l2cap_request_can_send_now_event(le_cbm_connection.cid);
                }
            }
            break;

        default:
            break;
    }
}

static void setup_i2c_slave() {
    printf("I2C slave setup\n");

    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);

    irq_set_priority(PIO0_IRQ_0, 0);  // Highest priority
    irq_set_enabled(PIO0_IRQ_0, true);
}

// ---------------- BLE ----------------
static void test_reset(le_cbm_connection_t * context){
    context->test_data_start = btstack_run_loop_get_time_ms();
    context->test_data_sent = 0;
    context->total_data_sent = 0;
}

static void test_track_data(le_cbm_connection_t * context, int bytes_transferred){
    context->test_data_sent += bytes_transferred;
    uint32_t now = btstack_run_loop_get_time_ms();
    uint32_t time_passed = now - context->test_data_start;
    if (time_passed < REPORT_INTERVAL_MS) return;
    int bytes_per_second = context->test_data_sent * 1000 / time_passed;
    printf("%c: %"PRIu32" bytes -> %u.%03u kB/s\n",
           context->name, context->test_data_sent,
           bytes_per_second / 1000, bytes_per_second % 1000);
    context->test_data_start = now;
    context->test_data_sent  = 0;
}

static void send_i2c_data_over_l2cap(void) {
	if (!send_buf_ready || !can_send || le_cbm_connection.cid == 0) return;
	int err = l2cap_send(le_cbm_connection.cid, send_buf, send_len);
	if (err) {
		printf("cid = 0x%2x\n", le_cbm_connection.cid);
		l2cap_request_can_send_now_event(le_cbm_connection.cid);
		return;
	}

	printf("Sent %u bytes over L2CAP\n", send_len);

	uint32_t flags = save_and_disable_interrupts();
	send_buf_ready = false;
	send_len = 0;
	can_send = false;
	restore_interrupts(flags);

	l2cap_request_can_send_now_event(le_cbm_connection.cid);
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_address;
    uint16_t psm;
    uint16_t cid;
    hci_con_handle_t handle;
    uint8_t status;

    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        printf("BTstack up, advertising...\n");
                        gap_advertisements_enable(1);
                    }
                    break;

                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    printf("Disconnected\n");
                    gap_advertisements_enable(1);
                    le_cbm_connection.cid = 0;
                    can_send = false;
                    break;

                case L2CAP_EVENT_CBM_INCOMING_CONNECTION:
                    psm = l2cap_event_cbm_incoming_connection_get_psm(packet);
                    cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
                    if (psm != BLE_L2CAP_PSM) break;
                    printf("Incoming L2CAP CBM connection, accepting...\n");
                    l2cap_cbm_accept_connection(cid, data_channel_buffer, sizeof(data_channel_buffer), initial_credits);
                    break;

                case L2CAP_EVENT_CBM_CHANNEL_OPENED:
                    l2cap_event_cbm_channel_opened_get_address(packet, event_address);
                    status = l2cap_event_cbm_channel_opened_get_status(packet);
                    if (status == ERROR_CODE_SUCCESS) {
                        psm = l2cap_event_cbm_channel_opened_get_psm(packet);
                        cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
                        handle = l2cap_event_cbm_channel_opened_get_handle(packet);
                        le_cbm_connection.cid = cid;
                        le_cbm_connection.connection_handle = handle;
                        le_cbm_connection.test_data_len =
                            btstack_min(l2cap_event_cbm_channel_opened_get_remote_mtu(packet),
                                        sizeof(le_cbm_connection.test_data));
                        state = TC_TEST_DATA;
                        printf("L2CAP CBM channel opened: handle=0x%04x cid=0x%02x psm=0x%02x\n",
                               handle, cid, psm);
                        test_reset(&le_cbm_connection);
                        l2cap_request_can_send_now_event(cid);
                    } else {
                        printf("L2CAP open failed, status=0x%02x\n", status);
                    }
                    break;

                case L2CAP_EVENT_CAN_SEND_NOW:
					printf("Le cbm connect event\n");
                    can_send = true;
                    send_i2c_data_over_l2cap();
                    break;

                case L2CAP_EVENT_CHANNEL_CLOSED:
                    cid = l2cap_event_channel_closed_get_local_cid(packet);
                    printf("L2CAP channel closed cid=0x%02x\n", cid);
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

// ---------------- Main ----------------
int main(void) {
    stdio_init_all();
    stdio_usb_init();

    if (cyw43_arch_init()) {
        printf("Failed to init CYW43\n");
        return -1;
    }

    printf("Starting BLE + I2C bridge...\n");

    l2cap_init();
    l2cap_cbm_register_service(&packet_handler, BLE_L2CAP_PSM, LEVEL_0);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Advertisements
    const uint8_t adv_data[] = {
        0x02, 0x01, 0x06,
        0x0C, 0x09, 'I','2','C','-','B','L','E',' ','B','r','i','d','g','e',
    };
    gap_advertisements_set_data(sizeof(adv_data), (uint8_t*)adv_data);
    gap_advertisements_enable(1);

    setup_i2c_slave();
    hci_power_control(HCI_POWER_ON);

    while (true) {
        async_context_poll(cyw43_arch_async_context());
        if (send_buf_ready && can_send) {
            send_i2c_data_over_l2cap();
        }
    }

    cyw43_arch_deinit();
    return 0;
}

