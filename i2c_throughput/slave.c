#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_slave.h"
#include "btstack.h"
#include "pico/cyw43_arch.h"


#define I2C_SLAVE_ADDR 0x17
#define I2C_BAUDRATE 100000
#define I2C_SLAVE_SDA_PIN PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SLAVE_SCL_PIN PICO_DEFAULT_I2C_SCL_PIN

#define BLE_L2CAP_PSM 0x25
#define I2C_BUFFER_SIZE 64 
#define I2C_PORT	i2c0

static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
static uint16_t i2c_buf_index = 0;
static bool i2c_data_ready = false;

static uint16_t l2cap_cid = 0;
static hci_con_handle_t connection_handle = 0;
static bool can_send = false;

static void send_i2c_data_over_l2cap(void) {
    if (!i2c_data_ready || l2cap_cid == 0) {
        return;
    }

    int err = l2cap_send(l2cap_cid, i2c_buffer, I2C_BUFFER_SIZE);
    if (err) {
        // Could not send now; wait for next can_send_now event
        return;
    }

    printf("Sent %d bytes over L2CAP\n", I2C_BUFFER_SIZE);
    i2c_data_ready = false;

    // Request next can send event to continue sending when ready
    l2cap_request_can_send_now_event(l2cap_cid);
}



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
				i2c_buf_index = 0;
				//i2c_read_byte_raw(i2c);
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
    i2c_slave_init(i2c0, I2C_SLAVE_ADDR, &i2c_slave_handler);
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    uint8_t status;
    hci_con_handle_t handle;
    uint16_t cid;

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("BLE stack working, waiting for connections\n");
                        gap_advertisements_enable(1);
            }
            break;

        case HCI_EVENT_CONNECTION_COMPLETE:
            status = hci_event_connection_complete_get_status(packet);
            if (status) {
                printf("Connection failed with status 0x%02x\n", status);
                break;
            }
            handle = hci_event_connection_complete_get_connection_handle(packet);
            connection_handle = handle;
            printf("Connected, handle 0x%04x\n", handle);
            // Create L2CAP channel here
            l2cap_cbm_create_channel(&packet_handler, handle, BLE_L2CAP_PSM, NULL, 0, L2CAP_LE_AUTOMATIC_CREDITS, LEVEL_0, &l2cap_cid);
            break;

        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
            status = l2cap_event_cbm_channel_opened_get_status(packet);
            if (status) {
                printf("L2CAP channel open failed, status 0x%02x\n", status);
                break;
            }
            cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
            l2cap_cid = cid;
            printf("L2CAP channel opened, CID 0x%04x\n", cid);
            break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            cid = l2cap_event_can_send_now_get_local_cid(packet);
            if (cid == l2cap_cid) {
                can_send = true;
                send_i2c_data_over_l2cap();
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Disconnected\n");
            connection_handle = 0;
            l2cap_cid = 0;
            can_send = false;
            i2c_buf_index = 0;
            i2c_data_ready = false;
            break;

        default:
            break;
    }
}

int main() {


	stdio_init_all();
    stdio_usb_init();

    printf("Starting BLE L2CAP server with streaming I2C slave data\n");

    if (cyw43_arch_init()) {
        printf("Failed to init CYW43\n");
        return -1;
    }

    l2cap_init();

    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);

    setup_i2c_slave();

    while (true) {
        async_context_poll(cyw43_arch_async_context());
        // Could add LED blink or other low priority tasks here
    }

    cyw43_arch_deinit();
    return 0;
}

