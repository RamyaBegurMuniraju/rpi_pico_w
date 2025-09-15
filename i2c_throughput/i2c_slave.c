#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "i2c_slave.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

// ==== CONFIGURATION ====
#define I2C_SLAVE_ADDRESS     0x17
#define I2C_BAUDRATE          100000
#define I2C_SLAVE_SDA_PIN     PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SLAVE_SCL_PIN     PICO_DEFAULT_I2C_SCL_PIN
#define I2C_BUFFER_SIZE       1024
#define BLE_L2CAP_PSM         0x25

// Dummy BD_ADDR to connect to (replace with actual address)
static const char *remote_bd_addr_str = "F4:C8:8A:16:EA:DF";
static bd_addr_t remote_bd_addr;

// ==== I2C Buffer ====
static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
static uint16_t i2c_buf_index = 0;
static bool i2c_data_ready = false;

// ==== BLE Variables ====
static uint16_t l2cap_cid = 0;
static hci_con_handle_t connection_handle = 0;
static bool can_send = false;
static bool l2cap_connected = false;

// ==== I2C SLAVE HANDLER ====
static void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
			printf("I2c has received\n");
            if (i2c_buf_index < I2C_BUFFER_SIZE) {
                i2c_buffer[i2c_buf_index++] = i2c_read_byte_raw(i2c);
                if (i2c_buf_index == I2C_BUFFER_SIZE) {
                    i2c_data_ready = true;
                }
            } else {
				printf("Discard\n");
                i2c_read_byte_raw(i2c); // Discard
            }
            break;
        case I2C_SLAVE_REQUEST:
            i2c_write_byte_raw(i2c, 0); // Dummy
            break;
        case I2C_SLAVE_FINISH:
            break;
    }
}

// ==== SEND OVER L2CAP ====
static void send_i2c_data_over_l2cap(void) {
    if (!i2c_data_ready || !can_send || !l2cap_connected) return;

    int err = l2cap_send(l2cap_cid, i2c_buffer, I2C_BUFFER_SIZE);
    if (err == 0) {
        printf("Sent %d bytes over L2CAP\n", I2C_BUFFER_SIZE);
        i2c_buf_index = 0;
        i2c_data_ready = false;
        can_send = false;
        l2cap_request_can_send_now_event(l2cap_cid);
    } else {
        printf("L2CAP send error: %d\n", err);
    }
}

// ==== BLE EVENT HANDLER ====
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    UNUSED(channel);
    UNUSED(size);

    bd_addr_t event_address;
    uint16_t psm;
    uint16_t cid;
    uint16_t conn_interval;
    uint8_t status;
    hci_con_handle_t handle;
    uint16_t cid;

    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)) {
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                printf("BTstack up, connecting to %s\n", remote_bd_addr_str);
                sscanf_bd_addr(remote_bd_addr_str, remote_bd_addr);
                gap_connect(remote_bd_addr, 0);
            }
            break;

        case HCI_EVENT_LE_META:
            if (hci_event_le_meta_get_subevent_code(packet) == GAP_SUBEVENT_LE_CONNECTION_COMPLETE) {
                connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
                printf("Connected to peripheral! Handle: 0x%04x\n", connection_handle);
                l2cap_cbm_create_channel(&packet_handler, connection_handle, BLE_L2CAP_PSM, NULL, 0,
                                         L2CAP_LE_AUTOMATIC_CREDITS, LEVEL_0, &l2cap_cid);
            }
            break;

        case L2CAP_EVENT_CBM_CHANNEL_OPENED:
            status = l2cap_event_cbm_channel_opened_get_status(packet);
            if (status) {
                printf("L2CAP channel open failed, status 0x%02x\n", status);
                break;
            }
            l2cap_cid = l2cap_event_cbm_channel_opened_get_local_cid(packet);
            printf("L2CAP channel open success. CID: 0x%04x\n", l2cap_cid);
            l2cap_connected = true;
            l2cap_request_can_send_now_event(l2cap_cid);
            break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            if (l2cap_event_can_send_now_get_local_cid(packet) == l2cap_cid) {
                can_send = true;
                send_i2c_data_over_l2cap();
            }
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            printf("Disconnected\n");
            connection_handle = 0;
            l2cap_cid = 0;
            can_send = false;
            l2cap_connected = false;
            i2c_buf_index = 0;
            i2c_data_ready = false;
            break;

        default:
            break;
    }
}

// ==== INIT I2C ====
static void setup_i2c_slave(void) {
    gpio_init(I2C_SLAVE_SDA_PIN);
    gpio_set_function(I2C_SLAVE_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SDA_PIN);

    gpio_init(I2C_SLAVE_SCL_PIN);
    gpio_set_function(I2C_SLAVE_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SLAVE_SCL_PIN);

    i2c_init(i2c0, I2C_BAUDRATE);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, &i2c_slave_handler);
}

// ==== MAIN ====
int main() {
    stdio_init_all();
    stdio_usb_init();

    printf("Starting BLE L2CAP client + I2C slave combo\n");

    if (cyw43_arch_init()) {
        printf("Failed to initialize CYW43\n");
        return -1;
    }

    // BLE Init
    l2cap_init();

    static btstack_packet_callback_registration_t hci_event_callback_registration;
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    hci_power_control(HCI_POWER_ON);

    // I2C Init
    setup_i2c_slave();

    // Run loop
    while (true) {
        async_context_poll(cyw43_arch_async_context());
    }

    cyw43_arch_deinit();
    return 0;
}

