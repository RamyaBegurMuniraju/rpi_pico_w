#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"

#define I2C_SLAVE_ADDRESS 0x17
#define I2C_BUFFER_SIZE 64

static uint8_t i2c_buffer[I2C_BUFFER_SIZE];
static volatile bool i2c_data_ready = false;
static volatile size_t i2c_buf_index = 0;

static uint16_t l2cap_cid = 0;
static bool ble_connected = false;
static bool ble_can_send = false;
void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event);

void i2c_slave_handler(i2c_inst_t *i2c, i2c_slave_event_t event) {
    switch (event) {
        case I2C_SLAVE_RECEIVE:
            if (i2c_buf_index < I2C_BUFFER_SIZE) {
                i2c_buffer[i2c_buf_index++] = i2c_read_byte_raw(i2c);
            }
            break;
        case I2C_SLAVE_FINISH:
            if (i2c_buf_index > 0) {
                i2c_data_ready = true;
            }
            break;
        default:
            break;
    }
}

void send_i2c_data_over_ble() {
    if (i2c_data_ready && ble_can_send && l2cap_cid) {
        l2cap_send(l2cap_cid, i2c_buffer, i2c_buf_index);
        printf("Sent %u bytes over BLE\n", i2c_buf_index);
        i2c_buf_index = 0;
        i2c_data_ready = false;
        ble_can_send = false;
        l2cap_request_can_send_now_event(l2cap_cid);
    }
}

void setup_i2c_slave() {
    gpio_init(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);

    gpio_init(PICO_DEFAULT_I2C_SCL_PIN);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    i2c_init(i2c0, 100 * 1000);
    i2c_slave_init(i2c0, I2C_SLAVE_ADDRESS, i2c_slave_handler);

    irq_set_priority(I2C0_IRQ, 0); // Prioritize I2C
}

void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size) {
    uint16_t cid;
    switch (packet_type) {
        case HCI_EVENT_PACKET:
            switch (hci_event_packet_get_type(packet)) {
                case BTSTACK_EVENT_STATE:
                    if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING) {
                        printf("BLE ready, advertising...\n");
                        gap_advertisements_enable(1);
                    }
                    break;
                case GAP_EVENT_ADVERTISING_REPORT:
                    break;
                case HCI_EVENT_LE_META:
                    switch (hci_event_le_meta_get_subevent_code(packet)) {
                        case GAP_SUBEVENT_LE_CONNECTION_COMPLETE:
                            ble_connected = true;
                            break;
                    }
                    break;
                case L2CAP_EVENT_CREDIT_BASED_CHANNEL_OPENED:
                    if (l2cap_event_credit_based_channel_opened_get_status(packet) == ERROR_CODE_SUCCESS) {
                        l2cap_cid = l2cap_event_credit_based_channel_opened_get_local_cid(packet);
                        ble_can_send = true;
                        printf("L2CAP channel opened: cid=0x%04x\n", l2cap_cid);
                    }
                    break;
                case L2CAP_EVENT_CAN_SEND_NOW:
                    ble_can_send = true;
                    send_i2c_data_over_ble();
                    break;
                case HCI_EVENT_DISCONNECTION_COMPLETE:
                    printf("Disconnected\n");
                    ble_connected = false;
                    l2cap_cid = 0;
                    break;
            }
            break;
        default:
            break;
    }
}

int main() {
    stdio_init_all();
    sleep_ms(2000);

    if (cyw43_arch_init()) {
        printf("CYW43 init failed!\n");
        return 1;
    }

    l2cap_init();
    l2cap_register_packet_handler(packet_handler);
    l2cap_create_le_credit_based_connection(packet_handler, 0x25, 64, 64, &l2cap_cid);

    sm_init();
    hci_event_callback_registration_t hci_event_callback;
    hci_event_callback.callback = packet_handler;
    hci_add_event_handler(&hci_event_callback);

    hci_power_control(HCI_POWER_ON);

    setup_i2c_slave();

    while (1) {
        async_context_poll(cyw43_arch_async_context());
        if (i2c_data_ready && ble_can_send) {
            send_i2c_data_over_ble();
        }
    }

    return 0;
}

