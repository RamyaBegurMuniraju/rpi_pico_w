#include <stdio.h>
#include "btstack.h"
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"

#define APP_AD_FLAGS 0x06

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

    if (packet_type != HCI_EVENT_PACKET) return;

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
}

int main() {
    stdio_init_all();

    if (cyw43_arch_init()) {
        printf("Failed to initialise CYW43\n");
        return -1;
    }

    printf("Starting BLE advertising example...\n");

    l2cap_init();
    sm_init();

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);

    // Turn on Bluetooth
    hci_power_control(HCI_POWER_ON);

    while (true) {
        async_context_poll(cyw43_arch_async_context());
        async_context_wait_for_work_until(cyw43_arch_async_context(), at_the_end_of_time);
    }

    return 0;
}

