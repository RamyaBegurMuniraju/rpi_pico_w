#if 0
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/i2c.h"
#include "i2c_slave.h"

#include "btstack.h"
#include "btstack_run_loop_embedded.h"

// ===== I2C config =====
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SCL_PIN     PICO_DEFAULT_I2C_SCL_PIN
#define I2C_SLAVE_ADDR  0x17

// ===== BLE (LE CoC / CBM) =====
#define CBM_PSM         0x0081
#define INITIAL_CREDITS L2CAP_LE_AUTOMATIC_CREDITS
#define TX_MAX          1024        // send per l2cap_send() call
#define CHUNK_SIZE      1024        // I2C chunk size + ACK unit

// ===== Single staging buffer (no ring) =====
static uint8_t  chunk_buf[CHUNK_SIZE];
static volatile uint16_t rx_count      = 0;    // bytes received into chunk_buf
static volatile uint16_t tx_off        = 0;    // bytes already sent over BLE from chunk_buf
static volatile bool     chunk_ready   = false;// true once rx_count == CHUNK_SIZE
// I2C 1-byte status/ACK: 1=READY to start next 1024, 0=BUSY (still filling or sending)
static volatile uint8_t  status_byte   = 1;

// ===== BLE state =====
typedef struct {
    hci_con_handle_t handle;
    uint16_t cid;
    uint16_t remote_mtu;   // SDU MTU from peer
    bool     can_send;
} lecoc_t;

static lecoc_t lecoc = {0};
static btstack_packet_callback_registration_t hci_cb;
static btstack_packet_callback_registration_t l2cap_cb;

// ========== I²C slave ISR (no ring) ==========
static void i2c_slave_isr(i2c_inst_t *i2c, i2c_slave_event_t event){
    switch (event){
        case I2C_SLAVE_RECEIVE: {
            while (i2c_get_read_available(i2c)){
                // If a new chunk starts and we were READY, immediately go BUSY
                if (rx_count == 0 && status_byte == 1){
                    status_byte = 0; // BUSY
                }

                uint8_t b = i2c_read_byte_raw(i2c);

                if (!chunk_ready){
                    if (rx_count < CHUNK_SIZE){
                        chunk_buf[rx_count++] = b;
                        if (rx_count == CHUNK_SIZE){
                            // Completed 1024B intake; trigger BLE send
                            chunk_ready = true;
                            if (lecoc.cid) {
                                l2cap_request_can_send_now_event(lecoc.cid);
                            }
                        }
                    } else {
                        // Master violated contract (sent >1024 before ACK) — drop extras
                        (void)b;
                    }
                } else {
                    // Previous chunk not sent yet; drop incoming (contract violation)
                    (void)b;
                }
            }
        } break;

        case I2C_SLAVE_REQUEST:
            // Master polls our ACK: 0x01=READY for next 1024, 0x00=BUSY
            i2c_write_byte_raw(I2C_PORT, status_byte);
            break;

        case I2C_SLAVE_FINISH:
        default:
            break;
    }
}

// ========== BLE send (event-driven) ==========
static void ble_send_chunk_if_possible(void){
    if (!lecoc.cid || !lecoc.can_send) return;
    if (!chunk_ready) return;

    // Per-send length: min(peer MTU, TX_MAX, remaining)
    uint16_t eff_mtu = lecoc.remote_mtu ? lecoc.remote_mtu : TX_MAX;
    if (eff_mtu > TX_MAX) eff_mtu = TX_MAX;
    if (eff_mtu == 0) eff_mtu = TX_MAX;

    uint16_t remaining = (rx_count > tx_off) ? (rx_count - tx_off) : 0;
    if (remaining == 0) return;

    uint16_t send_len = remaining < eff_mtu ? remaining : eff_mtu;

    int err = l2cap_send(lecoc.cid, chunk_buf + tx_off, send_len);
    if (err){
        // Controller busy; we'll get another CAN_SEND_NOW later
        return;
    }

    tx_off += send_len;

    if (tx_off >= rx_count){
        // Entire 1024-byte chunk has been sent out over BLE:
        rx_count    = 0;
        tx_off      = 0;
        chunk_ready = false;
        status_byte = 1;      // ACK next 1024 to the I2C master (READY)
    }

    // One send per CAN_SEND_NOW; if more remains, ask for another slot
    lecoc.can_send = false;
    if (chunk_ready){
        l2cap_request_can_send_now_event(lecoc.cid);
    }
}

// ========== BLE events ==========
static void ble_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t evt = hci_event_packet_get_type(packet);
    switch (evt){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                // ready
		printf("BLE is Up\n");
            }
            break;

        case HCI_EVENT_LE_META: {
            uint8_t sub = hci_event_le_meta_get_subevent_code(packet);
            if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE){
                lecoc.handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                gap_le_set_phy(lecoc.handle, 0, 0x02, 0x02, 0); // request 2M PHY
            }
        } break;

	  case HCI_EVENT_COMMAND_COMPLETE:
	     printf("Command complete opcode=0x%04x, status=%u\n",
        hci_event_command_complete_get_command_opcode(packet),
        hci_event_command_complete_get_return_parameters(packet)[0]);

	    if (hci_event_command_complete_get_command_opcode (packet) ==
		HCI_OPCODE_HCI_LE_READ_LOCAL_SUPPORTED_FEATURES)
	      {

		const uint8_t *params =
		  hci_event_command_complete_get_return_parameters (packet);
		// The payload starts with the status (1 byte)
		uint8_t status = params[0];
		if (status == 0)
		  {
		    // The LE Features bitmask starts after the status byte
		    const uint8_t *features = &params[1];
		    printf
		      ("LE Features: 0x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		       features[7], features[6], features[5], features[4],
		       features[3], features[2], features[1], features[0]);

		    // The LE Features are defined in Core Spec v5.2, Vol 6, Part B, Section 4.6.1
		    // The mask is Little Endian, so features[0] is the first byte.
		    // Bit 1 (0x02) indicates LE 2M PHY support.
		    if ((features[0] & 0x02) != 0)
		      {
			printf ("  - Supports LE 2M PHY\n");
		      }
		    // Bit 4 (0x10) indicates LE Coded PHY support.
		    if ((features[0] & 0x10) != 0)
		      {
			printf ("  - Supports LE Coded PHY\n");
		      }
		hci_send_cmd(&hci_read_local_version_information);
		  }
		else
		  {
		    printf
		      ("LE Read Local Supported Features command failed with status: 0x%02x\n",
		       status);
		  }
	      }

	    if (hci_event_command_complete_get_command_opcode (packet) == HCI_OPCODE_HCI_READ_LOCAL_VERSION_INFORMATION) {
		    const uint8_t *params = hci_event_command_complete_get_return_parameters(packet);

		    uint8_t  status       = params[0];
		    uint8_t  hci_version  = params[1];
		    uint16_t hci_revision = little_endian_read_16(params, 2);
		    uint8_t  lmp_version  = params[4];
		    uint16_t manufacturer = little_endian_read_16(params, 5);
		    uint16_t lmp_subv     = little_endian_read_16(params, 7);

		    printf("Local Version Info:\n");
		    printf("  Status: %u\n", status);
		    printf("  HCI Version: %u\n", hci_version);
		    printf("  HCI Revision: 0x%04x\n", hci_revision);
		    printf("  LMP Version: %u\n", lmp_version);
		    printf("  Manufacturer: 0x%04x\n", manufacturer);
		    printf("  LMP Subversion: 0x%04x\n", lmp_subv);

		const uint8_t adv[] =
		  { 0x02, 0x01, 0x06, 0x0E, 0x09, 'I', '2', 'C', ' ', 'B',
'L', 'E', ' ', 'B', 'r', 'i', 'd', 'g', 'e' };
		    gap_advertisements_set_params (0x30, 0x30, 0, 0, (bd_addr_t)
				    {
				    0}, 0x07, 0);
		    gap_advertisements_set_data (sizeof (adv), (uint8_t *) adv);
		    gap_advertisements_enable (1);
	    }
	    if (hci_event_command_complete_get_command_opcode (packet) ==
		HCI_OPCODE_HCI_LE_SET_EVENT_MASK)
	      printf ("LE Event Mask set complete.\n");
	    if (hci_event_command_complete_get_command_opcode (packet) ==
		HCI_OPCODE_HCI_LE_CONNECTION_UPDATE)
	      printf ("hci_opcode_hci_le_connection_update \n");
	    if (hci_event_command_complete_get_command_opcode (packet) ==
		HCI_OPCODE_HCI_LE_SET_DEFAULT_PHY)
	      printf ("hci_opcode_hci_le_set_default_phy\n");
	    break;


        case L2CAP_EVENT_CBM_INCOMING_CONNECTION: {
            if (l2cap_event_cbm_incoming_connection_get_psm(packet) == CBM_PSM){
                uint16_t cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
                static uint8_t l2cap_buf[TX_MAX * 2];
                l2cap_cbm_accept_connection(cid, l2cap_buf, sizeof(l2cap_buf), INITIAL_CREDITS);
            }
        } break;

        case L2CAP_EVENT_CBM_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
            if (status == ERROR_CODE_SUCCESS){
                lecoc.cid        = l2cap_event_cbm_channel_opened_get_local_cid(packet);
                lecoc.remote_mtu = l2cap_event_cbm_channel_opened_get_remote_mtu(packet);
                lecoc.can_send   = false;
                l2cap_request_can_send_now_event(lecoc.cid);
            } else {
                lecoc.cid = 0;
            }
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            lecoc.can_send = true;
            ble_send_chunk_if_possible();
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            lecoc.cid = 0;
            lecoc.handle = 0;
            lecoc.remote_mtu = 0;
            lecoc.can_send = false;
            // Reset chunk gate so master can restart cleanly
            rx_count = 0; tx_off = 0; chunk_ready = false; status_byte = 1;
            break;

        default: break;
    }
}

// ===== Setup =====
static void setup_i2c_slave(void){
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_isr);
}

int main(void){
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(300);

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }

    // Keep Wi-Fi off to minimize contention
    cyw43_arch_disable_sta_mode();
    cyw43_arch_disable_ap_mode();

    // BLE init
    l2cap_init();
    hci_cb.callback   = &ble_handler;
    l2cap_cb.callback = &ble_handler;
    hci_add_event_handler(&hci_cb);
    l2cap_add_event_handler(&l2cap_cb);
    l2cap_cbm_register_service(&ble_handler, CBM_PSM, LEVEL_0);

    setup_i2c_slave();
    hci_power_control(HCI_POWER_ON);

    while (true){
        async_context_poll(cyw43_arch_async_context());
        sleep_us(100);
    }
}
#endif



#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "hardware/i2c.h"
#include "hardware/irq.h"
#include "i2c_slave.h"

#include "btstack.h"
#include "btstack_run_loop_embedded.h"

// ===== I2C config =====
#define I2C_PORT        i2c0
#define I2C_SDA_PIN     PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SCL_PIN     PICO_DEFAULT_I2C_SCL_PIN
#define I2C_SLAVE_ADDR  0x17

// ===== BLE (LE CoC / CBM) =====
#define CBM_PSM         0x0081
#define INITIAL_CREDITS L2CAP_LE_AUTOMATIC_CREDITS
#define TX_MAX          1024        // send per l2cap_send() call
#define CHUNK_SIZE      1024        // I2C chunk size + ACK unit

// ===== Single staging buffer (no ring) =====
static uint8_t  chunk_buf[CHUNK_SIZE];
static volatile uint16_t rx_count      = 0;    // bytes received into chunk_buf
static volatile uint16_t tx_off        = 0;    // bytes already sent over BLE
static volatile bool     chunk_ready   = false;// true once rx_count == CHUNK_SIZE
// I2C 1-byte status/ACK: 1=READY to start next 1024, 0=BUSY (filling or sending)
static volatile uint8_t  status_byte   = 1;

// kick the TX path from main (avoid calling into BT from ISR)
static volatile bool     kick_send     = false;

// ===== BLE state =====
typedef struct {
    hci_con_handle_t handle;
    uint16_t cid;
    uint16_t remote_mtu;   // SDU MTU from peer
    bool     can_send;
} lecoc_t;

static lecoc_t lecoc = {0};
static btstack_packet_callback_registration_t hci_cb;
static btstack_packet_callback_registration_t l2cap_cb;

// ========== I²C slave ISR (no ring) ==========
static void i2c_slave_isr(i2c_inst_t *i2c, i2c_slave_event_t event){
    switch (event){
        case I2C_SLAVE_RECEIVE: {
            while (i2c_get_read_available(i2c)){
                // If a new chunk starts and we were READY, immediately go BUSY
                if (rx_count == 0 && status_byte == 1){
                    status_byte = 0; // BUSY
                }

                uint8_t b = i2c_read_byte_raw(i2c);

                if (!chunk_ready){
                    if (rx_count < CHUNK_SIZE){
                        chunk_buf[rx_count++] = b;
                        if (rx_count == CHUNK_SIZE){
                            // Completed 1024B intake; flag to trigger BLE send (outside ISR)
                            chunk_ready = true;
                            kick_send   = true;
                        }
                    } else {
                        // Master violated 1024B contract — drop extras
                        (void)b;
                    }
                } else {
                    // Previous chunk not sent yet; drop incoming (contract violation)
                    (void)b;
                }
            }
        } break;

        case I2C_SLAVE_REQUEST:
            // Master polls our ACK: 0x01=READY for next 1024, 0x00=BUSY
            i2c_write_byte_raw(I2C_PORT, status_byte);
            break;

        case I2C_SLAVE_FINISH:
        default:
            break;
    }
}

// ========== BLE send (event-driven) ==========
static void ble_send_chunk_if_possible(void){
    if (!lecoc.cid || !lecoc.can_send) return;
    if (!chunk_ready) return;

    // Per-send length: min(peer MTU, TX_MAX, remaining)
    uint16_t eff_mtu = lecoc.remote_mtu ? lecoc.remote_mtu : TX_MAX;
    if (eff_mtu > TX_MAX) eff_mtu = TX_MAX;
    if (eff_mtu == 0)     eff_mtu = TX_MAX;

    uint16_t remaining = (rx_count > tx_off) ? (rx_count - tx_off) : 0;
    if (remaining == 0) return;

    uint16_t send_len = remaining < eff_mtu ? remaining : eff_mtu;

    int err = l2cap_send(lecoc.cid, chunk_buf + tx_off, send_len);
    if (err){
        // Controller busy or no credits: re-arm and try again later
        l2cap_request_can_send_now_event(lecoc.cid);
        return;
    }

    tx_off += send_len;

    if (tx_off >= rx_count){
        // Entire 1024-byte chunk has been sent out over BLE:
        rx_count    = 0;
        tx_off      = 0;
        chunk_ready = false;
        status_byte = 1;      // ACK next 1024 to the I2C master (READY)
    }

    // One send per CAN_SEND_NOW; if more remains, ask for another slot
    lecoc.can_send = false;
    if (chunk_ready){
        l2cap_request_can_send_now_event(lecoc.cid);
    }
}

// ========== BLE events ==========
static void ble_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    uint8_t evt = hci_event_packet_get_type(packet);
    switch (evt){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                // Start advertising immediately (don’t rely on other callbacks)
                const uint8_t adv[] = {
                    0x02, 0x01, 0x06,
                    0x0E, 0x09, 'I','2','C',' ','B','L','E',' ','B','r','i','d','g','e'
                };
                gap_advertisements_set_params(0x30, 0x30, 0, 0, (bd_addr_t){0}, 0x07, 0);
                gap_advertisements_set_data(sizeof(adv), (uint8_t*)adv);
                gap_advertisements_enable(1);
                printf("BLE up, advertising\n");
            }
            break;

        case HCI_EVENT_LE_META: {
            uint8_t sub = hci_event_le_meta_get_subevent_code(packet);
            if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE){
                lecoc.handle = hci_subevent_le_connection_complete_get_connection_handle(packet);
                gap_le_set_phy(lecoc.handle, 0, 0x02, 0x02, 0); // request 2M PHY
            }
        } break;

        case L2CAP_EVENT_CBM_INCOMING_CONNECTION: {
            if (l2cap_event_cbm_incoming_connection_get_psm(packet) == CBM_PSM){
                uint16_t cid = l2cap_event_cbm_incoming_connection_get_local_cid(packet);
                static uint8_t l2cap_buf[TX_MAX * 2];
                l2cap_cbm_accept_connection(cid, l2cap_buf, sizeof(l2cap_buf), INITIAL_CREDITS);
            }
        } break;

        case L2CAP_EVENT_CBM_CHANNEL_OPENED: {
            uint8_t status = l2cap_event_cbm_channel_opened_get_status(packet);
            if (status == ERROR_CODE_SUCCESS){
                lecoc.cid        = l2cap_event_cbm_channel_opened_get_local_cid(packet);
                lecoc.remote_mtu = l2cap_event_cbm_channel_opened_get_remote_mtu(packet);
                lecoc.can_send   = false;
                l2cap_request_can_send_now_event(lecoc.cid);
                printf("LE CoC opened: cid=%u, mtu=%u\n", lecoc.cid, lecoc.remote_mtu);
            } else {
                lecoc.cid = 0;
            }
        } break;

        case L2CAP_EVENT_CAN_SEND_NOW:
            lecoc.can_send = true;
            ble_send_chunk_if_possible();
            break;

        case HCI_EVENT_DISCONNECTION_COMPLETE:
            lecoc.cid = 0;
            lecoc.handle = 0;
            lecoc.remote_mtu = 0;
            lecoc.can_send = false;
            // Reset chunk gate so master can restart cleanly
            rx_count = 0; tx_off = 0; chunk_ready = false; status_byte = 1;
            break;

        default: break;
    }
}

// ===== Setup =====
static void setup_i2c_slave(void){
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);
    i2c_slave_init(I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_isr);

    // IMPORTANT: let BT/HCI pre-empt I2C to avoid bus read errors
    // RP2040 has 4 effective NVIC levels; 0x80 = medium, 0xC0 = lowest.
    irq_set_priority(I2C0_IRQ, 0x80);
    irq_set_enabled(I2C0_IRQ, true);
}

int main(void){
    stdio_init_all();
    stdio_usb_init();
    sleep_ms(300);

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }

    // Keep Wi-Fi off to minimize contention
    cyw43_arch_disable_sta_mode();
    cyw43_arch_disable_ap_mode();

    // BLE init
    l2cap_init();
    hci_cb.callback   = &ble_handler;
    l2cap_cb.callback = &ble_handler;
    hci_add_event_handler(&hci_cb);
    l2cap_add_event_handler(&l2cap_cb);
    l2cap_cbm_register_service(&ble_handler, CBM_PSM, LEVEL_0);

    setup_i2c_slave();
    hci_power_control(HCI_POWER_ON);

    while (true){
        // service CYW43/BT
        async_context_poll(cyw43_arch_async_context());

        // kick TX path if ISR flagged a ready chunk
        if (kick_send && lecoc.cid){
            kick_send = false;
            l2cap_request_can_send_now_event(lecoc.cid);
        }

        sleep_us(100);
    }
}

