#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include "btstack.h"
#include "classic/rfcomm.h"
#include "classic/sdp_server.h"
#include "classic/sdp_util.h"   // spp_create_sdp_record

// ---------------- SPP / RFCOMM ----------------
#define DEVICE_NAME      "PicoW SPP 2MB"
#define RFCOMM_CHANNEL   1

// Stream exactly 2 MB once per connection (no giant .bss arrays)
#define TOTAL_BYTES      (2 * 1024 * 1024)
#define TX_BUF_CAP       1024              // temp TX buffer per packet (<= negotiated MTU)

static uint8_t  tx_buf[TX_BUF_CAP];
static uint32_t bytes_sent   = 0;           // progress
static uint16_t rfcomm_cid   = 0;
static uint16_t rfcomm_mtu   = 512;
static bool     link_up      = false;

// SDP buffer
static uint8_t spp_service_buffer[150];

// Event handler registration
static btstack_packet_callback_registration_t hci_cb;

// Forward decls
static void request_more_send(void);
static void send_next_chunk(void);

// Deterministic pattern (0..255 repeating) generated on the fly
static inline void fill_chunk(uint8_t *dst, uint32_t offset, uint16_t len){
    for (uint16_t i = 0; i < len; ++i){
        dst[i] = (uint8_t)((offset + i) & 0xFF);
    }
}

static void request_more_send(void){
    if (rfcomm_cid){
        rfcomm_request_can_send_now_event(rfcomm_cid);
    }
}

static void send_next_chunk(void){
    if (!link_up || !rfcomm_cid) return;

    if (bytes_sent >= TOTAL_BYTES){
        printf("DONE: sent %u bytes\n", (unsigned)TOTAL_BYTES);
        rfcomm_disconnect(rfcomm_cid);
        return;
    }

    // choose chunk size: min(remaining, rfcomm_mtu, TX_BUF_CAP)
    uint32_t remaining = TOTAL_BYTES - bytes_sent;
    uint16_t to_send   = (remaining > rfcomm_mtu) ? rfcomm_mtu : (uint16_t)remaining;
    if (to_send > TX_BUF_CAP) to_send = TX_BUF_CAP;

    fill_chunk(tx_buf, bytes_sent, to_send);

    int err = rfcomm_send(rfcomm_cid, tx_buf, to_send);
    if (err == 0){
        bytes_sent += to_send;
        if (bytes_sent < TOTAL_BYTES){
            request_more_send();
        }else{
            printf("DONE: sent %u bytes\n", (unsigned)TOTAL_BYTES);
            rfcomm_disconnect(rfcomm_cid); // optional
        }
    }else{
        // Not ready yet; ask again
        request_more_send();
    }
}

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    (void)channel; (void)size;
    if (packet_type != HCI_EVENT_PACKET) return;

    switch (hci_event_packet_get_type(packet)){
        case BTSTACK_EVENT_STATE:
            if (btstack_event_state_get_state(packet) == HCI_STATE_WORKING){
                gap_set_local_name(DEVICE_NAME);
                gap_set_class_of_device(0x5A020C);   // general computer/phone
                gap_discoverable_control(1);
                gap_connectable_control(1);
                printf("BT ready: %s discoverable, RFCOMM ch %d\n", DEVICE_NAME, RFCOMM_CHANNEL);
            }
            break;

        case HCI_EVENT_PIN_CODE_REQUEST: {
            bd_addr_t addr;
            hci_event_pin_code_request_get_bd_addr(packet, addr);
            gap_pin_code_response(addr, "0000");
            break;
        }

        case RFCOMM_EVENT_INCOMING_CONNECTION: {
            bd_addr_t event_addr;
            rfcomm_event_incoming_connection_get_bd_addr(packet, event_addr);
            uint8_t ch  = rfcomm_event_incoming_connection_get_server_channel(packet);
            uint16_t cid = rfcomm_event_incoming_connection_get_rfcomm_cid(packet);
            printf("RFCOMM incoming from %s on ch %u -> cid 0x%04x\n",
                   bd_addr_to_str(event_addr), ch, cid);
            rfcomm_accept_connection(cid);
            break;
        }

        case RFCOMM_EVENT_CHANNEL_OPENED: {
            uint8_t status = rfcomm_event_channel_opened_get_status(packet);
            if (status){
                printf("RFCOMM open failed, status=0x%02x\n", status);
                break;
            }
            rfcomm_cid = rfcomm_event_channel_opened_get_rfcomm_cid(packet);
            rfcomm_mtu = rfcomm_event_channel_opened_get_max_frame_size(packet);
            link_up    = true;
            bytes_sent = 0;  // reset per connection
            printf("RFCOMM opened: cid=%u, mtu=%u\n", rfcomm_cid, rfcomm_mtu);
            request_more_send();             // kick off streaming
            break;
        }

        case RFCOMM_EVENT_CAN_SEND_NOW:
            send_next_chunk();
            break;

        case RFCOMM_EVENT_CHANNEL_CLOSED:
            printf("RFCOMM closed (cid=%u). Sent %u bytes this session.\n",
                   rfcomm_cid, (unsigned)bytes_sent);
            link_up    = false;
            rfcomm_cid = 0;
            // Allow reconnects
            gap_discoverable_control(1);
            gap_connectable_control(1);
            break;

        default:
            break;
    }
}

static void bt_setup(void){
    l2cap_init();
    rfcomm_init();

    // SDP record for SPP
    sdp_init();
    memset(spp_service_buffer, 0, sizeof(spp_service_buffer));
    uint32_t handle = sdp_create_service_record_handle();
    spp_create_sdp_record(spp_service_buffer, handle, RFCOMM_CHANNEL, "SPP Streamer 2MB");
    sdp_register_service(spp_service_buffer);

    // RFCOMM server
    uint8_t st = rfcomm_register_service(packet_handler, RFCOMM_CHANNEL, 0xffff);
    printf("rfcomm_register_service -> 0x%02x\n", st);

    // HCI events
    hci_cb.callback = &packet_handler;
    hci_add_event_handler(&hci_cb);

    hci_power_control(HCI_POWER_ON);
}

int main(void){
    stdio_init_all();
    sleep_ms(1000);
    printf("\n[Pico W] SPP 2MB Streamer (server, ch %d)\n", RFCOMM_CHANNEL);

    if (cyw43_arch_init()){
        printf("CYW43 init failed\n");
        return -1;
    }
    //cyw43_arch_enable_btstack();  // important when using pico_btstack_cyw43

    bt_setup();

    while (true){
        tight_loop_contents();
    }
    return 0;
}

