 #if 0
// btstack_config.h (Classic, Pico W / CYW43)

// Logging (optional)
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// ---- ACL buffer sizing (Classic macro family) ----
// Max BR/EDR HCI ACL payload is 1021 bytes (spec). Using that is safe.
#define HCI_ACL_PAYLOAD_SIZE         1021
// Some code uses _BUFFER_SIZE as alias:
#define HCI_ACL_BUFFER_SIZE          HCI_ACL_PAYLOAD_SIZE
// Pre-buffer for incoming packets (H:4 header etc.). 14 is safe & common.
#define HCI_INCOMING_PRE_BUFFER_SIZE 14

// Controller/host flow control (good on CYW43)
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define MAX_NR_CONTROLLER_ACL_BUFFERS 3

// L2CAP resources
#define MAX_NR_L2CAP_SERVICES  2
#define MAX_NR_L2CAP_CHANNELS  2
// --- Persistent storage sizes (required by BTstack TLV backends) ---
#define NVM_NUM_LINK_KEYS          16    // BR/EDR link keys (Classic)
#define NVM_NUM_DEVICE_DB_ENTRIES  16    // LE bonds (harmless even if BLE is off)
#define HCI_HOST_ACL_PACKET_NUM   3      // number of host ACL buffers

// Link key DB in RAM
#define MAX_NR_BTSTACK_LINK_KEY_DB_MEMORY_ENTRIES 16
#endif

#if 0
// btstack_config.h
#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H

// Logging (optional)
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Classic only (do NOT define ENABLE_BLE here)
#define ENABLE_CLASSIC

// ---- REQUIRED when ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL is set ----
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
// Host-side buffer pool sizes reported via HCI_Host_Buffer_Size
#define HCI_HOST_ACL_PACKET_LEN     1021   // spec max for BR/EDR ACL payload
#define HCI_HOST_ACL_PACKET_NUM     4
#define HCI_HOST_SCO_PACKET_LEN     120    // any small value is fine if you don't use SCO
#define HCI_HOST_SCO_PACKET_NUM     3
// Prebuffer for incoming HCI packets
#define HCI_INCOMING_PRE_BUFFER_SIZE 14

// L2CAP resources (tweak if needed)
#define MAX_NR_L2CAP_SERVICES   2
#define MAX_NR_L2CAP_CHANNELS   2

// Link key storage (TLV backend needs these defined)
#define NVM_NUM_LINK_KEYS           16
#define NVM_NUM_DEVICE_DB_ENTRIES   16


// ---- ACL buffer sizing (Classic macro family) ----
// Max BR/EDR HCI ACL payload is 1021 bytes (spec). Using that is safe.
#define HCI_ACL_PAYLOAD_SIZE         1021
// Some code uses _BUFFER_SIZE as alias:
#define HCI_ACL_BUFFER_SIZE          HCI_ACL_PAYLOAD_SIZE


#endif
#endif

#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H

// Logging (optional)
#define ENABLE_LOG_INFO
#define ENABLE_LOG_ERROR
#define ENABLE_PRINTF_HEXDUMP

// Classic only
//#define ENABLE_CLASSIC

// ---- CYW43 transport requirements ----
#define HCI_INCOMING_PRE_BUFFER_SIZE    14
#define HCI_OUTGOING_PRE_BUFFER_SIZE     4   // >= 4 (CYW43 header)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT     4   // must be multiple of 4
#define HCI_ACL_PAYLOAD_ALIGNMENT        4

// ---- Controller->Host flow control (recommended on CYW43) ----
#define ENABLE_HCI_CONTROLLER_TO_HOST_FLOW_CONTROL
#define HCI_HOST_ACL_PACKET_LEN       1021
#define HCI_HOST_ACL_PACKET_NUM          4
#define HCI_HOST_SCO_PACKET_LEN        120   // fine even if you don’t use SCO
#define HCI_HOST_SCO_PACKET_NUM          3

// ---- L2CAP / storage ----
//#define MAX_NR_L2CAP_SERVICES            2
//#define MAX_NR_L2CAP_CHANNELS            2
#define L2CAP_MTU                     65535 
#define NVM_NUM_LINK_KEYS               16
#define NVM_NUM_DEVICE_DB_ENTRIES       16

// ---- ACL buffer sizing (Classic macro family) ----
// Max BR/EDR HCI ACL payload is 1021 bytes (spec). Using that is safe.
//#define HCI_ACL_PAYLOAD_SIZE         1021
// Some code uses _BUFFER_SIZE as alias:
//#define HCI_ACL_BUFFER_SIZE          HCI_ACL_PAYLOAD_SIZE
//
//
//
#define MAX_NR_HCI_CONNECTIONS        1
#define MAX_NR_CONTROLLER_ACL_BUFFERS 4  // small bump
#define HCI_ACL_PAYLOAD_SIZE 65535 

#define HCI_ACL_RX_BUFFER_SIZE (HCI_ACL_PAYLOAD_SIZE + 8)
#define HCI_ACL_TX_BUFFERS     6     // if RAM allows; default can be low
//#define L2CAP_BR_EDR_MAX_MTU   1017  // Classic
#define MAX_NR_L2CAP_SERVICES  4
#define MAX_NR_L2CAP_CHANNELS  4

// === Classic Bluetooth ===
#define ENABLE_L2CAP_ERTM                1
#define ENABLE_SDP_SERVER                1
#define ENABLE_GAP_CLASSIC		 1



#endif



