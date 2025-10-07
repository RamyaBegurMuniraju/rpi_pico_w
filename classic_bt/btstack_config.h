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

#if 1
#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H


// Classic only

// ---- CYW43 transport requirements ----
#define HCI_INCOMING_PRE_BUFFER_SIZE    14
#define HCI_OUTGOING_PRE_BUFFER_SIZE     4   // >= 4 (CYW43 header)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT     4   // must be multiple of 4
#define HCI_ACL_PAYLOAD_ALIGNMENT        4

#define HCI_HOST_ACL_PACKET_LEN       1021
#define HCI_HOST_ACL_PACKET_NUM          4

// ---- L2CAP / storage ----
#define L2CAP_MTU                     65535 
#define NVM_NUM_LINK_KEYS               4
#define NVM_NUM_DEVICE_DB_ENTRIES       4

#define MAX_NR_HCI_CONNECTIONS        1
#define MAX_NR_CONTROLLER_ACL_BUFFERS 4  // small bump
#define HCI_ACL_PAYLOAD_SIZE 1024 

#define HCI_ACL_RX_BUFFER_SIZE (HCI_ACL_PAYLOAD_SIZE + 8)
#define HCI_ACL_TX_BUFFERS     6     // if RAM allows; default can be low
#define MAX_NR_L2CAP_SERVICES  4
#define MAX_NR_L2CAP_CHANNELS  4

// === Classic Bluetooth ===
#define ENABLE_L2CAP_ERTM                1



#endif


#endif

#if 0

#pragma once


// Enable Classic stack
#define ENABLE_CLASSIC
#define ENABLE_L2CAP
#define HAVE_BTSTACK_STDIN


// Link key storage for pairing (TLV in RAM by default)
#define NVM_NUM_LINK_KEYS 16


// Max incoming L2CAP connections/services
//#define MAX_NO_L2CAP_SERVICES 4
//#define MAX_NO_L2CAP_CHANNELS 4

#define MAX_NR_L2CAP_CHANNELS 4
#define MAX_NR_L2CAP_SERVICES 4


// L2CAP MTU — Classic default 672; you can try 1024
#define L2CAP_DEFAULT_MTU 672


// Optional: printf hexdumps if you route HCI dumps to stdout in examples
#define ENABLE_PRINTF_HEXDUMP


// Buffer sizes
#define HCI_ACL_PAYLOAD_SIZE (1024 + 8)





#endif



#if 0

#pragma once

// ---- Enable Classic L2CAP on Pico W (CYW43) ----
#define ENABLE_CLASSIC
#define ENABLE_L2CAP

// Pairing/link keys (required for Classic)
#define NVM_NUM_LINK_KEYS        16

// L2CAP resources
#define MAX_NR_L2CAP_SERVICES    4
#define MAX_NR_L2CAP_CHANNELS    4
#define L2CAP_DEFAULT_MTU        672      // Classic default; can try 1024 if both sides support it

// ---- CYW43 transport requirements ----
// CYW43 HCI wrapper needs a 4-byte pre-buffer (packet header)
#define HCI_OUTGOING_PRE_BUFFER_SIZE     4
#define HCI_INCOMING_PRE_BUFFER_SIZE     14   // safe default used by many ports

// ACL payload sizing—keep >= your largest L2CAP SDU + headers.
// 1024+8 is usually fine; you can lower to 672+8 if you keep default MTU.
#define HCI_ACL_PAYLOAD_SIZE             (1024 + 8)

// Alignment must be a multiple of 4 for CYW43 DMA path
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT     4

// Optional: enable printf-style hexdumps if you want them in logs
#define ENABLE_PRINTF_HEXDUMP
#define MAX_NR_HCI_CONNECTIONS 1

#endif



#if 0

// btstack_config.h  — Pico W, Classic L2CAP, 1 ACL link
#pragma once

// --- Enable Classic stack pieces ---
#define ENABLE_CLASSIC
#define ENABLE_L2CAP

// Optional logging
//#define ENABLE_LOG_INFO
//#define ENABLE_LOG_ERROR
//#define ENABLE_PRINTF_HEXDUMP

// --- CYW43 transport requirements (must keep) ---
#define HCI_OUTGOING_PRE_BUFFER_SIZE   4
#define HCI_INCOMING_PRE_BUFFER_SIZE   14
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT   4

// --- Connections / resources (keep small on Pico W) ---
#define MAX_NR_HCI_CONNECTIONS         1
#define MAX_NR_L2CAP_SERVICES          2
#define MAX_NR_L2CAP_CHANNELS          2

// --- MTU / buffers ---
// Classic BR/EDR max ACL payload is 1021 bytes.
// Keep this at 1021 (safe) or at least >= 4 + your L2CAP MTU (e.g. 676 for MTU 672)
#define HCI_ACL_PAYLOAD_SIZE           1021

// Use default Classic L2CAP MTU (~672). You don't need to override it.
#define L2CAP_BR_EDR_MAX_MTU        1017  // (optional)

// --- Pairing DB sizes ---
#define NVM_NUM_LINK_KEYS              16
#define NVM_NUM_DEVICE_DB_ENTRIES      16
#endif
