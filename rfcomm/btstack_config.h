#pragma once
// ---- BTstack: Classic RFCOMM/BR-EDR ----
#define ENABLE_RFCOMM

// REQUIRED by BTstack core:
#define HCI_ACL_PAYLOAD_SIZE         1021     // >=48; 1021 is max for BR/EDR on CYW43


#define HAVE_EMBEDDED_TIME_MS
#define HAVE_BTSTACK_STDIN 0
#define NVM_NUM_LINK_KEYS          8
#define NVM_NUM_DEVICE_DB_ENTRIES  8


// ---- Transport (CYW43) required defines ----
#define HCI_INCOMING_PRE_BUFFER_SIZE   4      // >= 4
#define HCI_OUTGOING_PRE_BUFFER_SIZE   4      // >= 4 (CYW43 header)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT   4      // must be a multiple of 4
					      
// RFCOMM capacities
#define MAX_NR_RFCOMM_SERVICES   4   // ≥1
#define MAX_NR_RFCOMM_CHANNELS   3   // ≥1, data channels

// SDP records capacity (needed when you register an SPP record)
#define MAX_SDP_RECORDS          4   // ≥1

#define MAX_NR_HCI_CONNECTIONS         2
#define MAX_NR_L2CAP_SERVICES          4     // RFCOMM needs L2CAP service underneath
#define MAX_NR_L2CAP_CHANNELS          4

#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE_FOR_RFCOMM
#define ENABLE_L2CAP_ENHANCED_RETRANSMISSION_MODE
#define MAX_NR_SERVICE_RECORD_ITEMS	6 
#define MAX_NR_RFCOMM_MULTIPLEXERS	2

