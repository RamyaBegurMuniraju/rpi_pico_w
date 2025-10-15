
#ifndef _PICO_BTSTACK_BTSTACK_CONFIG_H
#define _PICO_BTSTACK_BTSTACK_CONFIG_H


// Classic only

// ---- CYW43 transport requirements ----
#define HCI_INCOMING_PRE_BUFFER_SIZE    14
#define HCI_OUTGOING_PRE_BUFFER_SIZE     4   // >= 4 (CYW43 header)
#define HCI_ACL_CHUNK_SIZE_ALIGNMENT     4   // must be multiple of 4
#define HCI_ACL_PAYLOAD_ALIGNMENT        4

//#define HCI_HOST_ACL_PACKET_LEN       1021
//#define HCI_HOST_ACL_PACKET_LEN       1695
#define HCI_HOST_ACL_PACKET_NUM          4


//#define HCI_ACL_PAYLOAD_SIZE            1021


// ---- L2CAP / storage ----
//#define L2CAP_MTU                     65535 
#define NVM_NUM_LINK_KEYS               4
#define NVM_NUM_DEVICE_DB_ENTRIES       4

#define MAX_NR_HCI_CONNECTIONS        1
#define MAX_NR_CONTROLLER_ACL_BUFFERS 4  // small bump
//#define HCI_ACL_PAYLOAD_SIZE 1024 
#define HCI_ACL_PAYLOAD_SIZE 4096
//ACL buffer size if HCI_ACL_PAYLOAD_SIZE + 8 
//#define HCI_ACL_BUFFER_SIZE            (HCI_ACL_PAYLOAD_SIZE + 8) 

#define HCI_ACL_RX_BUFFER_SIZE (HCI_ACL_PAYLOAD_SIZE + 8)
#define HCI_ACL_TX_BUFFERS     6     // if RAM allows; default can be low
#define MAX_NR_L2CAP_SERVICES  4
#define MAX_NR_L2CAP_CHANNELS  4

// === Classic Bluetooth ===
#define ENABLE_L2CAP_ERTM                1


#endif



