
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
#define I2C_SDA_PIN     4//PICO_DEFAULT_I2C_SDA_PIN
#define I2C_SCL_PIN     5//PICO_DEFAULT_I2C_SCL_PIN
#define I2C_SLAVE_ADDR  0x17

// ===== BLE (LE CoC / CBM) =====
#define CBM_PSM         0x0081
#define INITIAL_CREDITS L2CAP_LE_AUTOMATIC_CREDITS
#define MPS_BYTES       65535	// typical LE CoC MPS
#define TX_MAX          1024	// send per l2cap_send() call (keep = MPS)

// ===== Ring buffer (power of 2 for cheap wrap) =====
//#define RING_SHIFT      13	// 2^13 = 8192 bytes ring, 8KB
#define RING_SHIFT      15            // 2^13 = 8192 bytes ring, 8KB
#define RING_SIZE       (1u << RING_SHIFT)
#define RING_MASK       (RING_SIZE - 1)

static uint8_t ring[RING_SIZE];
static volatile uint32_t ring_w = 0;	// ISR writes here (producer)
static volatile uint32_t ring_r = 0;	// BLE pump reads here (consumer)

static inline uint32_t
ring_count (void)
{
  return (ring_w - ring_r) & 0xFFFFFFFFu;
}

static inline bool
ring_empty (void)
{
  return ring_w == ring_r;
}

static inline uint8_t
ring_get (uint32_t idx)
{
  return ring[idx & RING_MASK];
}

static inline void
ring_put (uint8_t b)
{
  ring[ring_w++ & RING_MASK] = b;
}				// called in ISR

// ===== BLE state =====
typedef struct
{
  hci_con_handle_t handle;
  uint16_t cid;
  uint16_t remote_mtu;		// SDU MTU (BTstack fragments to MPS)
  bool can_send;
} lecoc_t;

static lecoc_t lecoc = { 0 };

static btstack_packet_callback_registration_t hci_cb;
static btstack_packet_callback_registration_t l2cap_cb;

// ====== I²C slave ISR: produce into ring, no prints, no BLE calls ======
static void
i2c_slave_isr (i2c_inst_t *i2c, i2c_slave_event_t event)
{
  switch (event)
    {
    case I2C_SLAVE_RECEIVE:
      // Drain FIFO
      while (i2c_get_read_available (i2c))
	{
	  uint8_t b = i2c_read_byte_raw (i2c);
	  // Overwrite oldest if ring would overflow (optional drop policy)
	  if (ring_count () >= RING_SIZE - 1)
	    {
	      //printf("drop oldest to keep running\n");
	      ring_r++;
	    }
	  ring_put (b);
	}
      break;

    case I2C_SLAVE_FINISH:
      // Transaction ended: nothing special to do – BLE will pull when it can
      break;

    case I2C_SLAVE_REQUEST:
      // Master read: return a dummy byte
      i2c_write_byte_raw (i2c, 0x00);
      break;

    default:
      break;
    }
}

// ====== BLE send pump: consume from ring in MPS-sized chunks ======
static void
ble_pump_send (void)
{
  if (!lecoc.cid || !lecoc.can_send)
    {
      return;
    }

  // Limit one SDU per CAN_SEND_NOW to play nice with controller/credits.
  // If you want to push harder, you can loop and send multiple, but always
  // re-request CAN_SEND_NOW to avoid starving other stack work.
  if (!ring_empty ())
    {
      // Decide how many bytes to send now
      uint16_t mtu = lecoc.remote_mtu ? lecoc.remote_mtu : TX_MAX;
      if (mtu > TX_MAX)
	mtu = TX_MAX;		// keep at MPS for 1:1 credit usage
      uint32_t avail = ring_count ();
      uint16_t n = (avail >= mtu) ? mtu : (uint16_t) avail;

      // Copy out to a small TX buffer
      static uint8_t tx[TX_MAX];
      for (uint16_t i = 0; i < n; i++)
	{
	  tx[i] = ring_get (ring_r++);
	}

      int err = l2cap_send (lecoc.cid, tx, n);
      if (err)
	{
	  printf
	    ("// Couldn't send now (no credits/ACL buffers). Roll back read index and retry later\n");
	  ring_r -= n;
	  l2cap_request_can_send_now_event (lecoc.cid);
	  return;
	}

      // We used our turn; will get another CAN_SEND_NOW if credits remain
      lecoc.can_send = false;
      l2cap_request_can_send_now_event (lecoc.cid);	// keep pipeline full
    }
}

void
check_le_features (void)
{
  printf("Sending LE Read Local Supported Features command...\n");
  hci_send_cmd (&hci_le_read_local_supported_features);
  printf("Expecting opcode = 0x%04x\n", HCI_OPCODE_HCI_LE_READ_LOCAL_SUPPORTED_FEATURES);
}

// ====== BLE events ======
static void
ble_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet,
	     uint16_t size)
{
  (void) channel;
  (void) size;

  switch (packet_type)
    {
    case HCI_EVENT_PACKET:
      {
	uint8_t evt = hci_event_packet_get_type (packet);
	switch (evt)
	  {
	  case BTSTACK_EVENT_STATE:
	    if (btstack_event_state_get_state (packet) == HCI_STATE_WORKING)
	      {
		printf ("// Advertise as LE CoC sink\n");
#if 1
		const uint8_t adv[] =
		  { 0x02, 0x01, 0x06, 0x0E, 0x09, 'I', '2', 'C', ' ', 'B',
'L', 'E', ' ', 'B', 'r', 'i', 'd', 'g', 'e' };
		check_le_features ();
		gap_advertisements_set_params (0x30, 0x30, 0, 0, (bd_addr_t)
					       {
					       0}, 0x07, 0);
		gap_advertisements_set_data (sizeof (adv), (uint8_t *) adv);
		gap_advertisements_enable (1);
#endif
	      }
	    break;

	  case HCI_EVENT_LE_META:
	    {
	      uint8_t sub = hci_event_le_meta_get_subevent_code (packet);
	      if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
		{
		 printf("connection complete\n");
		  lecoc.handle =
		    hci_subevent_le_connection_complete_get_connection_handle
		    (packet);
		  gap_le_set_phy (lecoc.handle, 0, 0x02, 0x02, 0);	// ask 2M/2M
		}
	      if (sub == HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE) {
	      	uint8_t status  = hci_subevent_le_phy_update_complete_get_status(packet);
           	uint16_t handle = hci_subevent_le_phy_update_complete_get_connection_handle(packet);
            	uint8_t tx_phy  = hci_subevent_le_phy_update_complete_get_tx_phy(packet);
            	uint8_t rx_phy  = packet[7];
            	printf("LE PHY Update Complete: handle=0x%04x, status=%u, tx_phy=%u, rx_phy=%u\n",
                   	handle, status, tx_phy, rx_phy);
	      }

	      break;
	    }
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


	  case L2CAP_EVENT_CBM_INCOMING_CONNECTION:
	    {
	      printf ("accept incoming connection\n");
	      uint16_t psm =
		l2cap_event_cbm_incoming_connection_get_psm (packet);
	      if (psm == CBM_PSM)
		{
		  uint16_t cid =
		    l2cap_event_cbm_incoming_connection_get_local_cid
		    (packet);
		  static uint8_t l2cap_buf[TX_MAX * 2];
		  l2cap_cbm_accept_connection (cid, l2cap_buf,
					       sizeof (l2cap_buf),
					       INITIAL_CREDITS);
		}
	      break;
	    }

	  case L2CAP_EVENT_CBM_CHANNEL_OPENED:
	    {
	      uint8_t status =
		l2cap_event_cbm_channel_opened_get_status (packet);
	      if (status == ERROR_CODE_SUCCESS)
		{
		  lecoc.cid =
		    l2cap_event_cbm_channel_opened_get_local_cid (packet);
		  lecoc.remote_mtu =
		    l2cap_event_cbm_channel_opened_get_remote_mtu (packet);
		  lecoc.can_send = false;
		  // Kick the pump
		  l2cap_request_can_send_now_event (lecoc.cid);
		}
	      else
		{
		  lecoc.cid = 0;
		}
	      break;
	    }

	  case L2CAP_EVENT_CAN_SEND_NOW:
	    //printf("can send now\n");
	    lecoc.can_send = true;
	    ble_pump_send ();
	    break;

	  case HCI_EVENT_DISCONNECTION_COMPLETE:
	    lecoc.cid = 0;
	    lecoc.handle = 0;
	    lecoc.remote_mtu = 0;
	    lecoc.can_send = false;
	    // Optional: reset ring
	    ring_r = ring_w = 0;
	    break;

	  default:
	    break;
	  }
	break;
      }

    default:
      break;
    }
}

// ===== Setup =====
static void
setup_i2c_slave (void)
{
  gpio_set_function (I2C_SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function (I2C_SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up (I2C_SDA_PIN);
  gpio_pull_up (I2C_SCL_PIN);

  i2c_slave_init (I2C_PORT, I2C_SLAVE_ADDR, &i2c_slave_isr);
}

int
main (void)
{
  stdio_init_all ();
  stdio_usb_init ();
  sleep_ms (1000);

  if (cyw43_arch_init ())
    {
      printf ("CYW43 init failed\n");
      return -1;
    }

  // BLE init
  l2cap_init ();
  hci_cb.callback = &ble_handler;
  l2cap_cb.callback = &ble_handler;
  hci_add_event_handler (&hci_cb);
  l2cap_add_event_handler (&l2cap_cb);
  l2cap_cbm_register_service (&ble_handler, CBM_PSM, LEVEL_0);

  setup_i2c_slave ();
  hci_power_control (HCI_POWER_ON);

  // Lightweight periodic stats (1 Hz)
  absolute_time_t last = get_absolute_time ();
  uint32_t last_r = 0;
  while (true)
    {
      async_context_poll (cyw43_arch_async_context ());

      if (absolute_time_diff_us (last, get_absolute_time ()) >= 1000000)
	{
	  uint32_t cnt = ring_count ();
	  uint32_t pushed = (ring_r - last_r) & 0xFFFFFFFFu;
	  //printf("Ring: avail=%u, drained=%u B/s, mtu=%u\n", (unsigned)cnt, (unsigned)pushed, lecoc.remote_mtu);
	  last = get_absolute_time ();
	  last_r = ring_r;
	  ble_pump_send ();
	  //can_send = false;
	}
      sleep_ms (1);
    }
}
