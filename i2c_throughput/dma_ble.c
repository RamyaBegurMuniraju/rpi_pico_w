#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"

#include "hardware/i2c.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

#include "btstack.h"
#include "pico/btstack_cyw43.h"	// transport glue

// ---------------- I2C SLAVE + DMA CONFIG ----------------
#define I2C_PORT        i2c0
#define SDA_PIN         4
#define SCL_PIN         5
#define I2C_SLAVE_ADDR  0x17

// Ring buffer (no DMA address wrap; we manage wrap in software)
#define RING_BYTES          (192 * 1024)	// 192 KB
#define DMA_BLOCK_BYTES     8192	// 8 KB per DMA IRQ

// ---- DLE knobs ----
#define DLE_TX_OCTETS   251	// max LL payload bytes
#define DLE_TX_TIME_US  2120	// time in microseconds (controller will clamp)

static uint8_t ring_buf[RING_BYTES];
static volatile uint32_t rd_offset = 0;	// consumed by BLE (bytes)

// Single RX-DMA channel and write offset in BYTES
static int dma_ch = -1;
static volatile uint32_t wr_off = 0;	// producer write offset (bytes)
static uint32_t next_dst = 0;	// next DMA dst offset
static volatile uint32_t dma_irq_count = 0;	// debug

// ---------------- BLE LE CoC (CBM) CONFIG ----------------
// Use LE Credit-Based Connection-Oriented Channel
#define CBM_PSM             0x0081
//#define LECOC_TX_MTU        1024             // SDU size we send per call
#define LECOC_TX_MTU        4096	// SDU size we send per call
#define INITIAL_CREDITS     L2CAP_LE_AUTOMATIC_CREDITS


#define TESTING  1
static uint16_t lecoc_cid = 0;
static uint16_t lecoc_remote_mtu = LECOC_TX_MTU;
static bool lecoc_can_send = false;

static uint8_t tx_buf[LECOC_TX_MTU];

static btstack_packet_callback_registration_t hci_cb;
static btstack_packet_callback_registration_t l2cap_cb;

// ---------------- Utils ----------------
static inline uint32_t
min_u32 (uint32_t a, uint32_t b)
{
  return a < b ? a : b;
}

static inline uint32_t
ring_avail (void)
{
  uint32_t w = wr_off, r = rd_offset;
  return (w >= r) ? (w - r) : (RING_BYTES - (r - w));
}

static inline void
ring_consume (uint32_t n)
{
  rd_offset = (rd_offset + n) % RING_BYTES;
}

static inline uint32_t
ring_free (void)
{
  return (RING_BYTES - 1) - ring_avail ();
}

static volatile bool dma_paused = false;


// ---------------- DMA: one channel -> ring ----------------
static void
start_dma_transfer (int ch, uint32_t dst_off_bytes)
{
  volatile uint32_t *src = &i2c_get_hw (I2C_PORT)->data_cmd;	// pops RX FIFO

  dma_channel_config c = dma_channel_get_default_config (ch);
  channel_config_set_transfer_data_size (&c, DMA_SIZE_8);
  channel_config_set_read_increment (&c, false);
  channel_config_set_write_increment (&c, true);
  channel_config_set_dreq (&c,
			   I2C_PORT == i2c0 ? DREQ_I2C0_RX : DREQ_I2C1_RX);

  // DO NOT enable address ring in DMA config — we ring-wrap in software.
  dma_channel_configure (ch, &c, &ring_buf[0] + dst_off_bytes,	// destination in ring
			 src,	// I2C RX FIFO
			 DMA_BLOCK_BYTES,	// one full block per IRQ
			 false);

  dma_channel_set_irq0_enabled (ch, true);
  dma_start_channel_mask (1u << ch);
}



static inline void
maybe_resume_dma (void)
{
  if (dma_paused && ring_free () >= DMA_BLOCK_BYTES)
    {
      dma_paused = false;
      start_dma_transfer (dma_ch, next_dst);
    }
}

static void __isr
dma_irq_handler (void)
{
  if (dma_channel_get_irq0_status (dma_ch))
    {
      dma_channel_acknowledge_irq0 (dma_ch);

      // one block landed at next_dst
      wr_off = (wr_off + DMA_BLOCK_BYTES) % RING_BYTES;
      next_dst = (next_dst + DMA_BLOCK_BYTES) % RING_BYTES;
      dma_irq_count++;

      if (ring_free () < DMA_BLOCK_BYTES)
	{
	  dma_paused = true;	// <- backpressure kicks in
	  return;		// do NOT start next transfer
	}
      start_dma_transfer (dma_ch, next_dst);
    }
}

static void
dma_init_and_start (void)
{
  dma_ch = dma_claim_unused_channel (true);

  irq_set_exclusive_handler (DMA_IRQ_0, dma_irq_handler);
  irq_set_enabled (DMA_IRQ_0, true);
  dma_channel_set_irq0_enabled (dma_ch, true);

  wr_off = 0;
  rd_offset = 0;
  next_dst = 0;

  start_dma_transfer (dma_ch, next_dst);
}

// ---------------- I2C SLAVE SETUP ----------------
static void
i2c_slave_init_dma (void)
{
  gpio_set_function (SDA_PIN, GPIO_FUNC_I2C);
  gpio_set_function (SCL_PIN, GPIO_FUNC_I2C);
  gpio_pull_up (SDA_PIN);
  gpio_pull_up (SCL_PIN);

  // Power/configure block; set slave mode
  //i2c_init(I2C_PORT, 1000000);                 // freq value irrelevant in slave
  i2c_set_slave_mode (I2C_PORT, true, I2C_SLAVE_ADDR);

  // Enable RX-DMA on the I2C block
  i2c_hw_t *hw = i2c_get_hw (I2C_PORT);
  hw_set_bits (&hw->con, I2C_IC_CON_RX_FIFO_FULL_HLD_CTRL_BITS);
  hw->dma_rdlr = 0;		// DREQ when >=1 byte in RX FIFO
  hw_set_bits (&hw->dma_cr, I2C_IC_DMA_CR_RDMAE_BITS);
  hw_clear_bits (&hw->dma_cr, I2C_IC_DMA_CR_TDMAE_BITS);	// TX-DMA off

  // Flush any stale RX bytes
  while (i2c_get_read_available (I2C_PORT))
    (void) hw->data_cmd;

  dma_init_and_start ();
}


static inline void hexdump(const uint8_t *p, uint32_t n){
    for (uint32_t i = 0; i < n; i++){
        printf("%02X%s", (unsigned)p[i], ((i+1)%16) ? " " : "\n");
    }
    if (n % 16) printf("\n");
}
// ---------------- LE COC SEND PUMP ----------------
static bool
lecoc_try_send_once (void)
{
  if (!lecoc_cid || !lecoc_can_send)
    return false;

  uint32_t avail = ring_avail ();
  if (!avail)
    return false;

  uint32_t to_send = avail;
  if (to_send > lecoc_remote_mtu)
    to_send = lecoc_remote_mtu;
  if (to_send > LECOC_TX_MTU)
    to_send = LECOC_TX_MTU;

  // copy from ring (handle wrap)
  uint32_t rd = rd_offset % RING_BYTES;
  uint32_t first = to_send;
  uint32_t tail = 0;
  if (rd + to_send > RING_BYTES)
    {
      first = RING_BYTES - rd;
      tail = to_send - first;
    }
  memcpy (tx_buf, &ring_buf[rd], first);
  if (tail)
    memcpy (tx_buf + first, &ring_buf[0], tail);
#if TESTING
  hexdump(tx_buf, to_send);
#endif
  int err = l2cap_send (lecoc_cid, tx_buf, to_send);
  if (err)
    {
      printf("// No credits / no ACL buffers → re-arm\n");
      l2cap_request_can_send_now_event (lecoc_cid);
      return false;
    }
  ring_consume (to_send);
  maybe_resume_dma ();

  // we used our slot; re-arm to keep pipeline full
  lecoc_can_send = false;
  l2cap_request_can_send_now_event (lecoc_cid);
  return true;
}

// ---------------- BTSTACK HANDLERS (LE CoC / CBM) ----------------
static void
ble_packet_handler (uint8_t packet_type, uint16_t channel, uint8_t *packet,
		    uint16_t size)
{
  (void) channel;
  (void) size;
  if (packet_type != HCI_EVENT_PACKET)
    return;

  uint8_t event = hci_event_packet_get_type (packet);
  switch (event)
    {
    case BTSTACK_EVENT_STATE:
      if (btstack_event_state_get_state (packet) == HCI_STATE_WORKING)
	{
	  // Suggest maximums for new connections
	  hci_send_cmd (&hci_le_write_suggested_default_data_length,
			DLE_TX_OCTETS, DLE_TX_TIME_US);
	  // Start advertising as a sink
	  const uint8_t adv[] =
	    { 0x02, 0x01, 0x06, 0x0E, 0x09, 'I', '2', 'C', ' ', 'B', 'L', 'E',
' ', 'B', 'r', 'i', 'd', 'g', 'e' };
	  gap_advertisements_set_params (0x30, 0x30, 0, 0, (bd_addr_t)
					 {
					 0}, 0x07, 0);
	  gap_advertisements_set_data (sizeof (adv), (uint8_t *) adv);
	  gap_advertisements_enable (1);
	  printf ("BLE ready. Advertising LE CoC sink on PSM 0x%04x\n",
		  CBM_PSM);
	}
      break;

    case HCI_EVENT_LE_META:
      {
	uint8_t sub = hci_event_le_meta_get_subevent_code (packet);
	if (sub == HCI_SUBEVENT_LE_CONNECTION_COMPLETE)
	  {
	    // You can request 2M PHY for throughput
	    hci_con_handle_t handle =
	      hci_subevent_le_connection_complete_get_connection_handle
	      (packet);
	    gap_le_set_phy (handle, 0, 0x02, 0x02, 0);	// request 2M both ways
	    //
	    //gap_set_data_length(handle, DLE_TX_OCTETS, DLE_TX_TIME_US);
	  }
	if (sub == HCI_SUBEVENT_LE_PHY_UPDATE_COMPLETE)
	  {
	    uint8_t status =
	      hci_subevent_le_phy_update_complete_get_status (packet);
	    uint16_t handle =
	      hci_subevent_le_phy_update_complete_get_connection_handle
	      (packet);
	    uint8_t tx_phy =
	      hci_subevent_le_phy_update_complete_get_tx_phy (packet);
	    uint8_t rx_phy = packet[7];
	    printf ("LE PHY Update: handle=0x%04x status=%u tx=%u rx=%u\n",
		    handle, status, tx_phy, rx_phy);
	  }



	if (sub == HCI_SUBEVENT_LE_DATA_LENGTH_CHANGE)
	  {
	    uint16_t handle =
	      hci_subevent_le_data_length_change_get_connection_handle
	      (packet);
	    uint16_t max_tx_oct =
	      hci_subevent_le_data_length_change_get_max_tx_octets (packet);
	    uint16_t max_tx_time =
	      hci_subevent_le_data_length_change_get_max_tx_time (packet);
	    uint16_t max_rx_oct =
	      hci_subevent_le_data_length_change_get_max_rx_octets (packet);
	    uint16_t max_rx_time =
	      hci_subevent_le_data_length_change_get_max_rx_time (packet);
	    printf
	      ("DLE Change: h=0x%04x TX(oct=%u,time=%u) RX(oct=%u,time=%u)\n",
	       handle, max_tx_oct, max_tx_time, max_rx_oct, max_rx_time);
	  }

      }
      break;
    case HCI_EVENT_COMMAND_COMPLETE:
      /*printf("Command complete opcode=0x%04x, status=%u\n",
         hci_event_command_complete_get_command_opcode(packet),
         hci_event_command_complete_get_return_parameters(packet)[0]);
       */
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
	      hci_send_cmd (&hci_read_local_version_information);
	    }
	  else
	    {
	      printf
		("LE Read Local Supported Features command failed with status: 0x%02x\n",
		 status);
	    }
	}

      if (hci_event_command_complete_get_command_opcode (packet) ==
	  HCI_OPCODE_HCI_READ_LOCAL_VERSION_INFORMATION)
	{
	  const uint8_t *params =
	    hci_event_command_complete_get_return_parameters (packet);

	  uint8_t status = params[0];
	  uint8_t hci_version = params[1];
	  uint16_t hci_revision = little_endian_read_16 (params, 2);
	  uint8_t lmp_version = params[4];
	  uint16_t manufacturer = little_endian_read_16 (params, 5);
	  uint16_t lmp_subv = little_endian_read_16 (params, 7);

	  printf ("Local Version Info:\n");
	  printf ("  Status: %u\n", status);
	  printf ("  HCI Version: %u\n", hci_version);
	  printf ("  HCI Revision: 0x%04x\n", hci_revision);
	  printf ("  LMP Version: %u\n", lmp_version);
	  printf ("  Manufacturer: 0x%04x\n", manufacturer);
	  printf ("  LMP Subversion: 0x%04x\n", lmp_subv);

	  const uint8_t adv[] =
	    { 0x02, 0x01, 0x06, 0x0E, 0x09, 'I', '2', 'C', ' ', 'B',
	    'L', 'E', ' ', 'B', 'r', 'i', 'd', 'g', 'e'
	  };
	  gap_advertisements_set_params (0x30, 0x30, 0, 0, (bd_addr_t)
					 {
					 0}, 0x07, 0);
	  gap_advertisements_set_data (sizeof (adv), (uint8_t *) adv);
	  gap_advertisements_enable (1);
	}			/*
				   if (hci_event_command_complete_get_command_opcode (packet) ==
				   HCI_OPCODE_HCI_LE_SET_EVENT_MASK)
				   printf ("LE Event Mask set complete.\n"); */
      if (hci_event_command_complete_get_command_opcode (packet) ==
	  HCI_OPCODE_HCI_LE_CONNECTION_UPDATE)
	printf ("hci_opcode_hci_le_connection_update \n");
      if (hci_event_command_complete_get_command_opcode (packet) ==
	  HCI_OPCODE_HCI_LE_SET_DEFAULT_PHY)
	printf ("hci_opcode_hci_le_set_default_phy\n");
      break;



    case L2CAP_EVENT_CBM_INCOMING_CONNECTION:
      {
	uint16_t psm = l2cap_event_cbm_incoming_connection_get_psm (packet);
	uint16_t cid =
	  l2cap_event_cbm_incoming_connection_get_local_cid (packet);
	printf ("Incoming LE CoC: PSM=0x%04x, cid=%u\n", psm, cid);
	if (psm == CBM_PSM)
	  {
	    // Provide a small RX buffer for inbound SDUs (we mostly send; 2*MTU is fine)
	    static uint8_t rx_buf[LECOC_TX_MTU * 2];
	    l2cap_cbm_accept_connection (cid, rx_buf, sizeof (rx_buf),
					 INITIAL_CREDITS);
	  }
      }
      break;

    case L2CAP_EVENT_CBM_CHANNEL_OPENED:
      {
	uint8_t status = l2cap_event_cbm_channel_opened_get_status (packet);
	if (status)
	  {
	    printf ("LE CoC open failed: 0x%02x\n", status);
	    lecoc_cid = 0;
	    break;
	  }
	lecoc_cid = l2cap_event_cbm_channel_opened_get_local_cid (packet);
	lecoc_remote_mtu =
	  l2cap_event_cbm_channel_opened_get_remote_mtu (packet);
	if (lecoc_remote_mtu > LECOC_TX_MTU)
	  lecoc_remote_mtu = LECOC_TX_MTU;
	printf ("LE CoC open ok: cid=%u, remote_mtu=%u\n", lecoc_cid,
		lecoc_remote_mtu);
	l2cap_request_can_send_now_event (lecoc_cid);
      }
      break;

    case L2CAP_EVENT_CAN_SEND_NOW:
      lecoc_can_send = true;
      (void) lecoc_try_send_once ();	// one SDU per slot; we re-arm inside
      break;

    case HCI_EVENT_DISCONNECTION_COMPLETE:
      printf ("BLE link down. Resetting channel.\n");
      lecoc_cid = 0;
      lecoc_remote_mtu = LECOC_TX_MTU;
      lecoc_can_send = false;
      // optional: clear ring
      // rd_offset = wr_off = 0;
      break;

    default:
      break;
    }
}

int
main (void)
{
  stdio_init_all ();
  sleep_ms (1000);
  printf ("Pico W I2C Slave + DMA → BLE LE CoC bridge @ 0x%02X\n",
	  I2C_SLAVE_ADDR);

  if (cyw43_arch_init ())
    {
      printf ("CYW43 init failed\n");
      return -1;
    }

  // BTstack bring-up (LE-CoC)
  l2cap_init ();
  l2cap_cb.callback = &ble_packet_handler;
  hci_cb.callback = &ble_packet_handler;
  l2cap_add_event_handler (&l2cap_cb);
  hci_add_event_handler (&hci_cb);

  // Register LE CoC service (sink) on CBM PSM
  l2cap_cbm_register_service (&ble_packet_handler, CBM_PSM, LEVEL_0);

  hci_power_control (HCI_POWER_ON);

  // Start I2C slave + DMA capture
  i2c_slave_init_dma ();

  // Light telemetry
  absolute_time_t last = get_absolute_time ();
  while (true)
    {
      async_context_poll (cyw43_arch_async_context ());

      // Nudge send pipeline
      if (lecoc_cid && ring_avail () > 0)
	{
	  l2cap_request_can_send_now_event (lecoc_cid);
	}

      if (absolute_time_diff_us (last, get_absolute_time ()) > 1000000)
	{
	  last = get_absolute_time ();
	  // Uncomment if you want to see stats:
	  // printf("DMA_IRQ=%lu  avail=%lu  rd=%lu\n",
	  //    (unsigned long)dma_irq_count,
	  //    (unsigned long)ring_avail(),
	  //    (unsigned long)rd_offset);
	}
      sleep_ms (1);
    }
}
