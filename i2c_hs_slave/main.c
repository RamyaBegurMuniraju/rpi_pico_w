#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/clocks.h"
#include "i2c_hs_slave.h"

#define SDA_PIN   4
#define SCL_PIN   5
#define TARGET_ADDR 0x17	// expected 7-bit address from your master
#define TOTAL_BYTES (2 * 1024 * 1024)	// 2 MB
				       //
static uint8_t first_bytes[16];
static int first_bytes_cnt = 0;
static bool first_bytes_dumped = false;

static void
dump_first_bytes_once (void)
{
  if (first_bytes_dumped || first_bytes_cnt < 16)
    return;
  first_bytes_dumped = true;

  printf ("\n[SLAVE] First 16 bytes seen:\n");
  for (int i = 0; i < 16; ++i)
    {
      printf ("%02X%s", first_bytes[i], (i == 15) ? "\n" : " ");
    }

  // Interpret the first byte as address+R/W
  uint8_t addr_rw = first_bytes[0];
  uint8_t addr7 = addr_rw >> 1;
  uint8_t rw = addr_rw & 1;
  printf ("[SLAVE] Byte0 (addr+R/W) = 0x%02X -> addr=0x%02X (%u), %s\n",
	  addr_rw, addr7, addr7, rw ? "READ" : "WRITE");
  printf ("[SLAVE] Expect 0x2E for addr 0x17 + W=0. %s\n",
	  (addr_rw == 0x2E ? "OK" : "(!) Mismatch"));
}


static inline int
wait_byte_blocking (void)
{
  int b;
  while ((b = i2c_hs_slave_get_byte ()) < 0)
    {
      tight_loop_contents ();
    }
  return b;
}

int
main ()
{
  stdio_init_all ();
  sleep_ms (1000);
  printf ("RP2040 I2C High-Speed SLAVE (PIO+DMA) receiving 2 MB...\n");

  // Optional: overclock for extra timing margin at 3.4 MHz
  // set_sys_clock_khz(200000, true);

  i2c_hs_slave_t hs;
  i2c_hs_slave_init (&hs, pio0, 0, SDA_PIN, SCL_PIN);

  uint64_t total_data = 0;
  absolute_time_t t0 = get_absolute_time ();

  enum
  { WAIT_ADDR, DATA } state = WAIT_ADDR;

      while (total_data < TOTAL_BYTES)
	{
// If a STOP happened, go back to expecting an address
      if (i2c_hs_slave_stop_seen ())
	{
	  i2c_hs_slave_clear_stop ();
	  state = WAIT_ADDR;
	}


	  int b = i2c_hs_slave_get_byte ();
	  if (b < 0)
	    {
	      tight_loop_contents ();
	      continue;
	    }

	  if (state == WAIT_ADDR)
	    {
	      uint8_t addr_rw = (uint8_t) b;
	      if (!first_bytes_dumped && first_bytes_cnt < 16)
		first_bytes[first_bytes_cnt++] = addr_rw;
	      dump_first_bytes_once ();

	      uint8_t addr7 = addr_rw >> 1;
	      bool rw = addr_rw & 1;
	      if (addr7 == TARGET_ADDR && !rw)
		{
		  state = DATA;
		}
	      else
		{
		  state = WAIT_ADDR;
		}
	      continue;
	    }

	  // DATA byte
	  // --- add these two lines to record & maybe dump ---
	 /* if (!first_bytes_dumped && first_bytes_cnt < 16)
	    first_bytes[first_bytes_cnt++] = (uint8_t) b;
	  dump_first_bytes_once ();
*/
	  total_data++;

	  // periodic stats
	  absolute_time_t now = get_absolute_time ();
	  int64_t us = absolute_time_diff_us (t0, now);
	  if (us >= 1000000)
	    {
	      double kbps = (total_data * 8.0) / (us / 1000.0);
	      printf
		("Received: %llu / %u bytes  (~%.1f kbps)  ring_free=%u\n",
		 (unsigned long long) total_data, (unsigned) TOTAL_BYTES,
		 kbps, i2c_hs_slave_ring_free ());
	      t0 = now;
	    }
	}

  // Final stats
  absolute_time_t t1 = get_absolute_time ();
  double secs = absolute_time_diff_us (t0, t1) / 1e6;
  printf ("DONE: Received %u bytes.\n", (unsigned) TOTAL_BYTES);
  (void) secs;

  while (true)
    {
      tight_loop_contents ();
    }
}
