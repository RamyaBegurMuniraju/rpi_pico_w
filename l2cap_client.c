#define _GNU_SOURCE
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/socket.h>
#include <sys/types.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/l2cap.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>
#include <bluetooth/l2cap.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <sys/fcntl.h>

#define TOTAL_SIZE (2 * 1024 * 1024)    // 2MB
//#define TOTAL_SIZE 2 * 2048   
//#define TOTAL_SIZE 2048       // 
#define CHUNK_SIZE 65535    // Sending in 512 byte chunks

static void
die (const char *msg)
{
  perror (msg);
  exit (1);
}

static inline void
hexdump (const uint8_t *p, uint32_t n)
{
  for (uint32_t i = 0; i < n; i++)
    {
      printf ("%02X%s", (unsigned) p[i], ((i + 1) % 16) ? " " : "\n");
    }
  if (n % 16)
    printf ("\n");
}

int
get_rssi (int dev_id, uint16_t handle)
{
  int sock;
  struct hci_request rq;
  read_rssi_rp rp;
  int8_t rssi;

  sock = hci_open_dev (dev_id);
  if (sock < 0)
    {
      perror ("HCI device open failed");
      return -1;
    }

  memset (&rq, 0, sizeof (rq));
  rq.ogf = OGF_STATUS_PARAM;
  rq.ocf = OCF_READ_RSSI;
  rq.cparam = &handle;
  rq.clen = 2;
  rq.rparam = &rp;
  rq.rlen = sizeof (rp);

  if (hci_send_req (sock, &rq, 1000) < 0)
    {
      perror ("HCI read RSSI failed");
      close (sock);
      return -1;
    }

  rssi = rp.rssi;
  close (sock);
  return (int) rssi;
}

int
main (int argc, char **argv)
{

  uint8_t buf[CHUNK_SIZE];

  long total_bytes = 0;
  if (argc < 2)
    {
      fprintf (stderr, "Usage: %s <BDADDR> <PSM-hex e.g. 0x1001>\n", argv[0]);
      return 2;
    }
  const char *bdaddr_str = argv[1];
  uint16_t psm = (uint16_t) strtoul (argv[2], NULL, 0);
  struct timeval start_time, end_time;

  int s = socket (AF_BLUETOOTH, SOCK_SEQPACKET, BTPROTO_L2CAP);
  if (s < 0)
    die ("socket");

  // Optional: set security level (MEDIUM = auth; HIGH = auth+enc)
  struct bt_security sec = {.level = BT_SECURITY_MEDIUM };
  if (setsockopt (s, SOL_BLUETOOTH, BT_SECURITY, &sec, sizeof (sec)) < 0)
    {
      perror ("setsockopt(BT_SECURITY)");    /* non-fatal */
    }

  // Optional: tune L2CAP MTUs (Basic mode)
  struct l2cap_options opts;
  socklen_t optlen = sizeof (opts);
  memset (&opts, 0, sizeof (opts));
  // set desired outgoing MTU; incoming will be reported by kernel after connect
  //opts.omtu = 4096;        // classic default; kernel will clamp to remote MTU
  //opts.imtu = 4096;        // allow large reads from kernel buffer
  opts.flush_to = 0xFFFF;    // no flush timeout
  opts.mode = L2CAP_MODE_ERTM;
  if (setsockopt (s, SOL_L2CAP, L2CAP_OPTIONS, &opts, sizeof (opts)) < 0)
    {
      perror ("setsockopt(L2CAP_OPTIONS)");    /* non-fatal */
    }

  // Destination address
  struct sockaddr_l2 addr;
  memset (&addr, 0, sizeof (addr));
  addr.l2_family = AF_BLUETOOTH;
  str2ba (bdaddr_str, &addr.l2_bdaddr);
  addr.l2_psm = htobs (psm);    // IMPORTANT: host-to-Bluetooth byte order

  int sz = 512 * 1024;
  setsockopt (s, SOL_SOCKET, SO_SNDBUF, &sz, sizeof (sz));
  setsockopt (s, SOL_SOCKET, SO_RCVBUF, &sz, sizeof (sz));
  fprintf (stderr, "Connecting to %s PSM 0x%04x ...\n", bdaddr_str, psm);
  if (connect (s, (struct sockaddr *) &addr, sizeof (addr)) < 0)
    die ("connect");
  fprintf (stderr, "Connected.\n");

  // Query negotiated MTUs (optional)
  memset (&opts, 0, sizeof (opts));
  optlen = sizeof (opts);
  if (getsockopt (s, SOL_L2CAP, L2CAP_OPTIONS, &opts, &optlen) == 0)
    {
      fprintf (stderr, "MTU in=%u out=%u mode=%u\n", opts.imtu, opts.omtu,
           opts.mode);
    }


  // after you've connected and done getsockopt(...)
struct l2cap_conninfo conn;
socklen_t len = sizeof(conn);

if (getsockopt(s, SOL_L2CAP, L2CAP_CONNINFO, &conn, &len) < 0) {
    perror("getsockopt(L2CAP_CONNINFO)");
    return 1;
}

printf("HCI handle: 0x%04x\n", conn.hci_handle);

// get the HCI device that can reach this BDADDR
int dev_id = hci_get_route(&addr.l2_bdaddr);

if (dev_id < 0) {
    perror("hci_get_route");
    return 1;
}



  // Timing start
  //gettimeofday (&start_time, NULL);

  //int fd = open("2mb_1MHz.bin", O_CREAT|O_RDWR, 0666);
  int first_byte = 0;
  // Receive loop
  while (1)
    {
      int bytes_read = read (s, buf, CHUNK_SIZE);
      //if (write(fd, buf, bytes_read) == -1)
      //      printf("write faild\n");
      if (!first_byte)
    {
      gettimeofday (&start_time, NULL);
      first_byte = 1;
    }
      if (bytes_read < 0)
    {
      perror ("read");
      break;
    }
      else if (bytes_read == 0)
    {
      printf ("Client disconnected.\n");
      break;
    }
      //hexdump(buf, bytes_read);

      total_bytes += bytes_read;

      //int dev_id = hci_get_route (&conn.hci_handle);
      //int rssi = get_rssi (dev_id, conn.hci_handle);
//printf("Current RSSI = %d\n", get_rssi(dev_id, conn.hci_handle));

#if 0
      //printf("Received %d bytes\n", bytes_read);
      if (total_bytes % (100 * 1024) < bytes_read)
    {
      printf ("Progress: %ld bytes received\n", total_bytes);
      gettimeofday (&end_time, NULL);
// Throughput calculation
      double start_sec =
        start_time.tv_sec + start_time.tv_usec / 1000000.0;
      double end_sec = end_time.tv_sec + end_time.tv_usec / 1000000.0;
      double duration = end_sec - start_sec;

      double throughput_bps = (total_bytes * 8.0) / duration;
      double throughput_kbps = throughput_bps / 1000.0;
      double throughput_mbps = throughput_kbps / 1000.0;

      printf ("\n--- Throughput Report ---\n");
      printf ("Total Bytes Received: %ld bytes\n", total_bytes);
      printf ("Total Time: %.4f seconds\n", duration);
      printf ("Throughput: %.2f kbps (%.2f Mbps)\n", throughput_kbps,
          throughput_mbps);
    }
#endif
      if (total_bytes >= TOTAL_SIZE)
    {
      printf ("Received total target: %ld bytes\n", total_bytes);
      break;
    }
    }

  // Timing end
  gettimeofday (&end_time, NULL);

  // Throughput calculation
  double start_sec = start_time.tv_sec + start_time.tv_usec / 1000000.0;
  double end_sec = end_time.tv_sec + end_time.tv_usec / 1000000.0;
  double duration = end_sec - start_sec;

  double throughput_bps = (total_bytes * 8.0) / duration;
  double throughput_kbps = throughput_bps / 1000.0;
  double throughput_mbps = throughput_kbps / 1000.0;

  printf ("\n--- Throughput Report ---\n");
  printf ("Total Bytes Received: %ld bytes\n", total_bytes);
  printf ("Total Time: %.4f seconds\n", duration);
  printf ("Throughput: %.2f kbps (%.2f Mbps)\n", throughput_kbps,
      throughput_mbps);



  close (s);
  return 0;
}
