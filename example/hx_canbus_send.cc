#include <signal.h>
#include <chrono>
#include <thread>
#include <iostream>
#include "lightweightserial.h"

static bool gDone = false;
void signalHandler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    gDone = true;
}

void usage()
{
  std::cerr << "usage: hx_canbus_send <CAN_DEVICE>" << std::endl;
  exit(1);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  signal(SIGINT, signalHandler);
  signal(SIGTERM, signalHandler);

  if (argc != 2)
  {
    usage();
    exit(1);
  }

  const char *serialDevice = argv[1];
  LightweightSerial *port = new LightweightSerial(serialDevice, 3000000);
  if (!port)
    std::cerr << "Couldn't open the specified serial port" << std::endl;

  // Put it to 1 Mbps.
  port->write_cstr("S8\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Open the channel.
  port->write_cstr("O\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  port->write_cstr("E\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Flush tx buffers.
  while (port->is_ok() && !gDone)
  {
    // Send some data.
    // Example:
    // t34580123456789ABCDEF
    // Transmit a 11-bit ID frame with
    // ID = 0x210
    // Length = 8 bytes
    // Data = 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    // Ref: http://www.easysync-ltd.com/userfiles/editor/file/support/datasheets/DS_USB2-F-7x01(2).pdf
    port->write_cstr("t21080123456789ABCDEF\r");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  // Close device.
  port->write_cstr("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Bye" << std::endl;
  return 0;
}