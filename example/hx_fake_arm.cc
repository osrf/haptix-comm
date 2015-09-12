#include <poll.h>
#include <signal.h>
#include <unistd.h>
#include <chrono>
#include <cstring> //memset
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
  std::cerr << "usage: hx_canbus_receive <CAN_DEVICE>" << std::endl;
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

  while (port->is_ok() && !gDone)
  {
    // Send a Sync message every 10ms.
    // ID = 0x080
    // Length = 0 bytes.
    port->write_cstr("t4560");
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  // Close device.
  port->write_cstr("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  delete port;

  std::cout << "Bye" << std::endl;
  return 0;
}