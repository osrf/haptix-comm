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

  while (port->is_ok() && !gDone)
  {
    struct pollfd ufds[1];
    memset(ufds, 0, sizeof(ufds));
    ufds[0].fd = port->fd;
    ufds[0].events = POLLIN;

    // Poll every 500 milliseconds
    int pollReturnCode = poll(ufds, 1, 500);
    if (pollReturnCode == -1)
    {
      continue;
    }
    else if (pollReturnCode == 0)
    {
      // Timeout occurred
      continue;
    }
    else if (!(ufds[0].revents && POLLIN))
    {
      std::cerr << "Received out of band message in poll" << std::endl;
      continue;
    }

    uint8_t buffer[1];
    auto res = ::read(port->fd, &buffer[0], 1);
    if (res == 0)
    {
      std::cerr << "Reading error" << std::endl;
      continue;
    }

    for (int i = 0; i < res; ++i)
      std::cout << static_cast<char>(buffer[i]);

    if (buffer[0] == '\r')
      std::cout << std::endl;
  }

  // Close device.
  port->write_cstr("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::cout << "Bye" << std::endl;
  return 0;
}