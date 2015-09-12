#include <poll.h>   // poll
#include <signal.h>
#include <stdio.h>  // snprintf
#include <unistd.h> // read
#include <chrono>
#include <climits>
#include <cstring>  // strncat
#include <thread>
#include <iostream>
#include "lightweightserial.h"

// CAN #IDs.
static const uint32_t kSync    = 0x080;
static const uint32_t kSensor1 = 0x4AA;
static const uint32_t kSensor2 = 0x4AC;
static const uint32_t kSensor3 = 0x4BF;
static const uint32_t kACI1    = 0x210;
static const uint32_t kACI2    = 0x211;
static const uint32_t kACI3    = 0x212;

static LightweightSerial *port;

//////////////////////////////////////////////////
void tearDown()
{
  // Close device.
  port->write_cstr("C\r");
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  delete port;

  std::cout << std::endl << "Bye" << std::endl;
  exit(0);
}

//////////////////////////////////////////////////
void signalHandler(int signum)
{
  if (signum == SIGINT || signum == SIGTERM)
    tearDown();
}

//////////////////////////////////////////////////
void waitForSync()
{
  bool syncReceived = false;
  while (port->is_ok() && !syncReceived)
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


    std::string msg;
    char delimiter = '\r';
    bool delimiterFound = false;

    while (!delimiterFound)
    {
      uint8_t buffer[1];
      auto res = ::read(port->fd, &buffer[0], 1);
      if (res == 0)
      {
        std::cerr << "Reading error" << std::endl;
        return;
      }

      char c = static_cast<char>(buffer[0]);
      if (c == delimiter)
        delimiterFound = true;
      else
        msg += c;
    }

    // Check if this was a SYNC or something else.
    syncReceived = (msg.find("t" + kSync) != std::string::npos);
  }
}

//////////////////////////////////////////////////
void showMenu(uint16_t &_option, uint16_t &_value)
{
  do
  {
    std::cout << "DEKA teleoperator" << std::endl;
    std::cout << "1. Hand open" << std::endl;
    std::cout << "2. Hand close" << std::endl;
    std::cout << "3. Toggle grip positive" << std::endl;
    std::cout << "4. Toggle grip negative" << std::endl;
    std::cout << "5. Twist rotator positive" << std::endl;
    std::cout << "6. Twist rotator negative" << std::endl;
    std::cout << "7. Wrist flexor/deviator positive" << std::endl;
    std::cout << "8. Wrist flexor/deviator negative" << std::endl;
    std::cout << "9. Model select command" << std::endl << std::endl;

    std::cout << "0. Exit" << std::endl << std::endl;
    std::cout << "Select an option [0-9]: ";

    std::cin >> _option;
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');
    if (_option < 0 || _option > 9)
      std::cerr << std::endl << "Wrong option, should be [0-9]" << std::endl;
  }
  while (_option < 0 || _option > 9);

  if (_option == 0)
    tearDown();

  do
  {
    std::cout << std::endl << std::endl << "Select the value (0-1024): ";
    std::cin >> _value;
    std::cin.clear();
    std::cin.ignore(INT_MAX, '\n');
    if (_value < 0 || _value > 1024)
      std::cerr << std::endl << "Wrong value, should be [0-1024]" << std::endl;
  }
  while (_value < 0 || _value > 1024);
}

//////////////////////////////////////////////////
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
  port = new LightweightSerial(serialDevice, 3000000);
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
  while (port->is_ok())
  {
    // Arm inputs.
    uint16_t handOpen        = 0;
    uint16_t handClose       = 0;
    uint16_t toggleGripPos   = 0;
    uint16_t toggleGripNeg   = 0;
    uint16_t wristRotatorPos = 0;
    uint16_t wristRotatorNeg = 0;
    uint16_t wristFlexorPos  = 0;
    uint16_t wristFlexorNeg  = 0;
    uint16_t modeSelectCmd   = 0;

    // Read the command/value to send.
    uint16_t option;
    uint16_t input;
    showMenu(option, input);

    // Update the CAN #id and input.
    switch (option)
    {
      case 1:
        handOpen = input;
        break;
      case 2:
        handClose = input;
        break;
      case 3:
        toggleGripPos = input;
        break;
      case 4:
        toggleGripNeg = input;
        break;
      case 5:
        wristRotatorPos = input;
        break;
      case 6:
        wristRotatorNeg = input;
        break;
      case 7:
        wristFlexorPos = input;
        break;
      case 8:
        wristFlexorNeg = input;
        break;
      case 9:
        modeSelectCmd = input;
        break;
      default:
        std::cerr << "You shouldn't be here, move along please" << std::endl;
        break;
    };

    // Fill ACI1 input.
    uint8_t aci1[8] = {0};
    aci1[0] = handOpen      >> 8;
    aci1[1] = handOpen      & 0xff;
    aci1[2] = handClose     >> 8;
    aci1[3] = handClose     & 0xff;
    aci1[4] = toggleGripPos >> 8;
    aci1[5] = toggleGripPos & 0xff;
    aci1[6] = toggleGripNeg >> 8;
    aci1[7] = toggleGripNeg & 0xff;

    // Fill ACI2 input.
    uint8_t aci2[8] = {0};
    aci2[0] = wristRotatorPos >> 8;
    aci2[1] = wristRotatorPos & 0xff;
    aci2[2] = wristRotatorNeg >> 8;
    aci2[3] = wristRotatorNeg & 0xff;
    aci2[4] = wristFlexorPos  >> 8;
    aci2[5] = wristFlexorPos  & 0xff;
    aci2[6] = wristFlexorNeg  >> 8;
    aci2[7] = wristFlexorNeg  & 0xff;

    // Fill ACI3 input.
    uint8_t aci3[8] = {0};
    aci3[0] = modeSelectCmd >> 8;
    aci3[1] = modeSelectCmd & 0xff;

    // Fill aci1Msg with the #ID and data length (8).
    char aci1Msg[50] = {0};
    snprintf(aci1Msg, sizeof(aci1Msg), "t%x8", kACI1);

    // Fill aci1Msg with the input.
    for (auto i = 0; i < 8; ++i)
    {
      char hexByte[8];
      snprintf(hexByte, sizeof(hexByte), "%02x", aci1[i]);
      strncat(aci1Msg, hexByte, 2);
    }
    strncat(aci1Msg, "\r", 2);

    // Fill aci2Msg with the #ID and data length (8).
    char aci2Msg[50] = {0};
    snprintf(aci2Msg, sizeof(aci2Msg), "t%x8", kACI2);

    // Fill aci2Msg with the input.
    for (auto i = 0; i < 8; ++i)
    {
      char hexByte[10];
      snprintf(hexByte, sizeof(hexByte), "%02x", aci2[i]);
      strncat(aci2Msg, hexByte, 2);
    }
    strncat(aci2Msg, "\r", 2);

    // Fill aci3Msg with the #ID and data length (8).
    char aci3Msg[50] = {0};
    snprintf(aci3Msg, sizeof(aci3Msg), "t%x8", kACI3);

    // Fill aci3Msg with the input.
    for (auto i = 0; i < 8; ++i)
    {
      char hexByte[10];
      snprintf(hexByte, sizeof(hexByte), "%02x", aci3[i]);
      strncat(aci3Msg, hexByte, 2);
    }
    strncat(aci3Msg, "\r", 2);

    // Wait for the SYNC message.
    //waitForSync();
    std::cout << "Sync received, sending inputs:" << std::endl;
    std::cout << "ACI1: " << aci1Msg << std::endl;
    std::cout << "ACI2: " << aci2Msg << std::endl;
    std::cout << "ACI3: " << aci3Msg << std::endl;

    // Send CAN messages.
    port->write_cstr(aci1Msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    port->write_cstr("E\r");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    port->write_cstr(aci2Msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    port->write_cstr("E\r");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    port->write_cstr(aci3Msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    port->write_cstr("E\r");
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    std::cout << "Press [ENTER] for send another command" << std::endl;
    getchar();

    // Send some data.
    // Example:
    // t21080123456789ABCDEF
    // Transmit a 11-bit ID frame with
    // ID = 0x210
    // Length = 8 bytes
    // Data = 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF
    // Ref: http://www.easysync-ltd.com/userfiles/editor/file/support/datasheets/DS_USB2-F-7x01(2).pdf
    //port->write_cstr("t21080123456789ABCDEF\r");
    //std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  tearDown();

  return 0;
}