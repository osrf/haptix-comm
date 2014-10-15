#include "sliders_amidi.h"
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>
#include <cstdio>
#include <errno.h>
#include <boost/bind.hpp>

Sliders::Sliders()
: fd(-1)
{
  for (unsigned i = 0; i < NUM_CHANNELS; i++)
  {
    knobs[i] = sliders[i] = 0;
    channel_buttons[i][0] = channel_buttons[i][1] = false;
  }
  for (unsigned i = 0; i < NUM_BUTTONS; i++)
    buttons[i] = false;
  analog_control_map[0x02] = &sliders[0];
  analog_control_map[0x03] = &sliders[1];
  analog_control_map[0x04] = &sliders[2];
  analog_control_map[0x05] = &sliders[3];
  analog_control_map[0x06] = &sliders[4];
  analog_control_map[0x08] = &sliders[5];
  analog_control_map[0x09] = &sliders[6];
  analog_control_map[0x0c] = &sliders[7];
  analog_control_map[0x0d] = &sliders[8];
  analog_control_map[0x0e] = &knobs[0];
  analog_control_map[0x0f] = &knobs[1];
  analog_control_map[0x10] = &knobs[2];
  analog_control_map[0x11] = &knobs[3];
  analog_control_map[0x12] = &knobs[4];
  analog_control_map[0x13] = &knobs[5];
  analog_control_map[0x14] = &knobs[6];
  analog_control_map[0x15] = &knobs[7];
  analog_control_map[0x16] = &knobs[8];
  digital_control_map[0x17] = &channel_buttons[0][0];
  digital_control_map[0x21] = &channel_buttons[0][1];
  digital_control_map[0x18] = &channel_buttons[1][0];
  digital_control_map[0x22] = &channel_buttons[1][1];
  digital_control_map[0x19] = &channel_buttons[2][0];
  digital_control_map[0x23] = &channel_buttons[2][1];
  digital_control_map[0x1a] = &channel_buttons[3][0];
  digital_control_map[0x24] = &channel_buttons[3][1];
  digital_control_map[0x1b] = &channel_buttons[4][0];
  digital_control_map[0x25] = &channel_buttons[4][1];
  digital_control_map[0x1c] = &channel_buttons[5][0];
  digital_control_map[0x26] = &channel_buttons[5][1];
  digital_control_map[0x1d] = &channel_buttons[6][0];
  digital_control_map[0x27] = &channel_buttons[6][1];
  digital_control_map[0x1e] = &channel_buttons[7][0];
  digital_control_map[0x28] = &channel_buttons[7][1];
  digital_control_map[0x1f] = &channel_buttons[8][0];
  digital_control_map[0x29] = &channel_buttons[8][1];
  digital_control_map[0x2f] = &buttons[0];
  digital_control_map[0x2d] = &buttons[1];
  digital_control_map[0x30] = &buttons[2];
  digital_control_map[0x31] = &buttons[3];
  digital_control_map[0x2e] = &buttons[4];
  digital_control_map[0x2c] = &buttons[5];
}

Sliders::~Sliders()
{
}

bool Sliders::open(const char *midi_dev_name)
{
  fd = ::open(midi_dev_name, O_RDONLY | O_NONBLOCK);
  if (fd < 0)
    return false;
  printf("opened %s as fd %d\n", midi_dev_name, fd);
  return true;
}

void Sliders::blockingRead()
{
  unsigned char ch[3];
  snd_rawmidi_read(this->midi_in,&ch,3);
  if ((ch[0] & 0xf0) == 0xb0)
  {
    // it's a status message hooray
    uint8_t control = ch[1];
    uint8_t value = ch[2];
    // printf("received control[%d] value[%f]\n", control, value/127.0f);
    if (this->analog_control_map.find(control) != this->analog_control_map.end())
      *analog_control_map.at(control) = value / 127.0f;
  }
}

bool Sliders::midi_open(char *port){
 // Check available ports.
  int err = snd_rawmidi_open(&this->midi_in, NULL, port, 0);
  if (err > 0)
    printf("err: %d\n", err);

  this->blockingRead();

  return true;
}

void Sliders::print_state()
{
  // ascii art is awesome
  printf("          ");
  for (unsigned i = 0; i < NUM_CHANNELS; i++)
    printf("%4.2f ", knobs[i]);
  printf("\n");
  printf("%d %d %d     ", buttons[0], buttons[1], buttons[2]);
  for (unsigned i = 0; i < NUM_CHANNELS; i++)
    printf("%4.2f ", sliders[i]);
  printf("\n");
  printf("%d %d %d     ", buttons[3], buttons[4], buttons[5]);
  for (unsigned i = 0; i < NUM_CHANNELS; i++)
    printf("%d    ", channel_buttons[i][0]);
  printf("\n");
  printf("          ");
  for (unsigned i = 0; i < NUM_CHANNELS; i++)
    printf("%d    ", channel_buttons[i][1]);
  printf("\n\n");
}

