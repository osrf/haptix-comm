#ifndef SLIDERS_H
#define SLIDERS_H

#include <stdint.h>
#include <map>
#include <vector>
#include <alsa/asoundlib.h>

class Sliders
{
public:
  Sliders();
  ~Sliders();
  bool open(const char *midi_dev_name);
  bool midi_open(char *port);
  snd_rawmidi_t *midi_in;

  void blockingRead();
  bool poll();
  int fd;
  static const unsigned NUM_CHANNELS = 9, NUM_BUTTONS = 6;
  float sliders[NUM_CHANNELS];
  float knobs[NUM_CHANNELS];
  bool channel_buttons[NUM_CHANNELS][2];
  bool buttons[NUM_BUTTONS];
  std::map<uint8_t, float *> analog_control_map;
  std::map<uint8_t, bool *> digital_control_map;
  void print_state();

};

#endif

