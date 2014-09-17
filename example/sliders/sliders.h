#ifndef SLIDERS_H
#define SLIDERS_H

#include <stdint.h>
#include <map>
#include <vector>
#include "RtMidi.h"

class Sliders
{
public:
  Sliders();
  ~Sliders();
  bool open(const char *midi_dev_name);
  bool rt_midi_open(int port);
  RtMidiIn midi_in;

  //void onSliderChanged(double _deltatime, std::vector<unsigned char> *_message,
  //  void *_userData );
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

