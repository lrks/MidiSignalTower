/*
 * Based on http://developer.mbed.org/users/yamaguch/code/YMZ294/
 *
 */
#include "mbed.h"

#define ADDR_FREQ   0x00
#define ADDR_NOISE  0x06
#define ADDR_MIXER  0x07
#define ADDR_VOL    0x08
#define ADDR_ENV_FREQ   0x0b
#define ADDR_ENV_SHAPE  0x0d

enum Ch {CHANNEL_A = 0, CHANNEL_B = 1, CHANNEL_C = 2};
enum Mixer {
    NONE = 0x3f,
    TONE_A = ~(1 << 0), TONE_B = ~(1 << 1), TONE_C = ~(1 << 2),
    NOISE_A = ~(1 << 3), NOISE_B = ~(1 << 4), NOISE_C = ~(1 << 5),
};

class YMZ294
{
    public:
        YMZ294(PinName dataPin0, PinName dataPin1, PinName dataPin2, PinName dataPin3, PinName dataPin4, PinName dataPin5, PinName dataPin6, PinName dataPin7, PinName wrcsPin, PinName a0Pin, PinName icPin);
        void reset();
        void setTone(Ch channel, float freq);
        void setNote(Ch channel, uint8_t note);
        void muffleTone(Ch channel);
        void setVolume(Ch channel, int volume);
        void setNoise(float freq);
        void setEnvelope(float freq, int shape);
        void setMixer(Mixer m0, Mixer m1 = NONE, Mixer m2 = NONE, Mixer m3 = NONE, Mixer m4 = NONE, Mixer m5 = NONE);
    private:
        void write(char data, bool dataWrite = false);
        void writeData(char address, int data, int n = 1);
        BusOut dataPins;
        DigitalOut wrcsPin;
        DigitalOut a0Pin;
        DigitalOut icPin;
};
