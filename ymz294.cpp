/*
 * Based on http://developer.mbed.org/users/yamaguch/code/YMZ294/
 *
 */
#include "ymz294.h"

YMZ294::YMZ294(
    PinName dataPin0, PinName dataPin1, PinName dataPin2, PinName dataPin3,
    PinName dataPin4, PinName dataPin5, PinName dataPin6, PinName dataPin7,
    PinName wrcsPin, PinName a0Pin, PinName icPin) :
    dataPins(dataPin0, dataPin1, dataPin2, dataPin3, dataPin4, dataPin5, dataPin6, dataPin7),        
    wrcsPin(wrcsPin), a0Pin(a0Pin), icPin(icPin)
{
    reset();
}

void YMZ294::reset() {
    for (int i=0; i<3; i++) {
        writeData(ADDR_FREQ + (i << 1), 0, 2);
        writeData(ADDR_VOL + i, 0);
    }

    writeData(ADDR_MIXER, 0x3f);
    writeData(ADDR_NOISE, 0);
    writeData(ADDR_ENV_FREQ, 0, 2);
    writeData(ADDR_ENV_SHAPE, 0);

    wrcsPin = 1;
    a0Pin = 0;
    icPin = 0;
    wait_ms(10);
    icPin = 1;
}

void YMZ294::setTone(Ch channel, float freq) {
    // ft = fsc / 16TP
    int tp = 125000.0 / freq;
    writeData(ADDR_FREQ + (channel << 1), tp & 0x0fff, 2);
}

void YMZ294::setNote(Ch channel, uint8_t note) {
    int table[] = {
        15289, 14430, 13620, 12856, 12134, 11453, 10810, 10204, 9631, 9090, 8580, 8099, 7644, 7215, 6810, 6428, 6067, 5726, 5405, 5102, 4815, 4545, 4290, 4049, 3822, 3607, 3405, 3214, 3033, 2863, 2702, 2551, 2407, 2272, 2145, 2024, 1911, 1803, 1702, 1607, 1516, 1431, 1351, 1275, 1203, 1136, 1072, 1012, 955, 901, 851, 803, 758, 715, 675, 637, 601, 568, 536, 506, 477, 450, 425, 401, 379, 357, 337, 318, 300, 284, 268, 253, 238, 225, 212, 200, 189, 178, 168, 159, 150, 142, 134, 126, 119, 112, 106, 100, 94, 89, 84, 79, 75, 71, 67, 63, 59, 56, 53, 50, 47, 44, 42, 39, 37, 35, 33, 31, 29, 28, 26, 25, 23, 22, 21, 19, 18, 17, 16, 15, 14, 14, 13, 12, 11, 11, 10, 9,
    };
    writeData(ADDR_FREQ + (channel << 1), table[note % 128] & 0x0fff, 2);
}

void YMZ294::muffleTone(Ch channel) {
    setVolume(channel, 0);
    writeData(ADDR_FREQ + (channel << 1), 0, 2);
}

void YMZ294::setVolume(Ch channel, int volume) {
    writeData(ADDR_VOL + channel, volume & 0x1f);
}

void YMZ294::setNoise(float freq) {
    // fn = fsc / 16NP
    int np = 125000.0 / freq;
    writeData(ADDR_NOISE, np & 0x1f);
}

void YMZ294::setEnvelope(float freq, int shape) {
    // fe = fsc / 256EP
    int ep = 7812.5 / freq;
    writeData(ADDR_ENV_FREQ, ep & 0xffff, 2);
    writeData(ADDR_ENV_SHAPE, shape);
}

void YMZ294::setMixer(Mixer m0, Mixer m1, Mixer m2, Mixer m3, Mixer m4, Mixer m5) {
    writeData(ADDR_MIXER, m0 & m1 & m2 & m3 & m4 & m5);
}

void YMZ294::write(char data, bool dataWrite) {
    wrcsPin = 0;
    a0Pin = dataWrite;
    dataPins = data;
    wrcsPin = 1;
}

void YMZ294::writeData(char address, int data, int n) {
    for (int i=0; i<n; i++) {
        write(address++);
        write((data >>= (8 * i)) & 0xff, true);
    }
}
