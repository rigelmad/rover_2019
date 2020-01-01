/*
    max11604.h - Library for using the Maxim MAX11604 ADC with Ardiuno.
    Created by Dominic Yamarone for NUROVER, April 19, 2019
    Released into the wild.
*/

#ifndef max11604_h
#define max11604_h

#include "Arduino.h"

const int addr = 0x65;

class Max11604
{
    public:
        Max11604(int numChans);
        bool begin();
        int readSingleChannel(int channel);
        void readAllChannels(int readings[]);
    private:
        int _numChans;
};

#endif
