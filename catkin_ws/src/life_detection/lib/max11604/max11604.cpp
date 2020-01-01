/*
    max11604.cpp - Library for using the Maxim MAX11604 ADC with Arduino
    Created by Dominic Yamarone for NUROVER. April 19, 2019
    Released into the wild.

    Might work with other Maxim 8-bit I2C ADC's.
*/

#include "Arduino.h"
#include "max11604.h"
#include <Wire.h>

Max11604::Max11604(int numChans) {
    _numChans = numChans;
}

bool Max11604::begin()
{
    Wire.beginTransmission(addr);
    Wire.write(0x90);
    byte res = Wire.endTransmission();

    if (!res)
        return true;
    else
        return false;
}

int Max11604::readSingleChannel(int channel){
    byte channelSelect;
    int reading= 0;

    channelSelect = (byte)channel;
    channelSelect = channelSelect << 1;
    byte config = 0x31;
    config = config | channelSelect;

    Wire.beginTransmission(addr);
    Wire.write(config);
    Wire.endTransmission();

    delay(5);

    Wire.requestFrom(addr, 1);
    while(Wire.available())
    {
        if(!reading)
            reading = Wire.read();
        else
            Wire.read();
    }

    return reading;
}
void Max11604::readAllChannels(int readings[]){
    int i = 0;

    byte channelSelect = (byte) _numChans;
    channelSelect = channelSelect << 1;
    byte config = 0x01;
    config = config | channelSelect;

    Wire.beginTransmission(addr);
    Wire.write(config);
    Wire.endTransmission();

    delay(5);

    Wire.requestFrom(addr, _numChans);
    while(Wire.available())
    {
        readings[i] = Wire.read();
        i++;
    }
}
