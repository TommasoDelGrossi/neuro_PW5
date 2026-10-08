#ifndef ML_STIMULATOR_H
#define ML_STIMULATOR_H

#pragma once

#include "smpt_ml_client.h"

class ml_stimulator
{
public:
    Smpt_device stim_device;
    Smpt_ml_init ml_init;
    Smpt_ml_update ml_update;
    uint8_t packet_number = 0;
    
    ml_stimulator();
    ~ml_stimulator();
    bool init_stimulation(Smpt_device* device);
    void stimulation_calibration(Smpt_device* device, int channel, int current,int period, int pulsewidth);
    bool stimulate(Smpt_device* device, double stim_currents[5], int period, int pulsewidth);

private:

};

#endif