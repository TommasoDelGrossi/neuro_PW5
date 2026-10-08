#include "ml_stimulator.h"
#include <cstdio>




ml_stimulator::ml_stimulator()
{

}

ml_stimulator::~ml_stimulator()
{

}

bool ml_stimulator::init_stimulation(Smpt_device* device){
    
    smpt_clear_ml_init(&ml_init);
    ml_init.packet_number = packet_number++;
    return smpt_send_ml_init(device, &ml_init);

}

bool ml_stimulator::stimulate(Smpt_device* device, double stim_currents[5], int period, int pulsewidth){
    
    ml_update.packet_number = packet_number++;
    double cur;

    for (int ch = 0; ch < 5; ++ch) {
        cur = stim_currents[ch];

        ml_update.enable_channel[ch] = (cur != 0.0);

        ml_update.channel_config[ch].number_of_points = 3;
        ml_update.channel_config[ch].ramp = 3;
        ml_update.channel_config[ch].period = period;

        ml_update.channel_config[ch].points[0].time = pulsewidth;
        ml_update.channel_config[ch].points[1].time = 100;
        ml_update.channel_config[ch].points[2].time = pulsewidth;

        ml_update.channel_config[ch].points[0].current = cur;
        ml_update.channel_config[ch].points[2].current = -cur;
    }

    return smpt_send_ml_update(device, &ml_update);
}