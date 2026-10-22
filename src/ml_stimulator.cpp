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

void ml_stimulator::stimulation_calibration(Smpt_device* device, int channel, int current, int period , int pulsewidth){

    smpt_clear_ml_update(&ml_update);
    ml_update.enable_channel[channel] = true;  /* Enable channel red LEFT */
    ml_update.packet_number = packet_number++;

    ml_update.channel_config[channel].number_of_points = 3;  /* Set the number of points */
    ml_update.channel_config[channel].ramp = 3;              /* Three lower pre-pulses   */
    ml_update.channel_config[channel].period = period;

    /* Set the stimulation pulse */
    ml_update.channel_config[channel].points[0].time = pulsewidth;
    ml_update.channel_config[channel].points[1].time = 100;
    ml_update.channel_config[channel].points[2].time = pulsewidth;

    for (int i = 0; i < 5; i++){
        ml_update.channel_config[i].points[0].current = 0;
        ml_update.channel_config[i].points[2].current = 0;
    }

    ml_update.channel_config[channel].points[0].current = current;
    ml_update.channel_config[channel].points[2].current = -current;

    if(smpt_send_ml_update(device, &ml_update)){
        //printf("Calibration update sent: %d, %d, %d, %d\n", channel, current, period, pulsewidth);
    }
}



bool ml_stimulator::stimulate(Smpt_device* device, double stim_currents[5], int period, int pulsewidth){

    smpt_clear_ml_update(&ml_update);
    ml_update.packet_number = packet_number++;
    double cur;

    for (int ch = 0; ch < 5; ++ch) {
        cur = stim_currents[ch];

        ml_update.enable_channel[ch] = (cur != 0.0);

        ml_update.channel_config[ch].number_of_points = 3;
        ml_update.channel_config[ch].ramp = 0; // Rectangular envelope: no pre-pulse ramp.
        ml_update.channel_config[ch].period = period;

        ml_update.channel_config[ch].points[0].time = pulsewidth;
        ml_update.channel_config[ch].points[1].time = 100;
        ml_update.channel_config[ch].points[2].time = pulsewidth;

        ml_update.channel_config[ch].points[0].current = cur;
        ml_update.channel_config[ch].points[2].current = -cur;
    }

    return smpt_send_ml_update(device, &ml_update);
}
