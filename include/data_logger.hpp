#ifndef DATALOGGER_HPP
#define DATALOGGER_HPP

#include <string>
#include <fstream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>
#include "utils.h"

struct LogData
{

    long long int timestamp;

    int16_t subtask;
    int16_t subtask_phase;

    float performance;

    driver_working_mode_enum working_mode[4];

    float position_deg[4];
    float position_desired[4];
    float speed_deg_per_sec[4];
    float torque_Nm[4];

    double stim_current[5];
    double gain_ILC[5];

    // uint8_t error[4];
    // uint8_t warning[4];
    // uint16_t warning_code[4];

};

class DataLogger
{
private:
    std::ofstream file;
    bool is_open;

    // Strumenti per il thread in background
    std::vector<LogData> buffer;      
    std::thread worker_thread;        
    std::mutex mtx;                   
    std::atomic<bool> keep_running;   

    void background_task(); // La funzione che girerà in background

public:
    // Costruttore
    DataLogger(const std::string &filename);

    // Distruttore
    ~DataLogger();

    // Metodo per scrivere
    bool write(const LogData &data);
};

#endif