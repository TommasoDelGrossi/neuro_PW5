#include "data_logger.hpp"
#include <iostream>
#include <chrono>

// COSTRUTTORE

DataLogger::DataLogger(const std::string &filename) : is_open(false)
{
    file.open(filename);
    if (file.is_open())
    {
        is_open = true;
        file << "timestamp,"
             << "working_mode_1,working_mode_2,working_mode_3,working_mode_4,"
             << "pos_1_deg,pos_2_deg,pos_3_deg,pos_4_deg,"
             << "speed_1_deg_per_s,speed_2_deg_per_s,speed_3_deg_per_s,speed_4_deg_per_s,"
             << "torque_1_Nm,torque_2_Nm,torque_3_Nm,torque_4_Nm,"
             << "stim_current_1,stim_current_2,stim_current_3,stim_current_4,stim_current_5,"
             << "\n";
    }
    else
    {
        std::cerr << "Errore: impossibile aprire il log file." << std::endl;
    }

    // Inizializzazione thread
    buffer.reserve(1000);
    keep_running = true;
    worker_thread = std::thread(&DataLogger::background_task, this);
}

// DISTRUTTORE
DataLogger::~DataLogger()
{
    keep_running = false;
    if (worker_thread.joinable())
    {
        worker_thread.join();
    }

    if (file.is_open())
    {
        file.close();
    }
}

// SCRITTURA

bool DataLogger::write(const LogData &data)
{
    if (!file.is_open())
    {
        return false;
    }

    std::lock_guard<std::mutex> lock(mtx);
    buffer.push_back(data);

    return true;
}

void DataLogger::background_task()
{
    while (keep_running)
    {
        std::vector<LogData> local_buffer;

        {
            std::lock_guard<std::mutex> lock(mtx);
            local_buffer.swap(buffer);
        }

        if (!local_buffer.empty() && file.is_open())
        {
            for (const auto &riga : local_buffer)
            {
                file << riga.timestamp << ","

                     << riga.working_mode[0] << ","
                     << riga.working_mode[1] << ","
                     << riga.working_mode[2] << ","
                     << riga.working_mode[3] << ","

                     << riga.position_deg[0] << ","
                     << riga.position_deg[1] << ","
                     << riga.position_deg[2] << ","
                     << riga.position_deg[3] << ","

                     << riga.speed_deg_per_sec[0] << ","
                     << riga.speed_deg_per_sec[1] << ","
                     << riga.speed_deg_per_sec[2] << ","
                     << riga.speed_deg_per_sec[3] << ","

                     << riga.torque_Nm[0] << ","
                     << riga.torque_Nm[1] << ","
                     << riga.torque_Nm[2] << ","
                     << riga.torque_Nm[3] << ","

                     << riga.stim_current[0] << ","
                     << riga.stim_current[1] << ","
                     << riga.stim_current[2] << ","
                     << riga.stim_current[3] << ","
                     << riga.stim_current[4] << ","

                     << "\n";
            }
            file.flush();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
