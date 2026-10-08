#include "utils_exofes.hpp"

#include <iostream>
#include <fstream>
#include <cmath>
#include <cstring>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>

#include "grav_model.hpp"
#include "json.hpp"

using json = nlohmann::json;

namespace exofes::config
{

    RobotConfig load_configuration(const std::string &filepath)
    {
        RobotConfig config;
        std::ifstream file(filepath);

        if (!file.is_open())
        {
            std::cerr << "[ERROR] Impossibile aprire: " << filepath << ". Uso parametri a ZERO.\n";
            return config;
        }

        try
        {
            json j;
            file >> j; // Il parser legge il file e lo trasforma nell'oggetto 'j'

            // Caricamento dinamico dei profili
            if (j.contains("control_profiles"))
            {
                // Scorriamo ogni profilo trovato nel JSON
                for (auto &[profile_name, profile_data] : j["control_profiles"].items())
                {
                    for (int i = 0; i < 4; i++)
                    {
                        config.profiles[profile_name].Kp[i] = profile_data["Kp"][i];
                        config.profiles[profile_name].Kd[i] = profile_data["Kd"][i];
                    }
                }
            }

            // Caricamento dei limiti di coppia per i 4 giunti
            if (j.contains("torque_limits"))
            {
                for (int i = 0; i < 4; i++)
                {
                    config.torque_limits[i] = j["torque_limits"][i];
                }
            }

            if (j.contains("resting_position")){
            for (int i = 0; i < 4; i++)
            {
                config.resting_position[i] = j["resting_position"][i];
            }
            }
            if (j.contains("app_version")){
                config.app_version = j["app_version"];
            }

            std::cout << "[INFO] Configurazione caricata correttamente da: " << filepath << "\n";
            std::cout << "       Profili estratti: " << config.profiles.size() << "\n";
        }
        catch (const json::exception &e)
        {
            std::cerr << "[FATAL] Errore di formattazione o lettura nel file JSON: " << e.what() << "\n";
        }

        return config;
    }

    bool write_resting_position(const std::string& filepath, const float resting_position[4]){
        json j;
        // 1) Leggi file esistente
        {
            std::ifstream in(filepath);
            if (!in.is_open())
            {
                std::cerr << "[ERROR] Impossibile aprire in lettura: " << filepath << "\n";
                return false;
            }
            try
            {
                in >> j;
            }
            catch (const json::exception& e)
            {
                std::cerr << "[FATAL] JSON non valido in " << filepath << ": " << e.what() << "\n";
                return false;
            }
        }
        // 2) Aggiorna/crea solo la chiave "resting_position"
        j["resting_position"] = json::array();
        for (int i = 0; i < 4; ++i)
            j["resting_position"].push_back(resting_position[i]);
        // 3) Scrivi di nuovo il JSON (pretty print), preservando tutte le altre chiavi
        {
            std::ofstream out(filepath);
            if (!out.is_open())
            {
                std::cerr << "[ERROR] Impossibile aprire in scrittura: " << filepath << "\n";
                return false;
            }
            out << j.dump(4) << "\n";
        }
        return true;
    }
} 

// variabili per struttura dati assi

polimi_axis_data_struct polimi_axis_data[4];

// torque conversion parameters bit->Nm
float torque_sensor_sensitivity[4] = {-2.2894e-04, -12.5634e-04, 0, 3.9526e-04};

// filtraggio velocità
float previous_pos_deg[4] = {0, 0, 0, 0};
float previous_speed_deg_per_sec[4] = {0, 0, 0, 0};
float current_speed_deg_per_sec[4] = {0, 0, 0, 0};

// filtraggio coppia
int16_t previous_torque_bit[4] = {-14200, 0, 0, 0};

// variabili per settare modalità operativa driver
driver_working_mode_enum actual_driver_modes[4] = {STAND_STILL_AUTO, STAND_STILL_AUTO, STAND_STILL_AUTO, STAND_STILL_AUTO};
driver_initialization_state_enum driver_initialization_state = INITIALIZE_MASTER_STATE;
bool driver_initialized_flag = false;
bool working_mode_set = false;
int driver_initialization_t_out = 0;

// variabili per logger
FILE *log_file = NULL;
float log_desired_pos[4] = {0.0f, 0.0f, 0.0f, 0.0f};

// global variables

volatile sig_atomic_t running = 1;

long long int start_wait_time = -1;
long long int current_wait_time = 0;

wait_end_of_motion_state_enum wait_end_of_motion_state = WAIT_START_MOTION_STATE1;

extern bool command_to_exo;

//============utilities functions==============//

/*
 * @brief: funzione per aggiornare la struttura dati polimi_axis_data, che contiene i dati degli assi filtrati e convertiti in unità fisiche, a partire dai dati raw di axis_data
 * @param: void
 * @return: void
 */
void update_polimi_axis_data(void)
{
    update_exo_data();

    for (int i = 0; i < 4; i++)
    {
        // data from axis_data
        polimi_axis_data[i].error = axis_data[i].error;
        polimi_axis_data[i].warning = axis_data[i].warning;
        polimi_axis_data[i].on_target_position = axis_data[i].on_target_position;
        polimi_axis_data[i].motion_ongoing = axis_data[i].motion_ongoing;
        polimi_axis_data[i].driver_state = axis_data[i].driver_state;
        polimi_axis_data[i].working_mode = axis_data[i].working_mode;
        polimi_axis_data[i].warning_code = axis_data[i].warning_code;
        polimi_axis_data[i].position_deg = axis_data[i].position_deg;
        polimi_axis_data[i].position_bit = axis_data[i].position_bit;
        polimi_axis_data[i].motor_freq = axis_data[i].speed;

        // torque_filter
        polimi_axis_data[i].torque_bit = (1 - ALPHA_FILTER_TORQUE) * axis_data[i].torque + ALPHA_FILTER_TORQUE * previous_torque_bit[i];
        previous_torque_bit[i] = polimi_axis_data[i].torque_bit;

        polimi_axis_data[i].torque_Nm = polimi_axis_data[i].torque_bit * torque_sensor_sensitivity[i];

        // deg2rad
        polimi_axis_data[i].position_rad = polimi_axis_data[i].position_deg * DEG2RAD;

        // speed
        current_speed_deg_per_sec[i] = (polimi_axis_data[i].position_deg - previous_pos_deg[i]) / T_SAMPLE;
        current_speed_deg_per_sec[i] = (1 - ALPHA_FILTER_VELOCITY) * current_speed_deg_per_sec[i] + (ALPHA_FILTER_VELOCITY * previous_speed_deg_per_sec[i]);

        polimi_axis_data[i].speed_deg_per_sec = (1 - ALPHA_FILTER_VELOCITY) * current_speed_deg_per_sec[i] + (ALPHA_FILTER_VELOCITY * previous_speed_deg_per_sec[i]);
        polimi_axis_data[i].speed_rad_per_sec = polimi_axis_data[i].speed_deg_per_sec * DEG2RAD;

        previous_pos_deg[i] = polimi_axis_data[i].position_deg;
        previous_speed_deg_per_sec[i] = polimi_axis_data[i].speed_deg_per_sec;
    }
}

cartesian_position_t get_cartesian_position(void)
{
    cartesian_position_t pos;

    // conversione da gradi a metri (esempio semplificato, da adattare alla cinematica del robot)
    pos.x = master_data.x;
    pos.y = master_data.y;
    pos.z = master_data.z;

    return pos;
}

float calculate_performance(cartesian_position_t p1, cartesian_position_t p2)
{
    // Calcolo dell'errore (distanza euclidea)
    float error = std::sqrt(std::pow(p1.x - p2.x, 2) +
                            std::pow(p1.y - p2.y, 2) +
                            std::pow(p1.z - p2.z, 2));


    const float PERFECT_THRESHOLD = 20.0f;       //mm 
    const float FAIL_THRESHOLD = 200.0f;         //mm 
    
    const float MAX_SCORE = 100.0f;
    const float MIN_SCORE = 0.0f;

    // Interpolazione
    if (error <= PERFECT_THRESHOLD) 
    {
        return MAX_SCORE; // Esercizio perfetto
    } 
    else if (error >= FAIL_THRESHOLD) 
    {
        return MIN_SCORE; // Fuori dal range massimo
    } 
    else 
    {
        // Relazione lineare per i valori intermedi

        float d_x = FAIL_THRESHOLD - PERFECT_THRESHOLD; 
        float d_y = MIN_SCORE - MAX_SCORE;        
        float d_err = error - PERFECT_THRESHOLD;              
        
        // Calcolo della percentuale di penalità (da 0.0 a 1.0)

        float performance = d_y / d_x * d_err + MAX_SCORE; // Interpolazione lineare tra MAX_SCORE e MIN_SCORE
        
        return performance;
    }
}

/*
 * @brief: funzione per impostare la modalità operativa globale dei driver
 * @param: driver_working_mode_enum driver_modes[]: array di 4 elementi che contiene la modalità operativa desiderata per ciascun driver
 * @return: bool: true se la modalità operativa è stata impostata correttamente, false altrimenti
 */
bool set_global_working_mode(driver_working_mode_enum driver_modes[])
{
    // controlla la modalità operativa dei driver
    bool mode_changed = false;

    for (int kk = 0; kk < 4; kk++)
    {

        if (actual_driver_modes[kk] != driver_modes[kk])
        { // se anche uno degli assi ha un comando diverso, true

            mode_changed = true;
            break;
        }
    }

    if (mode_changed)
    {

        for (int kk = 0; kk < 4; kk++)
        {

            actual_driver_modes[kk] = driver_modes[kk];
        }

        driver_initialization_state = INITIALIZE_MASTER_STATE;
        working_mode_set = false; // la funzione ritorna questo bool. è falso fino alla fine
        return false;
    }

    // controllo timeout, se è passato troppo tempo dall'inizio dell'inizializzazione, resetto tutto
    if (driver_initialization_state != DRIVER_OK &&  // se non sono ancora in DRIVER_OK
        driver_initialization_t_out >= DRIVER_T_OUT) // 300 cicli -> 3 secondi

    {
        // printf("TIME-OUT REACHED, re-initializing\n");
        driver_initialization_state = INITIALIZE_MASTER_STATE;
        working_mode_set = false;
    }

    switch (driver_initialization_state)
    {

    case INITIALIZE_MASTER_STATE:

        driver_initialization_t_out = 0;
        set_master_working_mode(EXTERNAL_DEVELOPER_AUTO_WORKING_MODE); // TODO inserire controllo flag enable
        driver_initialization_state = WAIT_MASTER;
        working_mode_set = false;
        break;

    case WAIT_MASTER:

        if (master_data.master_working_mode == EXTERNAL_DEVELOPER_AUTO_WORKING_MODE)
        {

            driver_initialization_state = INITIALIZE_DRIVER_STATE;
        }

        working_mode_set = false;
        break;

    case INITIALIZE_DRIVER_STATE: // set driver working mode

        for (int kk = 0; kk < 4; kk++)
        {

            set_axis_working_mode(kk, actual_driver_modes[kk]);
        }

        driver_initialization_state = WAIT_DRIVER;
        working_mode_set = false;
        break;

    case WAIT_DRIVER: // wait for driver to be initialized

        driver_initialized_flag = true;

        for (int kk = 0; kk < 4; kk++)
        {

            if (axis_data[kk].working_mode != actual_driver_modes[kk])
            {

                driver_initialized_flag = false;
                break; // basta un falso per uscire
            }
        }

        DEBUG_PRINT("[SET_MODE] WAITDRIVER — checking axes: ");
        for (int kk = 0; kk < 4; kk++)
            DEBUG_PRINT("ax%d: actual=%d desired=%d | ", kk, axis_data[kk].working_mode, actual_driver_modes[kk]);
        DEBUG_PRINT("\n");

        if (driver_initialized_flag == true)
        {

            // printf("driver ok\n");
            driver_initialization_state = DRIVER_OK;
        }

        working_mode_set = false;
        break;

    case DRIVER_OK:

        if (driver_initialized_flag == true && master_data.master_working_mode == EXTERNAL_DEVELOPER_AUTO_WORKING_MODE)
        {
            working_mode_set = true;
            // printf("driver working\n");
        }
        else // in teoria non dovrebbe mai entrare qui, se è tutto ok rimane in DRIVER_OK, se qualcosa va storto torna in INITIALIZE_MASTER_STATE
        {
            working_mode_set = false;
            driver_initialization_state = INITIALIZE_MASTER_STATE;
            // printf("re-initializing\n");
        }

        DEBUG_PRINT("[SET_MODE] DRIVER_OK — working_mode_set=%d\n", working_mode_set);

        break;

    default:

        // printf("ERROR: invalid driver state\n");
        working_mode_set = false;
        driver_initialization_state = INITIALIZE_MASTER_STATE;
        break;
    }

    if (driver_initialization_state != DRIVER_OK)
    {
        driver_initialization_t_out++;
    }

    return working_mode_set;
}

//=========================funzioni di gravità==============================

float gravity_torque_4_only_robot(float q2, float q3, float q4)
{
    float c2, s2, c3, /*s3,*/ c4, s4;

    c2 = cosf(q2 + 1.57f);
    s2 = sinf(q2 + 1.57f);
    c3 = cosf(q3);
    // s3 = sinf(q3);
    c4 = cosf(q4);
    s4 = sinf(q4);

    return 0.219f * s4 * c2 - 6.29e-4f * c4 * c2 + 0.219f * s2 * c3 * c4 + 6.29e-4f * s2 * c3 * s4;
}

float gravity_torque_2_only_robot(float q2, float q3, float q4)
{
    float c2, s2, c3, s3, c4, s4;

    c2 = cosf(q2 + 1.57f);
    s2 = sinf(q2 + 1.57f);
    c3 = cosf(q3);
    s3 = sinf(q3);
    c4 = cosf(q4);
    s4 = sinf(q4);

    float result =
        //- 0.40123532f
        +51.28310334f * s4 * c3 * c4 * c2 - 1.10743029f * s3 * c2 - 0.030343204f * c4 * c3 * c2 - 57.12593312f * s2 * c3 * c3 - 0.15963014f * s4 * c3 * c2 - 5.84282978f * s2 * s3 * s3
        //- 6.09282978f * s2 * s3 * s3   // - 5.84282978f * s2 * s3 * s3
        - 51.28310334f * c4 * c3 * s4 * c2 - 1.10743029f * c4 * c4 * s3 * c2 - 0.049351079f * c3 * c2 - 0.15963014f * s2 * s3 * c4 * s3 - 1.10743029f * s4 * s3 * s4 * c2 + 0.030343204f * s2 * s3 * s3 * s4 - 0.15963014f * s2 * c3 * c4 * c3 + 0.030343204f * s2 * s4 * c3 + 51.28310334f * s2 * c3 * c4 * c3 * c4 - 1.10743029f * s2 * c3 * c4 * s3 * s4 + 51.28310334f * s2 * c3 * s4 * c3 * s4 + 1.10743029f * s2 * c3 * s4 * c4 * s3;

    return result;
}

//===================controllo exo=========================

namespace exofes::control
{

    // ==============================================================================
    // 1. COSTRUTTORE E RESET
    // ==============================================================================

    ExoController::ExoController()
    {
        reset(); // Riutilizziamo la logica di reset per inizializzare l'oggetto
    }

    void ExoController::reset()
    {
        // 1. Azzera i flag e le vecchie posizioni
        m_command_sent = false;
        for (int i = 0; i < 4; i++)
        {
            m_old_pose[i] = -9999.0f; // Valore sentinella
        }

        // 2. Blocca tutti i cronometri
        m_pos_timer.reset();
        m_antig_timer.reset();
        m_imp_timer.reset();
    }

    // ==============================================================================
    // 2. CONTROLLO DI POSIZIONE
    // ==============================================================================

    /*
     * @brief: funzione per condare il robot con il profilatore interno di posizione
     * @param: float *pose: array di 4 elementi che contiene la posizione target in gradi per ciascun asse
     * @return: bool: true se il comando è stato inviato, false altrimenti
     */

    bool ExoController::robotControl_Position(float *pose, int delay_ms)
    {
        if (!m_command_sent)
        {
            if (pose[0] != m_old_pose[0] || pose[1] != m_old_pose[1] ||
                pose[2] != m_old_pose[2] || pose[3] != m_old_pose[3])
            {
                if (!master_data.motion_ongoing && !command_to_exo)
                {
                    for (int i = 0; i < 4; i++)
                    {
                        set_target2(i, TARGET_POSITION_DEG, pose[i]);
                        m_old_pose[i] = pose[i];
                        log_desired_pos[i] = pose[i];
                    }
                    command_to_exo = true;
                    m_command_sent = true;
                    m_pos_timer.start(); // Parte il timer del metodo
                }
            }
        }

        if (m_command_sent)
        {
            // Sfrutto la classe timer
            bool timer_done = m_pos_timer.has_elapsed(delay_ms);

            if (timer_done && !master_data.motion_ongoing && !command_to_exo)
            {
                m_command_sent = false;
                DEBUG_PRINT("[POS_CTRL] DONE — reached target\n");
                for (int i = 0; i < 4; i++)
                    m_old_pose[i] = -9999.0f;
                return true;
            }
        }

        return false;
    }

    // ==============================================================================
    // 3. CONTROLLO ANTI-GRAVITÀ
    // ==============================================================================

    /*
     * @brief: funzione per portare il robot in compensazione della gravità
     * @param: bool active_axes[]: array di 4 elementi che specifica su quali assi usare il controllo in coppia
     * @param: molla_parameters_t molla: struttura che contiene i parametri della molla per la compensazione AGS
     * @param: int duration_ms: durata del controllo in modalità AGS in millisecondi
     * @return: bool: true se il tempo è passato, false altrimenti
     */

    bool ExoController::robotControl_AntiG(bool active_axes[], molla_parameters_t molla, int duration_ms)
    {
        float tau4_g = 0, tau2_g = 0, AGS_torque = 0;
        float antig_gain[4] = {0.0f, 1.0f, 0.0f, 0.5f};
        float tau_ref[4] = {0.0};

        float pos_rad_model[4] = {
            polimi_axis_data[J1].position_rad,
            polimi_axis_data[J2].position_rad,
            polimi_axis_data[J3].position_rad,
            polimi_axis_data[J4].position_rad};

        antig_output_t antig = {0};
        antig_compute_from_pos(pos_rad_model, &antig);

        tau2_g = antig_gain[J2] * antig.tau_gravity_Nm[J2] / torque_sensor_sensitivity[J2];
        tau4_g = antig_gain[J4] * antig.tau_gravity_Nm[J4] / torque_sensor_sensitivity[J4];
        AGS_torque = (molla.k * polimi_axis_data[1].position_deg + molla.offset);

        if (molla.hand)
        {
            tau_ref[1] = +tau2_g + AGS_torque;
            tau_ref[3] = -tau4_g;
        }
        else
        {
            tau_ref[1] = -tau2_g - AGS_torque;
            tau_ref[3] = +tau4_g;
        }

        // Saturazione Hardcoded TODO integrare questo controllo in Impedance
        for (int ii = 0; ii < 4; ii++)
        {
            switch (ii)
            {
            case 3:
                if (tau_ref[ii] > 8000)
                    tau_ref[ii] = 8000;
                else if (tau_ref[ii] < -8000)
                    tau_ref[ii] = -8000;
                break;
            case 1:
                if (tau_ref[ii] > 10000)
                    tau_ref[ii] = 10000;
                else if (tau_ref[ii] < -10000)
                    tau_ref[ii] = -10000;
                break;
            default:
                if (tau_ref[ii] > 20000)
                    tau_ref[ii] = 20000;
                else if (tau_ref[ii] < -20000)
                    tau_ref[ii] = -20000;
                break;
            }
        }

        for (int ii = 0; ii < 4; ii++)
        {
            log_desired_pos[ii] = polimi_axis_data[ii].position_deg;
            if (active_axes[ii])
                set_target2(ii, TARGET_TORQUE, tau_ref[ii]);
        }

        // La condizione di uscita è il completamento del timer
        return m_antig_timer.has_elapsed(duration_ms);
    }

    // ==============================================================================
    // 4. CONTROLLO DI IMPEDENZA
    // ==============================================================================

    /*
     * @brief: funzione per usare il robot in modalità impedenza
     * @param: bool active_axes[]: array di 4 elementi che specifica su quali assi usare il controllo in coppia
     * @param: float starting_pos[]: array di 4 elementi che contiene la posizione iniziale in gradi per ciascun asse
     * @param: float ending_pos[]: array di 4 elementi che contiene la posizione finale in gradi per ciascun asse
     * @param: int duration_ms: durata del movimento in millisecondi
     * @param: impedance_profiles profile: enum che definisce il profilo di impedenza da usare
     * @param: molla_parameters_t molla: struttura che contiene i parametri della molla per la compensazione AGS
     * @return: bool: true se il comando è stato inviato, false altrimenti
     */

    bool ExoController::robotControl_Impedance(bool active_axes[], float starting_pos[], float ending_pos[], int duration_ms, molla_parameters_t molla, const exofes::config::ImpedanceParams &profile, const float torque_limits[4])
    {
        float tau4_g = 0, tau2_g = 0, AGS_torque = 0;
        float antig_gain[4] = {0.0f, 1.0f, 0.0f, 1.0f};
        float target_pos[4], pos_error[4] = {0.0}, tau_ref_imp[4] = {0.0}, tau_ref[4] = {0.0};

        m_imp_timer.start(); // start timer

        // con get_time abbiamo il tempo passato dal lancio del timer
        float delta_t = (float)(m_imp_timer.get_elapsed_time_ms()) / duration_ms;
        if (delta_t > 1.0f)
            delta_t = 1.0f;

        float sigma = 10 * powf(delta_t, 3) - 15 * powf(delta_t, 4) + 6 * powf(delta_t, 5);

        for (int i = 0; i < 4; i++)
        {
            target_pos[i] = starting_pos[i] + (ending_pos[i] - starting_pos[i]) * sigma;
            log_desired_pos[i] = target_pos[i];
        }

        float pos_rad_model[4] = {
            polimi_axis_data[J1].position_rad,
            polimi_axis_data[J2].position_rad,
            polimi_axis_data[J3].position_rad,
            polimi_axis_data[J4].position_rad};

        antig_output_t antig = {0};
        antig_compute_from_pos(pos_rad_model, &antig);

        tau2_g = antig_gain[J2] * antig.tau_gravity_Nm[J2] / torque_sensor_sensitivity[J2];
        tau4_g = antig_gain[J4] * antig.tau_gravity_Nm[J4] / torque_sensor_sensitivity[J4];
        AGS_torque = (molla.k * polimi_axis_data[1].position_deg + molla.offset);

        for (int ii = 0; ii < 4; ii++)
        {
            pos_error[ii] = target_pos[ii] - polimi_axis_data[ii].position_deg;
            tau_ref_imp[ii] = profile.Kp[ii] * pos_error[ii] - profile.Kd[ii] * polimi_axis_data[ii].speed_deg_per_sec;
        }

        if (molla.hand)
        {
            tau_ref[0] = -tau_ref_imp[0];
            tau_ref[1] = tau_ref_imp[1] + tau2_g + AGS_torque;
            tau_ref[3] = tau_ref_imp[3] - tau4_g;
        }
        else
        {
            tau_ref[0] = -tau_ref_imp[0];
            tau_ref[1] = -tau_ref_imp[1] - tau2_g - AGS_torque;
            tau_ref[3] = -tau_ref_imp[3] + tau4_g;
        }

        // Saturazione pilotata dal JSON
        for (int ii = 0; ii < 4; ii++)
        {
            if (tau_ref[ii] > torque_limits[ii])
                tau_ref[ii] = torque_limits[ii];
            else if (tau_ref[ii] < -torque_limits[ii])
                tau_ref[ii] = -torque_limits[ii];
        }

        for (int ii = 0; ii < 4; ii++)
        {
            if (active_axes[ii])
                set_target2(ii, TARGET_TORQUE, tau_ref[ii]);
        }

        // Criterio di uscita
        if (delta_t >= 1.0f)
        {
            m_imp_timer.reset(); // Reset timer
            return true;
        }

        return false;
    }

} // namespace exofes::control