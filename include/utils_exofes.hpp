#ifndef UTILS_EXOFES_H
#define UTILS_EXOFES_H

#include <string>
#include <map>
#include <csignal>

#include "utils.h"
#include "timer.hpp"

#define N_LOG_MAX 200
#define DRIVER_T_OUT 300
#define ALPHA_FILTER_TORQUE 0.8   // 0.75
#define ALPHA_FILTER_VELOCITY 0.8 // 0.65
#define T_SAMPLE 0.01

#define DEBUG 0

#if DEBUG
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...) ((void)0)
#endif

namespace exofes::config
{

    // Parametri per un profilo di impedenza
    struct ImpedanceParams
    {
        float Kp[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float Kd[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    };

    // configurazione del robot = profilo di impedenza + saturazione coppia
    struct RobotConfig
    {
        std::map<std::string, ImpedanceParams> profiles;
        float torque_limits[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        float resting_position[4] = {0.0f, 0.0f, 0.0f, 0.0f};
        int app_version = 1; // versione del file di configurazione, utile per eventuali aggiornamenti futuri
    };

    // carica e scrivi la config da un json
    RobotConfig load_configuration(const std::string &filepath);
    bool write_resting_position(const std::string& filepath, const float resting_position[4]);

} // namespace exofes::config

typedef enum
{

    INITIALIZE_MASTER_STATE,
    WAIT_MASTER,
    INITIALIZE_DRIVER_STATE,
    WAIT_DRIVER,
    DRIVER_OK

} driver_initialization_state_enum;

typedef enum
{

    WAIT_START_MOTION_STATE1,
    IN_MOTION1,
    WAITING_DEFINED_TIME,
    WAIT_CONCLUDED

} wait_end_of_motion_state_enum;

typedef struct // struttura dati per logging
{
    // stesso enum di utils.h -> axis_data_t
    uint8_t error;
    uint8_t warning;
    uint8_t on_target_position;
    uint8_t motion_ongoing;
    driver_state_enum driver_state;
    driver_working_mode_enum working_mode;
    uint16_t warning_code;
    float position_deg;
    int16_t position_bit;
    float motor_freq;
    int16_t torque_bit;

    // aggiunte per log e funzioni aggiuntive
    float position_rad;
    float speed_deg_per_sec;
    float speed_rad_per_sec;
    float torque_Nm;

} polimi_axis_data_struct;

// dichiarazione struttura dati. Extern perché verrà letta da tutti gli script in questo ambiente
extern polimi_axis_data_struct polimi_axis_data[4];
extern volatile sig_atomic_t running;
extern float log_desired_pos[4];

typedef enum
{ // definizione dei profili di rigidezza per l'impedenza

    POSITION_CONTROL = -1,
    STIFF_IMPEDANCE = 3,
    COMPLIANT_IMPEDANCE = 2,
    GRAVITY = 1, // TODO: rigidezza nulla, poco smorzamento, solo compensazione gravità

} impedance_profiles;

typedef struct {
    float x;
    float y;
    float z;
} cartesian_position_t;

//================= utilities functions =====================================//

// update polimi data
void update_polimi_axis_data(void);

/*
 * @brief: Aggiorna i dati dal master e restituisce l'attuale posizione cartesiana
 */
cartesian_position_t get_cartesian_position(void);

/*
 * @brief: Calcola la distanza euclidea 3D tra due punti cartesiani
 */
float calculate_performance(cartesian_position_t p1, cartesian_position_t p2);

// set diver in specificied working modes
bool set_global_working_mode(driver_working_mode_enum driver_modes[]);

//=================== gravity compensation =========================================//

float gravity_torque_4_only_robot(float q2, float q3, float q4);

float gravity_torque_2_only_robot(float q2, float q3, float q4);

// ===========controllo exo============================

namespace exofes::control
{

    class ExoController
    {
    private:
        // --- STATO INTERNO PROTETTO ---

        // Stato per il controllo di Posizione
        float m_old_pose[4];
        bool m_command_sent;
        Timer m_pos_timer; // Cronometro dedicato alla posizione

        // Stato per il controllo Anti-Gravità
        Timer m_antig_timer; // Cronometro dedicato all'AntiG

        // Stato per il controllo di Impedenza
        Timer m_imp_timer; // Cronometro dedicato alla traiettoria (delta_t)

    public:
        // Costruttore: imposta i valori iniziali di sicurezza
        ExoController();

        // Reset d'emergenza: azzera tutti i timer e gli stati
        void reset();

        // --- METODI DI CONTROLLO ---
        bool robotControl_Position(float *pose, int delay_ms);

        bool robotControl_AntiG(bool active_axes[], molla_parameters_t molla, int duration_ms);

        bool robotControl_Impedance(
            bool active_axes[],
            float starting_pos[],
            float ending_pos[],
            int duration_ms,
            molla_parameters_t molla,
            const exofes::config::ImpedanceParams &profile,
            const float torque_limits[4]);
    };

} // namespace exofes::control

#endif
