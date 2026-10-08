#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <thread>
#include <algorithm>
#include <iomanip>
#include <cmath>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>
#include <semaphore.h>
#include <time.h>
#include <signal.h>

#include "smpt_ml_client.h"
#include "smpt_beta.h"

#include "utils.h"
#include "utils_exofes.hpp"
#include "messageParser.hpp"
#include "grav_model.hpp"
#include "data_logger.hpp"
#include "timer.hpp"

#include "calibration.h"
#include "robot_modes.h"
#include "ilc_constants.h"
#include "beta_stimulation1.h"
#include "user_id.h"
#include "common_ilc.h"
#include "exercise_map.h"

#include "ml_stimulator.h"

#define MAX_BUFFER_SIZE 256
#define FES_CHANNELS 5

#define B_ANTI_G_CALIB 1
#define B_ENABLE_ALLYPLAY 2
#define B_HUM_ROT_10 3
#define B_HUM_ROT_N10 4
#define B_SAVE_REST_POS 121

using namespace std;
using namespace std::chrono;
using namespace exofes::parser;

string userID;

ml_stimulator p24_stimulator;
double current[NUM_MUSCLES] = {0};

// aggiunto Robi
Smpt_device stim_device;
Smpt_ml_init ml_init;
Smpt_ml_update ml_update;
uint8_t packet_number = 0;

/* ── AllyArm Web Gui ────────────────────────────────────── */
int web_gui_button = 0;
bool anti_g_calib_flag = false;

// float act_pos[NUM_JOINTS];
float des_pos[NUM_JOINTS];
double k = 0;

molla_parameters_t molla;
float target_pos[4];

double get_current_time_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // tempo monotono (no salti)

    return (ts.tv_sec * 1000.0) + (ts.tv_nsec / 1e6);
}
double TIME = 0;
double time_offset = 0.0;
bool was_active = false;

double stim_current[NUM_MUSCLES] = {0};
double G[NUM_MUSCLES] = {0.6};
int exercise_t = 0;
int t_ms = 0;
bool flag_ILC = true;
bool stimulation = false;

double err_sum[NUM_JOINTS] = {0}; // somma errori per ogni giunto
double err_mean[NUM_JOINTS] = {0};
int count_err = 0;
int rep_old = -1;
// double Imin[NUM_MUSCLES];
// double Imax[NUM_MUSCLES];
// CALIBRAZIONE
// double Imin[5] = {14, 12, 14, 14, 5};
// double Imax[5] = {30, 28, 28, 18, 14};
double Imin[5] = {10, 10, 10, 10, 10};
double Imax[5] = {20, 20, 20, 20, 20};
// double Imin[5] = {1, 1, 1, 1, 1};
// double Imax[5] = {6, 6, 6, 6, 6};

double theta_start[NUM_JOINTS] = {0};
double theta_end[NUM_JOINTS] = {0};

// extern FesCalibrationChannel g_start_fes_calibration[FES_CHANNELS]; // mi arriva da altro file

// Stati dello stimolatore
typedef enum
{
    STIM_WAIT = 0, // inizializzazione / attesa
    STIM_IDLE = 1, // pronto ma non manda impulsi
    STIM_ON = 2,   // invio impulso
    STIM_OFF = 3,  // fermo stimolatore
    STIM_ERROR = 4 // errore
} stimulator_mode_t;

// Variabile globale
stimulator_mode_t stim_mode = STIM_WAIT;                          // parte da WAIT
array<vector<array<double, NUM_MUSCLES>>, N_TASKS> beta_patterns; // pattern beta per ogni task/esercizio
bool file_opened = false;                                         // flag file CSV aperto
ofstream CSV_user_file;                                           // stream per scrivere il file CSV

uint8_t buffer[200];
ParsedMessage msg;
ParseResult result;
float random_position[4] = {96.2, -32.3, -8.2, 71.5};
float resting_position[4] = {94, -46, -55, 104};
// float target_0[4] ={50, 13, 0, 20};
// float target_1[4] ={92, 13, 0, 20};
// float target_2[4] ={124, 13, 0, 20};
// float target_3[4] ={125, -21, 0, 100};

float target_0[4] = {0.0}; //{60, 0, -55, 15};
float target_1[4] = {0.0}; //{87, 0, -55, 15}; // 87 primo
float target_2[4] = {0.0}; //{115, 0, -55, 15};
float target_3[4] = {0.0}; //{80, -11, -30, 110};

// per razzo
// float target_0[4] = {115, -45, -55, 15};
// float target_1[4] = {115, 10, -55, 15};
bool init_mod_flag = 0;
bool torque_axis[4] = {true, true, false, true}; // axis working with torque control

driver_working_mode_enum driver_position[4] = {POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER};
driver_working_mode_enum driver_torque[4] = {TORQUE, TORQUE, STAND_STILL_AUTO, TORQUE};
driver_working_mode_enum driver_stop[4] = {SOFT_STOP, SOFT_STOP, SOFT_STOP, SOFT_STOP};
driver_working_mode_enum driver_torque_calib[4] = {TORQUE, TORQUE, DRIVER_OFF, TORQUE};
// driver_working_mode_enum driver_torque[4] = {TORQUE_CUSTOM, TORQUE_CUSTOM, STAND_STILL_AUTO, TORQUE_CUSTOM};

float deadband[N_JOINT] = {50.0, 50.0, 50.0, 50.0};
float k_p_torque[N_JOINT] = {0.001, 0.016, 0.04, 1.0};
float k_d_torque[N_JOINT] = {0.0, 0.0, 0.02, 0.0};
float k_e_torque[N_JOINT] = {1.6, 1.6, 1.6, 1.6};

exofes::control::ExoController Ally;    // oggetto exo, contiene le leggi di controllo
exofes::config::RobotConfig exo_config; // file di configurazione dell'exo con i profili di impedenza e i limiti di coppia

Timer rest_timer;       // cronometro per gestire i tempi di riposo tra le fasi
Timer stim_timer;       // cronometro per gestire i tempi di stimolazione
Timer setup_timer;      // parte a stop game e aspetta 2 sec prima di riportare in resting position
Timer correction_timer; // cronometro per gestire i tempi di correzione della posizione
string profile;         // aggiornato dal gioco, determina la modalità di assistenza (impedance_level)

// gestione asse 3
bool manual_jog_active = false;
float manual_jog_target[4] = {0.0f};

// dati per calcolare performance
cartesian_position_t pos_arrivo_impedenza;
cartesian_position_t pos_target_reale;
float performance_rep = 0.0f;

bool boot_init_flag = false;

int counter = 0;

// sottostati per la transizione di modalità driver
typedef enum
{
    TO_TORQUE,
    IMPEDANCE_CONTROL,
    TO_POSITION,
    POSITION_CORRECTION,
    SUBTASK_COMPLETE
} MotionSubState;

MotionSubState motion_sub_state = TO_TORQUE;

float last_ending_pos[4] = {0.0f};
int correction_time_ms = 0;
bool success_flag = true;

#define CORRECTION_THRESHOLD 7.0f // 7deg per definire success
#define MS_PER_DEGREE 50.0f       // 20deg/s

// log

std::unique_ptr<DataLogger> logger;
LogData dataTDG;

Timer state_timer; // cronometro per gestire i tempi di ogni stato e subtask

long long int start_time_ms;

// transparency control parameters

/*____Motion state variables____*/

bool command_to_exo = false;
int tick = 0;
bool stop_pressed = false;

/* ── Subtask IDs ─────────────────────────────────────────────── */
typedef enum
{
    NOT_IN_GAME = -1,  /* not in game mode                    */
    WAIT_START = 0,    /* wait start                          */
    REST_TO_START = 1, /* rest to start                       */
    START_TO_T1 = 2,   /* start to target 1                   */
    REST = 3,          /* rest                                */
    T1_TO_START = 4,   /* target 1 to start                   */
    START_TO_REST = 5, /* start to rest                       */
    T1_TO_END = 6,     /* target 1 to end                     */
    END_TO_START = 7,  /* end to start                        */
    FREE_MOVEMENT = 8, /* free movement                       */
    START_TO_T2 = 9,   /* start to target 2                   */
    T2_TO_START = 10,  /* target 2 to start                   */
    T2_TO_END = 11,    /* target 2 to end                     */
    START_TO_T3 = 12,  /* start to target 3                   */
    T3_TO_START = 13,  /* target 3 to start                   */
    T3_TO_END = 14,    /* target 3 to end                     */
} Subtask;

/* ── Game mode IDs ───────────────────────────────────────────── */
typedef enum
{
    MODE_NONE = 0,
    MODE_RAZZO = 1,
    MODE_ASTRONAVE = 2,
    MODE_PLANET = 3,
    MODE_ROVER = 4
} GameMode;

/* ── Sequences ───────────────────────────────────────────── */
const int astro[3][6] = {
    {START_TO_T1, REST, T1_TO_END, REST, END_TO_START, REST}, /* phase 0 — 5 steps */
    {START_TO_T2, REST, T2_TO_END, REST, END_TO_START, REST}, /* phase 1 — 5 steps */
    {START_TO_T3, REST, T3_TO_END, REST, END_TO_START, REST}, /* phase 2 — 6 steps */
};
const int astro_len[3] = {6, 6, 6};

const int planet[3][4] = {
    {START_TO_T1, REST, T1_TO_START, REST}, /* phase 0 — 4 steps */
    {START_TO_T2, REST, T2_TO_START, REST}, /* phase 1 — 4 steps */
    {START_TO_T3, REST, T3_TO_START, REST}, /* phase 2 — 3 steps */
};
const int planet_len[3] = {4, 4, 4};

const int razzo[] = {START_TO_T1, REST, REST_TO_START, REST};
const int razzo_len = 4;

const int rover[] = {REST_TO_START, FREE_MOVEMENT, START_TO_REST};
const int rover_len = 3;

/* ── FSM variables ───────────────────────────────────────── */
int16_t state = NOT_IN_GAME;
u_int8_t mode = MODE_NONE;
u_int8_t phase = 0;
u_int8_t step = 0;
uint16_t rep = 0;
bool subtask_done = false;
bool stim_flag = false;

static void compute_correction(float ending_pos[])
{
    correction_time_ms = 0;
    for (int i = 0; i < 4; i++)
    {
        last_ending_pos[i] = ending_pos[i];
        float err = fabsf(polimi_axis_data[i].position_deg - ending_pos[i]);
        int t = (int)(err * MS_PER_DEGREE);
        if (t > correction_time_ms)
            correction_time_ms = t;
    }
    if (correction_time_ms < 300)
        correction_time_ms = 300;
    if (correction_time_ms > 3000)
        correction_time_ms = 3000;
}

std::string getProfile(int profile_id)
{
    switch (profile_id)
    {
    case POSITION_CONTROL:
        return "POSITION_CONTROL";
    case STIFF_IMPEDANCE:
        return "STIFF_IMPEDANCE";
    case COMPLIANT_IMPEDANCE:
        return "COMPLIANT_IMPEDANCE";
    case GRAVITY:
        return "GRAVITY";
    default:
        return "STIFF_IMPEDANCE";
    }
}

void update_ilc_gains(int mode, int prev_trigger_value, double *err_mean, double *Imin, double *Imax);

void compute_stimulation(int state, int mode, int rep, int motion_sub_state);

void loop()
{
    counter++;
    // bool stim_time_elapsed=stim_timer.has_elapsed(3000);

    web_gui_button = get_button_pushed();

    if (web_gui_button == B_SAVE_REST_POS)
    {
        for (int i = 0; i < 4; i++)
        {
            resting_position[i] = polimi_axis_data[i].position_deg;
        }
        if (exofes::config::write_resting_position("./exo_params.json", resting_position))
        {
            printf("Resting position saved to json: [%f, %f, %f, %f]\n", resting_position[0], resting_position[1], resting_position[2], resting_position[3]);
        }
    }
    else if (web_gui_button == B_ANTI_G_CALIB)
    {
        anti_g_calib_flag = !anti_g_calib_flag;
        printf("Anti-gravity calibration mode: %s\n", anti_g_calib_flag ? "ON" : "OFF");
    }
    else if (web_gui_button == B_HUM_ROT_10)
    {
        if (!manual_jog_active) // Accetta il comando solo se non si sta già muovendo
        {
            for (int i = 0; i < 4; i++)
            {
                manual_jog_target[i] = polimi_axis_data[i].position_deg;
            }
            manual_jog_target[2] += 10.0f;

            manual_jog_active = true;
            printf("Humeral rotation +10deg \n");
        }
    }
    else if (web_gui_button == B_HUM_ROT_N10)
    {
        if (!manual_jog_active)
        {
            for (int i = 0; i < 4; i++)
            {
                manual_jog_target[i] = polimi_axis_data[i].position_deg;
            }
            manual_jog_target[2] -= 10.0f;

            manual_jog_active = true;
            printf("Humeral rotation -10deg \n");
        }
    }

    if (stim_flag)
    {
        p24_stimulator.stimulation_calibration(&p24_stimulator.stim_device, g_stim_channel, g_stim_current, 1000 / g_stim_frequency, g_stim_pulse_width);
        stim_flag = false;
    }
    else
    {

        // p24_stimulator.stimulation_calibration(&p24_stimulator.stim_device, 0, 0, 1000 / 30, 400);
        // smpt_send_ml_stop(&p24_stimulator.stim_device, p24_stimulator.packet_number);
    }

    // if (counter == 100)
    // {
    //     printf("STATE: %d \n", state);
    //     printf("STIM_STATE: %d \n", stim_mode);
    //     counter = 0;
    // }

    ssize_t msg_size = read_message_from_tcp(buffer, sizeof(buffer));
    update_polimi_axis_data();

    long int current_start_time = (long int)get_current_time_ms();

    if (master_data.on_target_position && command_to_exo)
    {
        tick++;
        if (tick > 5)
        {
            command_to_exo = false; // ero già in target desiderato
            // printf("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
            tick = 0;
        }
    }

    if (master_data.motion_ongoing == command_to_exo)
    {
        command_to_exo = false;
    }

    if (!boot_init_flag) // allo start, init exo in position
    {
        boot_init_flag = set_global_working_mode(driver_stop);
    }

    // aggiunto Robi
    // ── 0. Controllo stato stimolatore ──────────────
    switch (stim_mode)
    {
    case STIM_WAIT:
        // inizializza correnti, resettare timer
        for (int i = 0; i < NUM_MUSCLES; i++)
        {
            stim_current[i] = 0;
            G[i] = 0.6;
        }
        exercise_t = 0;

        break;

    case STIM_IDLE:

        for (int i = 0; i < NUM_MUSCLES; i++)
        {
            current[i] = 0;
        }

        stimulation = p24_stimulator.stimulate(&p24_stimulator.stim_device, current, 1000 / (int)g_start_fes_calibration[0].frequency, (int)g_start_fes_calibration[0].pulse_width);
        break;

    case STIM_ON:

        for (int i = 0; i < NUM_MUSCLES; i++)
        {
            current[i] = stim_current[i];
        }

        stimulation = p24_stimulator.stimulate(&p24_stimulator.stim_device, current, 1000 / (int)g_start_fes_calibration[0].frequency, (int)g_start_fes_calibration[0].pulse_width);
        printf("Stim update sent: %d\n", stimulation);
        break;

    case STIM_OFF:

        smpt_send_ml_stop(&p24_stimulator.stim_device, p24_stimulator.packet_number);
        smpt_close_serial_port(&p24_stimulator.stim_device);
        exit(0);
        break;

    case STIM_ERROR:
        stim_mode = STIM_WAIT;
        break;
    }

    // ================================================================
    // PARSE THE MESSAGE
    // ================================================================
    if (msg_size > 0)
    {
        uint8_t msg_id = buffer[0];

        // printf("\n[POLIMI] ══════════════════════════════════════════════════════════\n");
        printf("[POLIMI] Message ID: 0x%02X (%d)\n", msg_id, msg_id);
        // printf("[POLIMI] Message Size: %ld bytes\n", msg_size);

        print_message_hex(buffer, msg_size);

        result = parse_message(buffer, msg_size, &msg);
        printf("  [RESULT PARSER]: %d\n", result);

        if (result == PARSE_OK)
        {

            // Generic print — works for any message type
            print_parsed_message(&msg);
            update_globals_from_msg(&msg);
            switch (msg_id)
            {
            case AUTENTICAZIONE:
                printf("  [AUTENTICATE]\n");
                antig_set_parameters(g_auth_patient_weight, g_auth_patient_height);
                // antig_set_parameters(53, 158);
                antig_init_human_coupling(0.3f);
                if (exo_config.app_version == 1)
                {
                    printf("  [APP VERSION]: 1 correct\n");
                    set_game_answer_info_byte(successo, 0);
                }
                else
                {
                    printf("  [APP VERSION]: error\n");
                    set_game_answer_info_byte(successo, 1);
                }
                break;
            case INVIA_STIMOLO:
                set_game_answer_info_short(tempo_attesa, 3);
                if (!stim_flag)
                {
                    set_game_answer_info_byte(successo, 0);
                    printf("  [STIMULATION]\n");
                    printf("    EXO mode         : %u\n", g_stim_exo_mode);
                    printf("    Channel          : %d\n", g_stim_channel);
                    printf("    Frequency        : %d Hz\n", g_stim_frequency);
                    printf("    Current          : %d mA\n", g_stim_current);
                    printf("    Pulse width      : %d µs\n", g_stim_pulse_width);
                    stim_flag = 1;
                    // p24_stimulator.stimulation_calibration(&p24_stimulator.stim_device, 0, 7, 1000 / 30, 400);
                }
                else
                    set_game_answer_info_byte(successo, 1);

                break;
            case CORREZIONE_ANTI_G:
            {
                float alpha = 0.5f + ((float)g_anti_g_correction * 0.1f);
                if (alpha < 0.0f)
                {
                    alpha = 0.0f;
                }
                if (alpha > 1.2f)
                {
                    alpha = 1.2f;
                }
                antig_init_human_coupling(alpha);
                set_game_answer_info_byte(successo, 0);

                break;
            }
            case GIOCO_START:
                logger = std::make_unique<DataLogger>("/home/devuser/workspace/data_log/File_Log.csv");
                start_time_ms = (long long int)get_current_time_ms();
                // controllo che le variabili arrivate abbiano senso e do success
                set_global_working_mode(driver_position);
                set_game_answer_info_byte(successo, 0);
                mode = g_start_game_id;
                mode = g_start_game_id;
                performance_rep = 0.0f;
                // mode=1;
                // mode = 4;
                // TAKE THE TARGETS FROM THE START GAME MESSAGE
                for (int i = 0; i < 4; i++)
                {
                    target_0[i] = g_start_pose[0][i];
                    target_1[i] = g_start_pose[1][i];
                    target_2[i] = g_start_pose[2][i];
                    target_3[i] = g_start_pose[3][i];
                }
                profile = getProfile(g_start_assistance_level);
                // profile = "POSITION_CONTROL";
                // PARAMTERO DI LIVELLO ASSISTENZA
                for (int m = 0; m < NUM_MUSCLES; m++)
                {
                    Imin[m] = g_start_fes_calibration[m].current_min;
                    Imax[m] = g_start_fes_calibration[m].current_max;
                    // Imin[m] = 4;
                    // Imax[m] = 8;
                    g_start_fes_calibration[m].pulse_width = 400;
                    g_start_fes_calibration[m].frequency = 30;
                }
                state = WAIT_START;
                break;

            case GIOCO_STOP:

                mode = 0;
                phase = 0;
                step = 0;
                rep = 0;
                motion_sub_state = TO_TORQUE;
                subtask_done = false;

                command_to_exo = false;
                tick = 0;

                Ally.reset();
                rest_timer.reset();
                stim_timer.reset();

                if (logger != nullptr)
                {
                    logger.reset();
                }
                set_game_answer_info_byte(successo, 0);
                state = NOT_IN_GAME;
                stop_pressed = 1;

                break;

            case GET_PERFORMANCE:
            {
                // float score = 20 + (rand() % 80);
                printf("Performance: %f", performance_rep);
                set_game_answer_info_float(performance, performance_rep);
                set_game_answer_info_byte(successo, 0);
                break;
            }

            case STATO_EXO:
                set_game_answer_info_ulong(timestamp, (uint32_t)get_current_time_ms());
                set_game_answer_info_ushort(current_rep, rep);
                set_game_answer_info_short(subtask, state);
                set_game_answer_info_byte(successo, 0);
                break;

            case INVIA_LOG:
                set_game_answer_info_byte(successo, 0);
                break;

            case ESOSCHELETRO_IN_MODALITA_GAME:
                set_game_answer_info_byte(game, 1);
                set_game_answer_info_byte(successo, 0);
                break;

            case RICHIESTA_ROM:
                set_game_answer_info_float(rest0, resting_position[0]);
                set_game_answer_info_float(rest1, resting_position[1]);
                set_game_answer_info_float(rest2, resting_position[2]);
                set_game_answer_info_float(rest3, resting_position[3]);
                set_game_answer_info_byte(successo, 0);
                break;

            default:
                break;
            }
        }
    }

    /* ── 1. EXECUTE current state ─────────────────────────── */
    switch (state)
    {

    case NOT_IN_GAME:

        mode = MODE_NONE;
        phase = 0;
        step = 0;
        rep = 0;
        stop_pressed = 0;

        // motion_sub_state = TO_TORQUE;
        subtask_done = false;

        if (!manual_jog_active && !anti_g_calib_flag)
        {
            Ally.reset();
            rest_timer.reset();
            stim_timer.reset();
            state_timer.reset();
        }

        if (anti_g_calib_flag)
        {
            printf("Anti-gravity calibration mode: ON\n");
            switch (motion_sub_state)
            {
            case TO_TORQUE:
                if (set_global_working_mode(driver_torque))
                    motion_sub_state = IMPEDANCE_CONTROL;
                break;

            case IMPEDANCE_CONTROL:
                if (Ally.robotControl_AntiG(torque_axis, molla, 600000)) //(int)(g_start_repetition_times[0] * 1000)   // 120000
                    motion_sub_state = TO_POSITION;
                break;

            case TO_POSITION:
                if (set_global_working_mode(driver_position))
                    motion_sub_state = SUBTASK_COMPLETE;
                break;

            case SUBTASK_COMPLETE:
                motion_sub_state = TO_TORQUE;
                break;

            default:
                break;
            }
        }
        else
        {

            if (manual_jog_active)
            {
                switch (motion_sub_state)
                {
                case TO_TORQUE:
                    if (set_global_working_mode(driver_position))
                        motion_sub_state = IMPEDANCE_CONTROL;
                    break;

                case IMPEDANCE_CONTROL:
                    if (Ally.robotControl_Position(manual_jog_target, 1000))
                    {
                        motion_sub_state = TO_POSITION;
                    }
                    break;

                case TO_POSITION:
                    if (set_global_working_mode(driver_stop))
                        motion_sub_state = SUBTASK_COMPLETE;
                    break;

                case SUBTASK_COMPLETE:
                    motion_sub_state = TO_TORQUE;
                    manual_jog_active = false;
                    break;

                default:
                    break;
                }
            }
            else
            {
                // Nessun jog in corso, forziamo il soft stop in sicurezza
                set_global_working_mode(driver_stop);
                motion_sub_state = TO_TORQUE;
            }
        }
        DEBUG_PRINT("\n[NOT_IN_GAME   (-1)] Not in game mode — waiting for selection.\n");
        break;

    case WAIT_START:
    {
        if (!file_opened)
        {
            open_rehamove_file();
            file_opened = true;
        }

        if (set_global_working_mode(driver_position))
        {

            if (mode == MODE_RAZZO)
            {

                if (Ally.robotControl_Position(target_0, 5000))
                {
                    DEBUG_PRINT("[WAIT_START     (0)] Wait start — mode selected: %d\n", mode);
                    state = razzo[step];
                }
            }

            else
            {

                if (Ally.robotControl_Position(resting_position, 5000))
                {
                    DEBUG_PRINT("[WAIT_START     (0)] Wait start — mode selected: %d\n", mode);
                    if (mode == MODE_ASTRONAVE)
                        state = astro[phase][step];
                    else if (mode == MODE_PLANET)
                        state = planet[phase][step];
                    else
                        state = rover[step];
                }
            }
        }

        // aggiunto Robi
        generate_beta_patterns_for_selected_exercise(mode, g_start_repetition_times[0] * 1000); // devo passare in ingresso a questa le correnti di calibrazione da variabili globali. forse devo passare anche la durata.
        // beta_stimulation_1 con in input correnti calibrate che leggo da variabili globali

        break;
    }

    case REST_TO_START:

        if (mode == MODE_RAZZO)
        {
            subtask_done = Ally.robotControl_Position(target_0, g_start_repetition_times[2] * 1000);
        }
        else
        {
            subtask_done = Ally.robotControl_Position(target_0, 3000);
        }
        break;

    case START_TO_T1:

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:

            if (mode == MODE_RAZZO)
            {
                if (Ally.robotControl_Impedance(torque_axis, target_0, target_1, (int)(g_start_repetition_times[0] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
                {
                    compute_correction(target_1);
                    pos_arrivo_impedenza = get_cartesian_position();
                    motion_sub_state = TO_POSITION;
                }
            }
            else
            {
                for (int i = 0; i < NUM_JOINTS; i++)
                {
                    theta_start[i] = target_0[i]; // oppure target_1/2 se vuoi precisione
                    theta_end[i] = target_1[i];
                }
                if (Ally.robotControl_Impedance(torque_axis, resting_position, target_0, (int)(g_start_repetition_times[0] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
                {
                    compute_correction(target_0);
                    pos_arrivo_impedenza = get_cartesian_position();
                    motion_sub_state = TO_POSITION;
                }
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:

            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {

                    pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case REST:
        DEBUG_PRINT("[REST           (3)] Rest.\n");
        subtask_done = rest_timer.has_elapsed((int)(g_start_repetition_times[1] * 1000));
        break;

    case T1_TO_START:

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_0, resting_position, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(resting_position);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:

            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case START_TO_REST:
        subtask_done = Ally.robotControl_Position(resting_position, 1000);
        break;

    case T1_TO_END:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = target_0[i];
            theta_end[i] = target_3[i];
        }

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_0, target_3, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(target_3);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case END_TO_START:
        subtask_done = Ally.robotControl_Position(resting_position, 1500);
        break;

    case FREE_MOVEMENT:

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_AntiG(torque_axis, molla, (int)(g_start_repetition_times[0] * 1000))) //(int)(g_start_repetition_times[0] * 1000)   // 120000
                motion_sub_state = TO_POSITION;
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = SUBTASK_COMPLETE;
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            success_flag = true;
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;

        default:
            break;
        }

        break;

    case START_TO_T2:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = resting_position[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = target_1[i];
        }

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, resting_position, target_1, (int)(g_start_repetition_times[0] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(target_1);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case T2_TO_START:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = target_1[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = resting_position[i];
        }

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_1, resting_position, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(resting_position);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case T2_TO_END:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = target_1[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = target_3[i];
        }
        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_1, target_3, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(target_3);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case START_TO_T3:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = resting_position[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = target_2[i];
        }

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, resting_position, target_2, (int)(g_start_repetition_times[0] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(target_2);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case T3_TO_START:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = target_2[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = resting_position[i];
        }

        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_2, resting_position, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(resting_position);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    case T3_TO_END:

        for (int i = 0; i < NUM_JOINTS; i++)
        {
            theta_start[i] = target_2[i]; // oppure target_1/2 se vuoi precisione
            theta_end[i] = target_3[i];
        }
        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_Impedance(torque_axis, target_2, target_3, (int)(g_start_repetition_times[2] * 1000), molla, exo_config.profiles[profile], exo_config.torque_limits))
            {
                compute_correction(target_3);
                pos_arrivo_impedenza = get_cartesian_position();
                motion_sub_state = TO_POSITION;
            }
            break;

        case TO_POSITION:
            DEBUG_PRINT("[FSM state=%d] TO_POSITION | motion_ongoing=%d command_to_exo=%d\n",
                        state, master_data.motion_ongoing, command_to_exo);
            if (set_global_working_mode(driver_position))
                motion_sub_state = POSITION_CORRECTION;
            break;

        case POSITION_CORRECTION:
            if (setup_timer.has_elapsed(800)) // attendo 0.8 secondi per percepire la fine del movimento
            {
                if (Ally.robotControl_Position(last_ending_pos, correction_time_ms))
                {
                    cartesian_position_t pos_target_reale = get_cartesian_position();

                    performance_rep = calculate_performance(pos_arrivo_impedenza, pos_target_reale);

                    motion_sub_state = SUBTASK_COMPLETE;
                }
            }
            break;

        case SUBTASK_COMPLETE:
            DEBUG_PRINT("[FSM state=%d] SUBTASK_COMPLETE\n", state);
            motion_sub_state = TO_TORQUE;
            subtask_done = true;
            break;
        }

        break;

    default:
        DEBUG_PRINT("[UNKNOWN       (%d)] Unrecognised subtask.\n", state);
        break;

    } // end of execute switch

    // aggiunto Robi
    compute_stimulation(state, mode, rep, motion_sub_state);

    /* ── 2. ADVANCE — compute next state ──────────────────── */
    switch (state)
    {
    /* ·· NOT IN GAME MODE ······························· */
    case NOT_IN_GAME:

        // state = WAIT_START;

        break;

    /* ·· WAIT START (mode initialised) ·················· */
    case WAIT_START:

        break;

    /* ·· GAME SUBTASKS ··································· */
    default:
    {
        if (subtask_done)
        {
            subtask_done = false;
            step++;

            int seq_len;
            if (mode == MODE_ASTRONAVE)
            {
                seq_len = astro_len[phase];
            }
            else if (mode == MODE_RAZZO)
            {
                seq_len = razzo_len;
            }
            else if (mode == MODE_PLANET)
            {
                seq_len = planet_len[phase];
            }
            else
            {
                seq_len = rover_len;
            }

            if (step < seq_len)
            {
                /* more steps remaining in this repetition */
                if (mode == MODE_ASTRONAVE)
                    state = astro[phase][step];
                else if (mode == MODE_RAZZO)
                    state = razzo[step];
                else if (mode == MODE_PLANET)
                    state = planet[phase][step];
                else
                    state = rover[step];
            }
            else
            {
                /* ── end of one full repetition ──────────── */
                step = 0;
                rep++;

                if (mode == MODE_ROVER)
                {
                    printf("[Rover done]\n");
                    state = NOT_IN_GAME;
                }
                else if (mode == MODE_RAZZO)
                {
                    if (rep < g_start_repetitions)
                    {
                        printf("  [Razzo rep %d/%d — looping]\n", rep, g_start_repetitions);
                        state = razzo[step];
                    }
                    else
                    {
                        printf("  [Razzo done after %d reps]\n", g_start_repetitions);

                        // aggiunto Robi
                        close_rehamove_file();
                        file_opened = false;
                        stim_mode = STIM_WAIT;

                        state = NOT_IN_GAME;
                    }
                }
                else if (mode == MODE_PLANET)
                {
                    if (rep < g_start_repetitions)
                    {
                        printf("  [Planet phase %d — rep %d/%d — looping]\n",
                               phase, rep, g_start_repetitions);
                        state = planet[phase][step];
                    }
                    else
                    {

                        // aggiunto Robi
                        close_rehamove_file();
                        file_opened = false;
                        stim_mode = STIM_WAIT;

                        rep = 0;
                        phase++;
                        if (phase < 3)
                        {
                            open_rehamove_file();
                            file_opened = true;
                            printf("  [Planet → entering phase %d]\n", phase);
                            state = planet[phase][step];
                        }
                        else
                        {
                            printf("  [Planet done — all 3 phases complete]\n");
                            state = NOT_IN_GAME;
                        }
                    }
                }
                else
                { /* MODE_ASTRONAVE */
                    if (rep < g_start_repetitions)
                    {
                        printf("  [Astronave phase %d — rep %d/%d — looping]\n",
                               phase, rep, g_start_repetitions);
                        state = astro[phase][step];
                    }
                    else
                    {
                        // aggiunto Robi
                        close_rehamove_file();
                        file_opened = false;
                        stim_mode = STIM_WAIT;
                        rep = 0;
                        phase++;
                        if (phase < 3)
                        {
                            open_rehamove_file();
                            file_opened = true;
                            printf("  [Astronave → entering phase %d]\n", phase);
                            state = astro[phase][step];
                        }
                        else
                        {
                            printf("  [Astronave done — all 3 phases complete]\n");
                            state = NOT_IN_GAME;
                        }
                    }
                }
            }
        }
        if (stop_pressed)
        {
            state = NOT_IN_GAME;
            stop_pressed = 0;
            anti_g_calib_flag = false;
        }
        break;
    }
    } /* end advance switch */

    dataTDG.timestamp = current_start_time - start_time_ms;
    dataTDG.subtask = state;
    dataTDG.performance = performance_rep;

    for (int i = 0; i < 4; i++)
    {

        dataTDG.position_deg[i] = polimi_axis_data[i].position_deg;
        dataTDG.position_desired[i] = log_desired_pos[i];
        dataTDG.speed_deg_per_sec[i] = polimi_axis_data[i].speed_deg_per_sec;
        dataTDG.torque_Nm[i] = polimi_axis_data[i].torque_Nm;
        dataTDG.working_mode[i] = polimi_axis_data[i].working_mode;
        // dataTDG.error[i] = polimi_axis_data[i].error;
        // dataTDG.warning[i] = polimi_axis_data[i].warning;
        // dataTDG.warning_code[i] = polimi_axis_data[i].warning_code;
    }

    for (int j = 0; j < 5; j++)
    {

        dataTDG.stim_current[j] = stim_current[j];
        dataTDG.gain_ILC[j] = G[j];
    }

    // scrivi solo se il gioco è attivo

    if (state != NOT_IN_GAME && logger != nullptr)
    {
        logger->write(dataTDG);
    }
}
void compute_stimulation(int state, int mode, int rep, int motion_sub_state)
{
    static int prev_trigger_value = -2;

    double q_i = 0.0;

    bool flag_ILC = true;

    // =========================
    // LOGICA ILC -> fine fase
    // =========================
    if (state == REST)
    {
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            if (count_err > 0)
                // err_mean[i] = err_sum[i] / count_err;
                err_mean[i] = err_sum[i];
        }

        update_ilc_gains(mode, prev_trigger_value, err_mean, Imin, Imax);

        for (int i = 0; i < NUM_JOINTS; i++)
            err_sum[i] = 0;

        count_err = 0;

        exercise_t = 0;
    }

    // =========================
    // RESET TRA RIPETIZIONI
    // =========================
    if (rep != rep_old)
    {
        t_ms = 0;
    }

    prev_trigger_value = state;

    // =========================
    // FASI ATTIVE
    // =========================
    bool active_phase =
        (state == START_TO_T1 || state == START_TO_T2 || state == START_TO_T3 ||
         state == T1_TO_END || state == T2_TO_END || state == T3_TO_END ||
         state == T1_TO_START || state == T2_TO_START || state == T3_TO_START);
    // printf("STATE: %d | active_phase: %d\n", state, active_phase);

    if (active_phase && motion_sub_state == IMPEDANCE_CONTROL)
    {
        double TIME_abs = get_current_time_ms();
        if (!was_active)
        {
            time_offset = TIME_abs;
        }
        // 🔴 tempo relativo alla fase
        TIME = TIME_abs - time_offset;
        was_active = true;

        // printf("SCRIVO FILE\n");
        //  printf("state: %d \n", state);
        exercise_t++;

        update_exo_data();
        // convert_position_bits_to_degrees();
        // double t_theta = exercise_t * 10.0;                    // tempo corrente, all'interno della fase attiva, in ms.
        // double T_theta = g_start_repetition_times[0] * 1000.0; // durata totale della fase attiva in ms.

        // double tau_theta = t_theta / T_theta;

        // double s = 10 * pow(tau_theta, 3) - 15 * pow(tau_theta, 4) + 6 * pow(tau_theta, 5);
        for (int i = 0; i < NUM_JOINTS; i++)
        {
            // des_pos[i] = theta_start[i] + (theta_end[i] - theta_start[i]) * s;
            des_pos[i] = target_pos[i];
            err_sum[i] = polimi_axis_data[i].position_deg - des_pos[i]; // pos_des non c'è ancora!!!
        }

        count_err++;

        // =========================
        // CALCOLO CORRENTE
        // =========================
        for (int m = 0; m < NUM_MUSCLES; m++)
        {
            bool active = exercise_map[mode].phases[state][m];
            // printf("active: %d, mode: %d, state: %d \n, ", active, mode, state);

            if (!active)
            {
                stim_current[m] = 0.0;
                continue;
            }

            if (exercise_t < g_start_repetition_times[0] * 100)
            {
                if (m == 1 && mode == 2 && (state == T1_TO_END || state == T2_TO_END || state == T3_TO_END))
                {
                    q_i = k;
                }
                else
                {
                    q_i = beta_patterns[mode][exercise_t][m];
                }

                if (flag_ILC && (state != T1_TO_END && state != T2_TO_END && state != T3_TO_END))
                {
                    double a = (q_i - Imin[m]) * G[m] / 0.6;
                    q_i = Imin[m] + a;
                }
                if ((exercise_t == (g_start_repetition_times[0] * 100) - 10) && m == 1 && mode == 2 && (state == START_TO_T1 || state == START_TO_T2 || state == START_TO_T3))
                {
                    k = q_i;
                }
            }
            else
            {
                q_i = 0.0;
            }

            if (q_i < Imin[m])
                q_i = Imin[m];
            if (q_i > Imax[m])
                q_i = Imax[m];

            stim_current[m] = q_i;
        }

        // =========================
        // INVIO STIMOLAZIONE
        // =========================
        // write_rehamove_file();

        stim_mode = STIM_ON;
    }
    else if (state == NOT_IN_GAME)
    {
        stim_mode = STIM_WAIT;
    }
    else
    {
        was_active = false;
        time_offset = 0;
        TIME = 0;
        for (int i = 0; i < NUM_MUSCLES; i++)
        {
            stim_current[i] = 0;
        }
        stim_mode = STIM_IDLE;
    }

    // =========================
    //  LOG DATI
    // =========================
    // if (true)
    // write_rehamove_file();
    write_rehamove_file();
    t_ms += 10;
    rep_old = rep;
}

std::vector<std::array<double, NUM_MUSCLES>>
concat(const std::vector<std::array<double, NUM_MUSCLES>> &p1,
       const std::vector<std::array<double, NUM_MUSCLES>> &p2)
{
    std::vector<std::array<double, NUM_MUSCLES>> result;

    result.reserve(p1.size() + p2.size());

    result.insert(result.end(), p1.begin(), p1.end());
    result.insert(result.end(), p2.begin(), p2.end());

    return result;
}

// ── Restituisce i parametri beta per qualsiasi esercizio ─────────────
BetaParams get_beta_params_for_exercise(int /*mode*/)
{
    BetaParams p;
    p.t_start = 0;

    // Tutti i muscoli con stessi parametri
    p.p = {2, 2, 2, 2, 2};
    p.q = {1, 1, 1, 1, 1};

    return p;
}

void update_ilc_gains(int mode, int prev_trigger_value, double *err_mean, double *Imin, double *Imax)
{
    const double ALFA = 0.02;
    const double THRESH = 5;

    // conversione err_mean
    std::array<double, NUM_JOINTS> err_arr;
    for (int i = 0; i < NUM_JOINTS; i++)
        err_arr[i] = err_mean[i];
    // conversione err_mean
    std::array<double, NUM_MUSCLES> Imin_arr;
    for (int i = 0; i < NUM_MUSCLES; i++)
        Imin_arr[i] = Imin[i];

    std::array<double, NUM_MUSCLES> Imax_arr;
    for (int i = 0; i < NUM_MUSCLES; i++)
        Imax_arr[i] = Imax[i];

    // conversione G
    std::array<double, NUM_MUSCLES> G_arr;
    for (int i = 0; i < NUM_MUSCLES; i++)
        G_arr[i] = G[i];

    // chiamata funzione reale
    ::update_ilc_gains(G_arr, mode, prev_trigger_value, err_arr, ALFA, THRESH, Imin_arr, Imax_arr);

    // copia indietro
    for (int i = 0; i < NUM_MUSCLES; i++)
        G[i] = G_arr[i];
}

void open_rehamove_file()
{
    time_t now = time(NULL);

    char buffer[100];
    strftime(buffer, sizeof(buffer), "%Y-%m-%d_%H-%M-%S", localtime(&now));

    std::string filename = "./dati_rehamove/dati_" + std::string(buffer) + ".csv";
    CSV_user_file.clear();

    CSV_user_file.open(filename);
    if (CSV_user_file.is_open())
    {
        std::cout << "[CSV] File aperto: " << filename << std::endl;

        // Scrivi intestazione (puoi copiare quella che c'è già)
        CSV_user_file << "time_real,time_rep_ms,time_phase_10ms,"
                      << "stim_current1,stim_current2,stim_current3,stim_current4,stim_current5,"
                      << "act_pos_J1,act_pos_J2,act_pos_J3,act_pos_J4,"
                      << "des_pos_J1,des_pos_J2,des_pos_J3,des_pos_J4,"
                      << "err_J1,err_J2,err_J3,err_J4,"
                      //<< "act_torque_J1,act_torque_J2,act_torque_J3,act_torque_J4,"
                      //<< "des_torque_J1,des_torque_J2,des_torque_J3,des_torque_J4,"
                      << "flag_ILC,"
                      << "G_DA,G_DM,G_DP,G_T,G_B,"
                      << "act_torque_J1,act_torque_J2,act_torque_J3,act_torque_J4,"
                      << "act_velocity_J1,act_velocity_J2,act_velocity_J3,act_velocity_J4,"
                      << "mod_driver_J1,mod_driver_J2,mod_driver_J3,mod_driver_J4,"
                      << "subtask_state, motion_sub_state,exercise_mode,exercise_rep,"
                      << "TARGET_POS_J1,TARGET_POS_J2,TARGET_POS_J3,TARGET_POS_J4,"
                      << "motor_freq_J1, motor_freq_J2, motor_freq_J3, motor_freq_4"
                      << std::endl;
    }
    else
    {
        std::cout << "[CSV] Errore apertura file!" << std::endl;
    }
}

void write_rehamove_file()
{

    CSV_user_file << endl

                  << TIME << ","
                  << t_ms << ","       // tempo all'interno di una ripetizione
                  << exercise_t << "," // tempo all'interno di una fase
                  // << stim_counter << "," //conteggio cicli all'interno delle 25 ripetizioni
                  // << motion_command << ","
                  // << stim_mode << ","
                  //               << rehamove_mode << ","
                  //<< duration_total_ms << ","--> NON SO SE RICHIEDERLA PERCHè NON LA SO LA DURATA TOTALE, DIPENDE DAI RIPOSI E DAL TEMPO DEL ROBOT.
                  //               << time_scaling <<","
                  // << direction << ","
                  //               << amplitude_shoulder << ","
                  //               << amplitude_elbow << ","
                  //               << amplitude_shoulder_go << ","
                  //               << amplitude_elbow_go << ","
                  //               << amplitude_shoulder_back << ","
                  //               << amplitude_elbow_back << ","
                  //               << width_shoulder << ","
                  //               << width_elbow << ","
                  //               << charge[0] << ","
                  //               << charge[1] << ","
                  //               << charge[2] << ","
                  //               << charge[3] << ","
                  //               << charge[4] << ","
                  << stim_current[0] << ","
                  << stim_current[1] << ","
                  << stim_current[2] << ","
                  << stim_current[3] << ","
                  << stim_current[4] << ","
                  //               << fs.fes_State.stim_value[0]<< ","
                  //               << fs.fes_State.stim_value[1] << ","
                  //               << fs.fes_State.stim_value[2] << ","
                  //               << fs.fes_State.stim_value[3] << ","
                  //               << fs.fes_State.stim_value[4] << ","
                  //               << pulsewidth[0] << ","
                  //               << pulsewidth[1] << ","
                  //               << pulsewidth[2] << ","
                  //               << pulsewidth[3] << ","
                  //               << pulsewidth[4] << ","

                  << polimi_axis_data[0].position_deg << ","
                  << polimi_axis_data[1].position_deg << ","
                  << polimi_axis_data[2].position_deg << ","
                  << polimi_axis_data[3].position_deg << ","

                  << des_pos[0] << ","
                  << des_pos[1] << ","
                  << des_pos[2] << ","
                  << des_pos[3] << ","

                  << polimi_axis_data[0].position_deg - des_pos[0] << ","
                  << polimi_axis_data[1].position_deg - des_pos[1] << ","
                  << polimi_axis_data[2].position_deg - des_pos[2] << ","
                  << polimi_axis_data[3].position_deg - des_pos[3] << ","

                  //<< act_torque[0] << ","
                  //<< act_torque[1] << ","
                  //<< act_torque[2] << ","
                  //<< act_torque[3] << ","

                  //<< des_torque[0] << ","
                  //<< des_torque[1] << ","
                  //<< des_torque[2] << ","
                  //<< des_torque[3] << ","

                  //               << act_velocity[0] << ","
                  //               << act_velocity[1] << ","
                  //               << act_velocity[2] << ","
                  //               << act_velocity[3] << ",";
                  //               << stiffness[0] << ","
                  //               << stiffness[1] << ","
                  //               << stiffness[2] << ","
                  //               << stiffness[3] << ","
                  //               << damping[0] << ","
                  //              << damping[1] << ","
                  //               << damping[2] << ","
                  //               << damping[3] << ","
                  //               << mean_position_error_shoulder_go << ","
                  //               << mean_position_error_elbow_go << ","
                  //              << std_position_error_shoulder_go << ","
                  //              << std_position_error_elbow_go << ","
                  //               << mean_position_error_shoulder_back << ","
                  //               << mean_position_error_elbow_back << ","
                  //               << std_position_error_shoulder_back << ","
                  //               << std_position_error_elbow_back << ","
                  //               << upperarm_assistance << ","
                  //               << forearm_assistance << ","
                  //               << num_iter << ",";
                  //               << ones_count << ","
                  //               << two_count << ","
                  //               << three_count << ","
                  //               << c.data->counter_value << ","
                  //               << start_intention << ",";
                  << flag_ILC << ","
                  << G[0] << ","
                  << G[1] << ","
                  << G[2] << ","
                  << G[3] << ","
                  << G[4] << ","
                  << polimi_axis_data[0].torque_Nm << ","
                  << polimi_axis_data[1].torque_Nm << ","
                  << polimi_axis_data[2].torque_Nm << ","
                  << polimi_axis_data[3].torque_Nm << ","
                  << polimi_axis_data[0].speed_deg_per_sec << ","
                  << polimi_axis_data[1].speed_deg_per_sec << ","
                  << polimi_axis_data[2].speed_deg_per_sec << ","
                  << polimi_axis_data[3].speed_deg_per_sec << ","
                  << polimi_axis_data[0].working_mode << ","
                  << polimi_axis_data[1].working_mode << ","
                  << polimi_axis_data[2].working_mode << ","
                  << polimi_axis_data[3].working_mode << ","
                  << state << ","
                  << motion_sub_state << ","
                  << mode << ","
                  << rep << ","
                  << target_pos[0] << ","
                  << target_pos[1] << ","
                  << target_pos[2] << ","
                  << target_pos[3] << ","
                  << polimi_axis_data[0].motor_freq << ","
                  << polimi_axis_data[1].motor_freq << ","
                  << polimi_axis_data[2].motor_freq << ","
                  << polimi_axis_data[3].motor_freq;

    CSV_user_file.flush(); // forza la scrittura su disco
    // printf("[CSV RIGA OK] \n");
}

void close_rehamove_file()
{
    if (CSV_user_file.is_open())
    {
        CSV_user_file.close();
        cout << "USER data file closed correctly." << endl;
    }
}

void generate_beta_patterns_for_selected_exercise(int mode, int duration_phase_ms) // forse devo mandargli anche il tempo "duration_phase_ms"
{
    // ── Ottieni i parametri beta per l'esercizio selezionato ──
    BetaParams p1 = get_beta_params_for_exercise(mode);

    // Qui puoi impostare una durata in ms basata su quanto vuoi duri la fase 1
    p1.t_end = duration_phase_ms; // esempio: primo valore dei tempi di fase

    // Genera il pattern beta per la prima fase
    auto pattern1 = generate_beta_pattern_go_back(p1.t_end, p1, Imin, Imax);

    beta_patterns[mode] = pattern1;
}
// int main(void)
// {
//     printf("\n");
//     // printf("╔══════════════════════════════════════════════════════════════════╗\n");
//     // printf("║           POLIMI Application - Exoskeleton Game Control         ║\n");
//     // printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

//     if (init(loop) != 0)
//     {
//         fprintf(stderr, "[POLIMI_INIT] ❌ Initialization failed\n");
//         return 1;
//     }

//     printf("[POLIMI_INIT] ✅ All shared memories initialized\n");
//     printf("[POLIMI_INIT] ✅ TCP→POLIMI reader initialized\n");
//     printf("[POLIMI_INIT] ✅ POLIMI→TCP writer initialized\n");

//     // printf("\n╔══════════════════════════════════════════════════════════════════╗\n");
//     // printf("║       POLIMI initialized and waiting for TCP commands          ║\n");
//     // printf("╚══════════════════════════════════════════════════════════════════╝\n\n");

//     pid_t pid = fork();
//     if (pid < 0)
//     {
//         perror("fork failed");
//         return 1;
//     }

//     srand(time(NULL));

//     // printf("[POLIMI] Starting message receive loop...\n");
//     fflush(stdout);

//     while (1)
//     {
//     //usleep(100000); // 100ms sleep
//     }

//     return 0;
// }

int main(void)
{
    const char *port_name = "/dev/P24";
    p24_stimulator.stim_device = {0};
    smpt_open_serial_port(&p24_stimulator.stim_device, port_name);

    bool P24_initialized = p24_stimulator.init_stimulation(&p24_stimulator.stim_device);

    printf("P24 initialized: %s\n", P24_initialized ? "true" : "false");

    start_time_ms = (long long int)get_current_time_ms();

    Ally.reset();

    std::cout << "\n=== Init Control Params ===\n";
    exo_config = exofes::config::load_configuration("./exo_params.json");
    std::cout << "=========================================\n\n";

    resting_position[0] = exo_config.resting_position[0];
    resting_position[1] = exo_config.resting_position[1];
    resting_position[2] = exo_config.resting_position[2];
    resting_position[3] = exo_config.resting_position[3];

    // signal(SIGINT, handle_sigint);

    if (init(loop) != 0)
        return 1;

    printf("Shared memory reader started...\n");

    molla = get_molla_parameters();
    
    printf("Rigidezza: %f, Offset: %f, Mano: %d\n", molla.k, molla.offset, molla.hand);

    printf("Rigidezza: %f, Offset: %f, Mano: %d \n", molla.k, molla.offset, molla.hand);

    set_global_working_mode(driver_stop);

    // for (int i = 0; i < N_JOINT; i++)
    // {
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_DEAD_ZONE, deadband[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_PROPORTIONAL, k_p_torque[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_EXPONENTIAL, k_e_torque[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_DERIVATIVE, k_d_torque[i]);
    // }
    // for (int i = 0; i < N_JOINT; i++)
    // {
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_DEAD_ZONE, deadband[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_PROPORTIONAL, k_p_torque[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_EXPONENTIAL, k_e_torque[i]);
    //     set_working_parameters(i, TORQUE_CUSTOM_CONTROL_K_DERIVATIVE, k_d_torque[i]);
    // }

    pid_t pid = fork();
    if (pid < 0)
    {
        perror("fork failed");
        return 1;
    }

    while (running)
    {
        usleep(10000);
    }

    return 0;
}
