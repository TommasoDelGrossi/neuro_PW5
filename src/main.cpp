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
#include <atomic>
#include <ctime>

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

#include "utils.h"
#include "utils_exofes.hpp"
#include "grav_model.hpp"
#include "data_logger.hpp"
#include "timer.hpp"
#include "ml_stimulator.h"

#define MAX_BUFFER_SIZE 256
#define FES_CHANNELS 1

using namespace std;
using namespace std::chrono;

/* ── P24 Variables ────────────────────────────────────── */
ml_stimulator p24_stimulator;
bool P24_initialized = false;

Smpt_device stim_device;
Smpt_ml_init ml_init;
Smpt_ml_update ml_update;
uint8_t packet_number = 0;

double current[NUM_MUSCLES] = {0};
double Imin[NUM_MUSCLES] = {10, 10, 10, 10, 10};
double Imax[NUM_MUSCLES] = {20, 20, 20, 20, 20};

// Stati dello stimolatore
typedef enum
{
    STIM_WAIT = 0, // inizializzazione / attesa
    STIM_IDLE = 1, // pronto ma non manda impulsi
    STIM_ON = 2,   // invio impulso
    STIM_OFF = 3,  // fermo stimolatore
    STIM_ERROR = 4 // errore
} stimulator_mode_t;
stimulator_mode_t stim_mode = STIM_WAIT;

/* ── AllyArm Variables ────────────────────────────────────── */

bool torque_axis[4] = {false, false, false, true}; // axis working with torque control

driver_working_mode_enum driver_position[4] = {POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER};
driver_working_mode_enum driver_torque[4] = {STAND_STILL_AUTO, STAND_STILL_AUTO, STAND_STILL_AUTO, TORQUE};
driver_working_mode_enum driver_stop[4] = {SOFT_STOP, SOFT_STOP, SOFT_STOP, SOFT_STOP};
driver_working_mode_enum driver_torque_calib[4] = {TORQUE, TORQUE, DRIVER_OFF, TORQUE};

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

exofes::control::ExoController Ally;    // oggetto exo, contiene le leggi di controllo
exofes::config::RobotConfig exo_config; // file di configurazione dell'exo con i profili di impedenza e i limiti di coppia

molla_parameters_t molla;

float target_pos[4] = {90.0f, 0.0f, 0.0f, 90.0f};

string profile; // aggiornato dal gioco, determina la modalità di assistenza (impedance_level)

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

bool command_to_exo = false;
int tick = 0;
bool stop_pressed = false;
bool stop_initialized = false;
std::atomic<bool> stop_confirmed{false};

bool positioning_active = false;
bool position_mode_ready = false;
bool position_reached = false;

bool exercise_active = false;
bool antig_90s_completed = false;

/* ── Generic Variables ────────────────────────────────────── */

double get_current_time_ms()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts); // tempo monotono (no salti)

    return (ts.tv_sec * 1000.0) + (ts.tv_nsec / 1e6);
}

Timer rest_timer;       // cronometro per gestire i tempi di riposo tra le fasi
Timer stim_timer;       // cronometro per gestire i tempi di stimolazione
Timer setup_timer;      // parte a stop game e aspetta 2 sec prima di riportare in resting position
Timer correction_timer; // cronometro per gestire i tempi di correzione della posizione
Timer state_timer;      // cronometro per gestire i tempi di ogni stato e subtask

long long int start_time_ms;

bool init_flag = false;

int counter = 0;

std::unique_ptr<DataLogger> logger;
LogData log_data;

typedef enum
{
    DEFAULT = 0,
    ENABLE_EXO = 1,
    GO_TO_POSITION = 2,
    ENABLE_FES = 3,
    RECTANGULAR = 4,
    TRAPEZOIDAL = 5,
    BIOMIMETIC = 6,
    START_EXERCISE = 7,
    STOP_EXERCISE = 8,
    EXIT = 9

} GUI_State;

std::atomic<GUI_State> requested_button{DEFAULT};
std::atomic<bool> stop_requested{false};

GUI_State gui_button = DEFAULT;

void compute_stimulation(int state, int mode, int rep, int motion_sub_state);

void loop()
{
    update_polimi_axis_data();

    GUI_State gui_button = requested_button.exchange(DEFAULT);

    if (stop_requested.exchange(false))
    {
        stop_pressed = true;
    }

    // Ignore commands until Exo is enabled
    if (!init_flag &&
        gui_button != DEFAULT &&
        gui_button != ENABLE_EXO &&
        gui_button != STOP_EXERCISE &&
        gui_button != EXIT)
    {

        std::cout << "[WARNING] Enable Exo first!\n";
        gui_button = DEFAULT;
    }

    if (stop_pressed)
    {
        if (!stop_initialized)
        {
            // Reset delle variabili di movimento
            command_to_exo = false;
            tick = 0;

            Ally.reset();

            rest_timer.reset();
            stim_timer.reset();
            state_timer.reset();

            motion_sub_state = TO_TORQUE;

            positioning_active = false;
            position_mode_ready = false;
            position_reached = false;
            exercise_active = false;

            // Disabilita ulteriori aggiornamenti FES
            stim_mode = STIM_WAIT;

            // Arresto Mid-Level dello stimolatore
            if (P24_initialized)
            {
                if (!smpt_send_ml_stop(
                        &p24_stimulator.stim_device,
                        p24_stimulator.packet_number++))
                {
                    std::cerr << "[STOP] Error sending FES stop!\n";
                }
            }

            stop_initialized = true;

            std::cout << "[STOP] Stopping Exo and FES...\n";
        }

        // Continua la transizione dei driver finche' non e' completata
        if (set_global_working_mode(driver_stop) && !stop_confirmed)
        {
            stop_confirmed = true;
            std::cout << "[STOP] Exo SOFT_STOP confirmed.\n";
        }

        return;
    }

    switch (gui_button)
    {

    case ENABLE_EXO:

        if (!init_flag)
        {

            std::string filename = "File_Log_" +
                                   std::to_string(std::time(nullptr)) + ".csv";

            logger = std::make_unique<DataLogger>(filename);

            start_time_ms = (long long int)get_current_time_ms();
            init_flag = true;

            std::cout << "[EXO] Enabled\n";
            std::cout << "[LOGGER] Recording to " << filename << "\n";
        }

        break;

    case GO_TO_POSITION:

        if (!positioning_active && !exercise_active && !stop_pressed)
        {
            positioning_active = true;
            position_mode_ready = false;
            position_reached = false;

            command_to_exo = false;
            tick = 0;

            std::cout << "[EXO] Moving to start position\n";
        }

        break;

    case ENABLE_FES:
        // Richiesta abilitazione FES
        break;

    case RECTANGULAR:
        // Seleziona pattern rettangolare
        break;

    case TRAPEZOIDAL:
        // Seleziona pattern trapezoidale
        break;

    case BIOMIMETIC:
        // Seleziona pattern biomimetico
        break;

    case START_EXERCISE:

        if (position_reached &&
            !positioning_active &&
            !exercise_active &&
            !stop_pressed)
        {
            exercise_active = true;
            antig_90s_completed = false;

            motion_sub_state = TO_TORQUE;

            std::cout << "[EXO] Starting AntiG exercise...\n";
        }
        else
        {
            std::cout << "[WARNING] Exo not ready for AntiG!\n";
        }

        break;

    case STOP_EXERCISE:
        stop_requested = true; // Richiede arresto della prova
        break;

    case EXIT:

        init_flag = false;

        if (logger != nullptr)
        {
            logger.reset(); // Call DataLogger destructor
            std::cout << "[LOGGER] CSV closed.\n";
        }

        std::cout << "[SYSTEM] Press Ctrl+C to terminate.\n";
        break;

    case DEFAULT:
        break;
    }

    // if (stim_flag)
    // {
    //     p24_stimulator.stimulation_calibration(&p24_stimulator.stim_device, g_stim_channel, g_stim_current, 1000 / g_stim_frequency, g_stim_pulse_width);
    //     stim_flag = false;
    // }
    // else
    // {

    //     // p24_stimulator.stimulation_calibration(&p24_stimulator.stim_device, 0, 0, 1000 / 30, 400);
    //     // smpt_send_ml_stop(&p24_stimulator.stim_device, p24_stimulator.packet_number);
    // }

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

    /*-------- POSITION CONTROL -----------*/

    if (positioning_active && !stop_pressed)
    {
        if (!position_mode_ready)
        {
            if (set_global_working_mode(driver_position))
            {
                position_mode_ready = true;

                std::cout << "[EXO] Position mode enabled\n";
            }
        }
        else
        {
            if (Ally.robotControl_Position(target_pos, 3000)) // movimento di setup dura 3 secondi
            {
                positioning_active = false;
                position_reached = true;
                state_timer.reset();

                std::cout << "[EXO] Target position reached!\n";
            }
        }
    }

    /*---------ANTIG CONTROL-------------*/

    if (exercise_active && !stop_pressed)
    {
        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_AntiG(torque_axis, molla, 600000)) // qui si può sutomizzare la durata dell'esercizio. ripetizioni magari?
                motion_sub_state = TO_POSITION;
            break;

        case TO_POSITION:
            if (set_global_working_mode(driver_position))
                motion_sub_state = SUBTASK_COMPLETE;
            break;

        case SUBTASK_COMPLETE:
            exercise_active = false;
            antig_90s_completed = true;
            motion_sub_state = TO_TORQUE;
            break;

        default:
            break;
        }
    }

    // // ── Controllo stato stimolatore ──────────────
    // switch (stim_mode)
    // {
    // case STIM_WAIT:
    //     // inizializza correnti, resettare timer
    //     for (int i = 0; i < NUM_MUSCLES; i++)
    //     {
    //         stim_current[i] = 0;
    //         G[i] = 0.6;
    //     }
    //     exercise_t = 0;

    //     break;

    // case STIM_IDLE:

    //     for (int i = 0; i < NUM_MUSCLES; i++)
    //     {
    //         current[i] = 0;
    //     }

    //     stimulation = p24_stimulator.stimulate(&p24_stimulator.stim_device, current, 1000 / (int)g_start_fes_calibration[0].frequency, (int)g_start_fes_calibration[0].pulse_width);
    //     break;

    // case STIM_ON:

    //     for (int i = 0; i < NUM_MUSCLES; i++)
    //     {
    //         current[i] = stim_current[i];
    //     }

    //     stimulation = p24_stimulator.stimulate(&p24_stimulator.stim_device, current, 1000 / (int)g_start_fes_calibration[0].frequency, (int)g_start_fes_calibration[0].pulse_width);
    //     printf("Stim update sent: %d\n", stimulation);
    //     break;

    // case STIM_OFF:

    //     smpt_send_ml_stop(&p24_stimulator.stim_device, p24_stimulator.packet_number);
    //     smpt_close_serial_port(&p24_stimulator.stim_device);
    //     exit(0);
    //     break;

    // case STIM_ERROR:
    //     stim_mode = STIM_WAIT;
    //     break;
    // }

    if (init_flag && logger != nullptr)
    {
        log_data.timestamp =
            (long long int)get_current_time_ms() - start_time_ms;

        for (int i = 0; i < 4; i++)
        {
            log_data.working_mode[i] =
                polimi_axis_data[i].working_mode;

            log_data.position_deg[i] =
                polimi_axis_data[i].position_deg;

            log_data.speed_deg_per_sec[i] =
                polimi_axis_data[i].speed_deg_per_sec;

            log_data.torque_Nm[i] =
                polimi_axis_data[i].torque_Nm;
        }

        // FES non ancora implementata
        for (int i = 0; i < 5; i++)
        {
            log_data.stim_current[i] = 0.0;
        }

        if (!logger->write(log_data))
        {
            std::cerr << "[ERROR] Logger file not available\n";
            init_flag = false;
        }
    }
}

// void generate_beta_patterns_for_selected_exercise(int mode, int duration_phase_ms) // forse devo mandargli anche il tempo "duration_phase_ms"
// {
//     // ── Ottieni i parametri beta per l'esercizio selezionato ──
//     BetaParams p1 = get_beta_params_for_exercise(mode);

//     // Qui puoi impostare una durata in ms basata su quanto vuoi duri la fase 1
//     p1.t_end = duration_phase_ms; // esempio: primo valore dei tempi di fase

//     // Genera il pattern beta per la prima fase
//     auto pattern1 = generate_beta_pattern_go_back(p1.t_end, p1, Imin, Imax);

//     beta_patterns[mode] = pattern1;
// }

void print_menu()
{

    std::cout << "\n========== EXO-FES WORKSHOP ==========\n";

    std::cout << "1 - ENABLE EXO\n";
    std::cout << "2 - GO TO POSITION\n";
    std::cout << "3 - ENABLE FES\n";

    std::cout << "\n--- Stimulation Pattern ---\n";
    std::cout << "4 - RECTANGULAR\n";
    std::cout << "5 - TRAPEZOIDAL\n";
    std::cout << "6 - BIOMIMETIC\n";

    std::cout << "\n--- Exercise ---\n";
    std::cout << "7 - START EXERCISE\n";
    std::cout << "8 - STOP EXERCISE\n";

    std::cout << "\n--- Safe Exit ---\n";
    std::cout << "\n9 - EXIT\n";

    std::cout << "======================================\n";
}

void terminal_input()
{

    std::string input;
    int selection;

    while (true)
    {

        std::cout << "\nSelect command: " << std::flush;

        if (!std::getline(std::cin, input))
        {
            break;
        }

        std::stringstream ss(input);
        char extra;

        if (!(ss >> selection) || (ss >> extra))
        {
            std::cout << "Invalid input. Enter a number.\n";
            continue;
        }

        switch (selection)
        {

        case ENABLE_EXO:
        case GO_TO_POSITION:
        case ENABLE_FES:
        case RECTANGULAR:
        case TRAPEZOIDAL:
        case BIOMIMETIC:
        case START_EXERCISE:

            requested_button.store(
                static_cast<GUI_State>(selection));
            break;

        case STOP_EXERCISE:

            stop_requested = true;
            break;

        case EXIT:

            if (!stop_confirmed.load())
            {
                std::cout << "[WARNING] Stop Exo first (command 8).\n";
            }
            else
            {
                if (logger != nullptr)
                {
                    logger.reset();
                    std::cout << "[LOGGER] CSV closed.\n";
                }

                std::cout << "[SYSTEM] Press Ctrl+C to terminate.\n";
            }

            break;

        default:
            std::cout << "Unknown command.\n";
            break;
        }
    }
}

int main(void)
{
    const char *port_name = "/dev/P24";
    p24_stimulator.stim_device = {0};
    smpt_open_serial_port(&p24_stimulator.stim_device, port_name);

    P24_initialized = p24_stimulator.init_stimulation(&p24_stimulator.stim_device);

    printf("P24 initialized: %s\n", P24_initialized ? "true" : "false");

    start_time_ms = (long long int)get_current_time_ms();

    Ally.reset();

    std::cout << "\n=== Init Control Params ===\n";
    exo_config = exofes::config::load_configuration("./exo_params.json");
    std::cout << "=========================================\n\n";

    // signal(SIGINT, handle_sigint);

    if (init(loop) != 0)
        return 1;

    printf("Shared memory reader started...\n");

    molla = get_molla_parameters();

    printf("Rigidezza: %f, Offset: %f, Mano: %d\n", molla.k, molla.offset, molla.hand);

    set_global_working_mode(driver_stop);

    print_menu();

    terminal_input();

    return 0;
}
