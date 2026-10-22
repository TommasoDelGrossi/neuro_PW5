#include "workshop_runtime.hpp"
#include "fes_patterns.hpp"

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
#include <cerrno>
#include <mutex>

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
#include "smpt_messages.h"

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
bool P24_initialized = false; // Successful ML_Init send, not a device acknowledgement
bool P24_port_open = false;   // Owns a descriptor; does not prove USB is still connected
bool fes_reopen_before_stop = false; // Discard a failed transport before retrying STOP

Smpt_device stim_device;
Smpt_ml_init ml_init;
Smpt_ml_update ml_update;
uint8_t packet_number = 0;

double current[NUM_MUSCLES] = {0};
// Original Codice_ok(2).zip: calculated stimulation commands, copied into current[] at each control tick.
double stim_current[NUM_MUSCLES] = {0};
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

float target_pos[4] = {90.0f, 0.0f, 90.0f, 90.0f};  // set from WorkshopSettings at startup

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
bool fes_stop_ok = true;

// EXIT is acknowledged by loop() before the terminal destroys the logger.
// Once requested, session closure cannot be cancelled by a restart command.
bool session_closing = false; // Only accessed by loop()
std::atomic<bool> logger_close_ready{false};

bool positioning_active = false;
bool position_mode_ready = false;
bool position_reached = false;

bool exercise_active = false;
bool antig_90s_completed = false;

/* ── FES runtime state ─────────────────────────────────── */
// Independent snapshot for the ongoing trial; never loaded from disk in loop().
std::vector<BiomimeticPoint> active_biomimetic_profile;
FesPatternType active_fes_pattern = FesPatternType::NONE;
FesPatternConfig active_fes_config;
std::atomic<bool> fes_pattern_running{false};
bool fes_phase_started = false; // Start the FES clock only once AntiG is active.
FesPatternPhase fes_phase = FesPatternPhase::IDLE;
int fes_rep = 0;

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
Timer fes_phase_timer;
Timer p24_keepalive_timer;

long long int start_time_ms;

bool init_flag = false;

int counter = 0;

std::unique_ptr<DataLogger> logger;
LogData log_data;

std::atomic<GUI_State> requested_button{DEFAULT};
std::atomic<bool> stop_requested{false};

GUI_State gui_button = DEFAULT;

// Diagnostic output only while sending ML_Init/ML_Stop. No serial polling is added.
void report_fes_error(const char *message)
{
    const int saved_errno = errno;
    std::cerr << "[FES / SMPT] " << message << "\n";
    errno = saved_errno;
}

void ignore_fes_diagnostic(const char *)
{
}

// Start with a fresh transport. This function sends no protocol commands.
// Used before every new ML_Init and when recovering a failed STOP transport.
// ============================================================================
// BOX 1 - P24 SERIAL LIFECYCLE (DO NOT MODIFY IN STUDENT EXERCISES)
// ============================================================================
bool reopen_p24_port()
{
    if (P24_port_open)
    {
        errno = 0;
        const bool closed = smpt_close_serial_port(&p24_stimulator.stim_device);
        const int close_errno = errno;
        // Linux releases the descriptor even on a close error. Do not close it twice.
        P24_port_open = false;
        if (!closed)
        {
            stim_mode = STIM_ERROR;
            std::cerr << "[FES] Serial close failed, errno=" << close_errno
                      << ". Retry the requested command.\n";
            return false;
        }
    }

    p24_stimulator.stim_device = {0};
    errno = 0;
    P24_port_open = smpt_open_serial_port(
        &p24_stimulator.stim_device, "/dev/P24");
    const int open_errno = errno;

    if (!P24_port_open)
    {
        stim_mode = STIM_ERROR;
        std::cerr << "[FES] Cannot open /dev/P24, errno=" << open_errno
                  << ". Check connection, then retry the requested command.\n";
        return false;
    }

    fes_reopen_before_stop = false;
    // Keep the protocol flag: a pending STOP is not completed by reopening.
    return true;
}


// ML_Update is dispatched cyclically below, using the same STIM_ON / STIM_IDLE
// logic and ml_stimulator::stimulate() call as the original working program.
// In the original, STIM_ON copies stim_current[] to current[]; STIM_IDLE sends
// all-zero currents. The rectangular FSM only determines which state applies.

// Mid-Level requires ML_Get_Current_Data at least every 2 s after ML_Update.
// We send it every 1 s while the rectangular sequence is active.
bool send_p24_keepalive()
{
    Smpt_ml_get_current_data request{};
    smpt_clear_ml_get_current_data(&request);
    p24_stimulator.packet_number %= 64;
    request.packet_number = p24_stimulator.packet_number++;
    request.data_selection = Smpt_Ml_Data_Stimulation_Eight_Channels;

    if (!smpt_send_ml_get_current_data(&p24_stimulator.stim_device, &request))
    {
        std::cerr << "[FES] ML_Get_Current_Data keep-alive failed. Requesting STOP.\n";
        stim_mode = STIM_ERROR;
        stop_requested.store(true);
        return false;
    }
    return true;
}

// ============================================================================
// BOX 2 - REAL-TIME CALLBACK: COMMANDS -> EXO -> FES -> LOGGER
// ============================================================================
void loop()
{
    update_polimi_axis_data();

    GUI_State gui_button = requested_button.exchange(DEFAULT);

    if (stop_requested.exchange(false))
    {
        // A new STOP always wins over a pending movement command.
        if (stop_pressed && !fes_stop_ok)
            stop_initialized = false; // Explicit retry after an ML Stop send error

        stop_pressed = true;
        stop_confirmed.store(false);
        if (gui_button != EXIT)
            gui_button = DEFAULT;
    }

    if (gui_button == EXIT)
    {
        session_closing = true;
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

    // --- BOX 2A: safety STOP and controlled restart ------------------------
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
            antig_90s_completed = false;
            fes_pattern_running = false;
            fes_phase_started = false;
            fes_phase = FesPatternPhase::IDLE;
            active_fes_pattern = FesPatternType::NONE;
            active_biomimetic_profile.clear();
            for (double &c : stim_current)
                c = 0.0;
            fes_rep = 0;
            fes_phase_timer.reset();
            p24_keepalive_timer.reset();

            // Disabilita ulteriori aggiornamenti FES
            stim_mode = STIM_OFF;

            // ML Stop deinitializes stimulation; do not reinitialize it on Exo restart.
            // This checks sending, not acknowledgement by the P24.
            fes_stop_ok = !P24_initialized;
            // At most one immediate reconnect/retry for a stale USB descriptor.
            for (int attempt = 0; P24_initialized && attempt < 2; ++attempt)
            {
                if (!P24_port_open || fes_reopen_before_stop)
                {
                    std::cout << "[FES STOP] Reopening /dev/P24 for STOP only; no ML_Init.\n";
                    if (!reopen_p24_port())
                        break; // STOP remains pending; another command 8 retries.
                }

                p24_stimulator.packet_number %= 64;
                const uint8_t stop_packet = p24_stimulator.packet_number++;

                smpt_init_error_callback(report_fes_error);
                errno = 0;
                fes_stop_ok = smpt_send_ml_stop(
                    &p24_stimulator.stim_device, stop_packet);
                const int stop_errno = errno;
                // nullptr does not unregister callbacks in SMPT; use a no-op.
                smpt_init_error_callback(ignore_fes_diagnostic);

                if (fes_stop_ok)
                {
                    P24_initialized = false;
                    stim_mode = STIM_OFF;
                    std::cout << "[FES] ML_Stop sent. Use 3 to initialize FES again.\n";
                }
                else
                {
                    stim_mode = STIM_ERROR;
                    std::cerr << "[FES STOP] packet=" << static_cast<unsigned>(stop_packet)
                              << ", errno=" << stop_errno;
                    if (stop_errno != 0)
                        std::cerr << " (" << std::strerror(stop_errno) << ")";
                    std::cerr << "\n";
                    if (stop_errno == EIO || stop_errno == ENODEV)
                    {
                        fes_reopen_before_stop = true;
                        if (attempt == 0)
                        {
                            std::cerr << "[FES STOP] Transport error. Reconnecting and retrying STOP once.\n";
                            continue;
                        }
                    }
                    std::cerr << "[STOP] FES stop send failed. Verify P24; command 8 retries.\n";
                }
                break;
            }
            for (double &c : current)
                c = 0.0;

            stop_initialized = true;

            std::cout << "[STOP] Stopping Exo and FES...\n";
        }

        // Complete the existing driver transition before accepting a restart.
        if (!stop_confirmed.load())
        {
            bool exo_stopped = set_global_working_mode(driver_stop);
            if (exo_stopped && fes_stop_ok)
            {
                stop_confirmed.store(true);
                std::cout << "[STOP] Exo SOFT_STOP confirmed; FES stop send handled.\n";
                if (init_flag)
                    std::cout << ">>> 2: reposition; 3: initialize FES; 9: close log (each + ENTER).\n";
                else
                    std::cout << ">>> 1 + ENTER: enable session; 9 + ENTER: close session.\n";
            }
        }

        if (session_closing)
        {
            if (stop_confirmed.load())
            {
                init_flag = false;
                // No later callback can reach logger or the control blocks.
                logger_close_ready.store(true);
            }
            return;
        }

        const bool restart_position = (gui_button == GO_TO_POSITION && init_flag);
        const bool enable_new_session = (gui_button == ENABLE_EXO && !init_flag);
        const bool reinitialize_fes = (gui_button == ENABLE_FES && init_flag);

        if (stop_confirmed.load() &&
            (restart_position || enable_new_session || reinitialize_fes))
        {
            // Explicit restart: use the existing command cases below.
            stop_confirmed.store(false);
            stop_pressed = false;
            stop_initialized = false;
            if (restart_position)
                std::cout << "[EXO] Restart requested. Repositioning before AntiG.\n";
            else if (reinitialize_fes)
                std::cout << "[FES] Reinitialization requested; Exo remains in SOFT_STOP.\n";
        }
        else
        {
            if (gui_button == GO_TO_POSITION || gui_button == START_EXERCISE ||
                gui_button == ENABLE_FES)
            {
                if (!stop_confirmed.load())
                    std::cout << "[WARNING] Wait for STOP confirmation.\n";
                else if (!init_flag)
                    std::cout << "[WARNING] No active log session. Restart the program.\n";
                else
                    std::cout << "[WARNING] After STOP, use 2 before 7.\n";
            }
            return;
        }
    }

    // --- BOX 2B: terminal commands ---------------------------------------
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
            Ally.reset();
            state_timer.reset();

            std::cout << "[EXO] Moving to start position\n";
        }

        break;

    case ENABLE_FES:
    {

        if (P24_initialized)
        {
            std::cout << "[WARNING] FES already initialized. Command 3 ignored.\n";
            break;
        }

        if (positioning_active || exercise_active)
        {
            std::cout << "[WARNING] Initialize FES before starting movement.\n";
            break;
        }

        // Repeat the known-good clear -> open -> init_stimulation sequence.
        // An old descriptor may refer to a deleted tty even if its flag is true.
        if (!reopen_p24_port())
            break;

        p24_stimulator.packet_number %= 64;
        smpt_init_error_callback(report_fes_error);
        errno = 0;
        P24_initialized = p24_stimulator.init_stimulation(
            &p24_stimulator.stim_device);
        const int init_errno = errno;
        smpt_init_error_callback(ignore_fes_diagnostic);

        if (P24_initialized)
        {
            for (double &c : current)
                c = 0.0;
            stim_mode = STIM_IDLE;
            fes_stop_ok = false; // This new ML session needs a new STOP.
            std::cout << "[FES] ML_Init sent. No stimulation requested.\n";
        }
        else
        {
            stim_mode = STIM_ERROR;
            std::cerr << "[FES] ML_Init send failed, errno=" << init_errno
                      << ". Check P24, then retry 3 (fresh connection).\n";
        }

        printf("P24 initialized: %s\n", P24_initialized ? "true" : "false");
        break;
    }

    case RECTANGULAR:
    case TRAPEZOIDAL:
    case BIOMIMETIC:
        // Configuration is collected by terminal_input(), before command 7.
        break;

    case START_EXERCISE:
        // AntiG is independent of FES: command 7 must work with NO pattern
        // configured and/or with the P24 not initialized.
        if (position_reached &&
            !positioning_active &&
            !exercise_active &&
            !stop_pressed)
        {
            exercise_active = true;
            antig_90s_completed = false;

            motion_sub_state = TO_TORQUE;
            Ally.reset();

            active_fes_pattern = FesPatternType::NONE;
            const FesPatternType requested_pattern = selected_fes_pattern.load();
            if (requested_pattern != FesPatternType::NONE && P24_initialized)
            {
                active_fes_pattern = requested_pattern;
                if (requested_pattern == FesPatternType::RECTANGULAR)
                    active_fes_config = rectangular_config;
                else if (requested_pattern == FesPatternType::TRAPEZOIDAL)
                    active_fes_config = trapezoidal_config;
                else if (requested_pattern == FesPatternType::BIOMIMETIC)
                {
                    // Snapshot once before the trial, never in the 100-Hz loop.
                    std::lock_guard<std::mutex> guard(biomimetic_config_mutex);
                    active_fes_config = biomimetic_config;
                    active_biomimetic_profile = biomimetic_profile;
                }
            }
            else if (requested_pattern != FesPatternType::NONE)
            {
                std::cout << "[WARNING] P24 not initialized; starting AntiG WITHOUT FES.\n";
            }

            fes_pattern_running = (active_fes_pattern != FesPatternType::NONE);
            fes_phase_started = false;
            fes_phase = fes_pattern_running ? FesPatternPhase::ON : FesPatternPhase::IDLE;
            fes_rep = fes_pattern_running ? 1 : 0;
            fes_phase_timer.reset();
            p24_keepalive_timer.reset();
            for (double &c : stim_current) c = 0.0;
            for (double &c : current) c = 0.0;

            // Original execution order: enter AntiG before cyclic ML_Updates.
            stim_mode = STIM_WAIT;
            std::cout << "[EXO] Starting AntiG exercise...";
            if (!fes_pattern_running)
                std::cout << " (without FES)";
            std::cout << "\n";
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
        // Handled by the latched STOP/closure branch before this switch.
        break;

    case DEFAULT:
        break;
    }

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

    // --- BOX 2C: approach initial position -------------------------------

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
            if (Ally.robotControl_Position(target_pos, workshop_settings.position_move_ms)) // movimento di setup dura 3 secondi
            {
                positioning_active = false;
                position_reached = true;
                state_timer.reset();

                std::cout << "[EXO] Target position reached!\n";
            }
        }
    }

    // --- BOX 2D: robot gravity compensation ------------------------------

    if (exercise_active && !stop_pressed)
    {
        switch (motion_sub_state)
        {
        case TO_TORQUE:
            if (set_global_working_mode(driver_torque))
                motion_sub_state = IMPEDANCE_CONTROL;
            break;

        case IMPEDANCE_CONTROL:
            if (Ally.robotControl_AntiG(torque_axis, molla, workshop_settings.antig_duration_ms)) // qui si può sutomizzare la durata dell'esercizio. ripetizioni magari?
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

    // --- BOX 2E: FES timing and original cyclic ML_Update ----------------
    // The pattern computes the current envelope; the original stimulate()
    // wrapper constructs and transmits the same three-point biphasic pulse.
    if (fes_pattern_running && !stop_pressed)
    {
        if (motion_sub_state == IMPEDANCE_CONTROL)
        {
            if (!fes_phase_started)
            {
                fes_phase_started = true;
                fes_phase = FesPatternPhase::ON;
                fes_phase_timer.start();
                p24_keepalive_timer.start();
                std::cout << "[FES] " << fes_pattern_name(active_fes_pattern)
                          << " pattern started: repetition 1/"
                          << active_fes_config.repetitions << " (ON).\n";
            }

            // Read the elapsed time BEFORE checking the transition. The same
            // 100-Hz control callback drives both the timing and the envelope.
            const long long phase_elapsed_ms = fes_phase_timer.get_elapsed_time_ms();
            if (fes_phase == FesPatternPhase::ON &&
                phase_elapsed_ms >= active_fes_config.on_ms)
            {
                fes_phase = FesPatternPhase::OFF;
                fes_phase_timer.reset();
                fes_phase_timer.start();
                stim_mode = STIM_IDLE;
                std::cout << "[FES] Repetition " << fes_rep << "/"
                          << active_fes_config.repetitions << " -> OFF.\n";
            }
            else if (fes_phase == FesPatternPhase::OFF &&
                     phase_elapsed_ms >= active_fes_config.off_ms)
            {
                if (fes_rep >= active_fes_config.repetitions)
                {
                    fes_pattern_running = false;
                    fes_phase_started = false;
                    fes_phase = FesPatternPhase::COMPLETE;
                    fes_phase_timer.reset();
                    p24_keepalive_timer.reset();
                    stim_mode = STIM_WAIT;
                    for (double &c : current) c = 0.0;
                    for (double &c : stim_current) c = 0.0;
                    std::cout << "[FES] " << fes_pattern_name(active_fes_pattern)
                              << " pattern completed. FES remains initialized; AntiG continues.\n";
                    active_fes_pattern = FesPatternType::NONE;
                    active_biomimetic_profile.clear();
                }
                else
                {
                    ++fes_rep;
                    fes_phase = FesPatternPhase::ON;
                    fes_phase_timer.reset();
                    fes_phase_timer.start();
                    std::cout << "[FES] Repetition " << fes_rep << "/"
                              << active_fes_config.repetitions << " -> ON.\n";
                }
            }

            if (fes_pattern_running)
            {
                for (double &c : stim_current) c = 0.0;
                for (double &c : current) c = 0.0;

                if (fes_phase == FesPatternPhase::ON)
                {
                    const long long t_ms = fes_phase_timer.get_elapsed_time_ms();
                    const double command_mA = current_during_fes_on(
                        active_fes_config, active_fes_pattern, t_ms, active_biomimetic_profile);
                    stim_current[active_fes_config.channel - 1] = command_mA;
                    current[active_fes_config.channel - 1] = command_mA;
                    stim_mode = STIM_ON;
                }
                else
                {
                    stim_mode = STIM_IDLE; // disable all channels via ML_Update
                }

                const int period_ms = std::max(1, static_cast<int>(
                    std::lround(1000.0 / active_fes_config.frequency_Hz)));
                p24_stimulator.packet_number %= 64;
                errno = 0;
                const bool sent = p24_stimulator.stimulate(
                    &p24_stimulator.stim_device, current,
                    period_ms, active_fes_config.pulse_width_us);
                const int update_errno = errno;

                if (!sent)
                {
                    std::cerr << "[FES] Cyclic ML_Update failed (errno=" << update_errno;
                    if (update_errno != 0)
                        std::cerr << ", " << std::strerror(update_errno);
                    std::cerr << "). Requesting STOP.\n";
                    stim_mode = STIM_ERROR;
                    fes_pattern_running = false;
                    stop_requested.store(true);
                }
                else if (p24_keepalive_timer.has_elapsed(1000))
                {
                    send_p24_keepalive();
                }
            }
        }
        else if (fes_phase_started)
        {
            std::cerr << "[FES] AntiG is no longer active. Requesting STOP.\n";
            stop_requested.store(true);
        }
    }

    // --- BOX 2F: CSV telemetry -------------------------------------------
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

        for (int i = 0; i < 5; i++)
            log_data.stim_current[i] = current[i];

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


WorkshopSettings workshop_settings;

// ============================================================================
// BOX 4 - STARTUP: preserved P24 -> Exo initialization order
// ============================================================================
int run_workshop(const WorkshopSettings &settings)
{
    // Immutable configuration after threads are started.
    workshop_settings = settings;
    for (int joint = 0; joint < 4; ++joint)
        target_pos[joint] = workshop_settings.start_position_deg[joint];

    // Restore the working initialization order from Codice_ok(2).zip:
    // open /dev/P24 -> ML_Init -> start the Exo threads.
    const char *port_name = "/dev/P24";
    p24_stimulator.stim_device = {0};
    P24_port_open = smpt_open_serial_port(&p24_stimulator.stim_device, port_name);

    if (P24_port_open)
    {
        P24_initialized = p24_stimulator.init_stimulation(&p24_stimulator.stim_device);
        fes_stop_ok = !P24_initialized;
        // The original starts in STIM_WAIT: no ML_Update before an exercise.
        stim_mode = STIM_WAIT;
        printf("P24 initialized: %s\n", P24_initialized ? "true" : "false");
    }
    else
    {
        std::cerr << "[FES] Cannot open /dev/P24 at startup. Command 3 retries.\n";
    }

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

    if (molla.hand == 1)
        target_pos[2] = -90.0f; // mano sinistra, pollice a 0°

    printf("Rigidezza: %f, Offset: %f, Mano: %d\n", molla.k, molla.offset, molla.hand);

    set_global_working_mode(driver_stop);

    print_menu();

    terminal_input();

    return 0;
}
