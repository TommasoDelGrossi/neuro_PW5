#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
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

// =================== P24 ===================
ml_stimulator p24_stimulator;
bool P24_initialized = false;
bool P24_port_open = false;

enum stimulator_mode_t { STIM_WAIT, STIM_IDLE, STIM_ON, STIM_OFF, STIM_ERROR };
stimulator_mode_t stim_mode = STIM_WAIT;         // Requested state
stimulator_mode_t stim_mode_applied = STIM_WAIT; // Last successfully sent state
double current[5] = {0, 0, 0, 0, 0};

// Usare il canale corrispondente al muscolo stimolato
constexpr int FES_CHANNEL = 0;

// Valori iniziali software, NON limiti clinici certificati.
// La corrente massima 20 mA riprende Imax nel vecchio main.
constexpr double MAX_FES_CURRENT_MA = 20.0;
constexpr double MAX_FES_FREQUENCY_HZ = 60.0;
constexpr int MAX_FES_PULSE_WIDTH_US = 500;
constexpr int MAX_MOVEMENT_MS = 5000;
constexpr int MAX_REPETITIONS = 25;

struct RectangularParams {
    double frequency_hz = 0.0;
    int pulse_width_us = 0;
    double current_mA = 0.0;
    int period_ms = 0;
    int movement_ms = 0;
    int repetitions = 0;
};

RectangularParams pending_rect; // Terminal thread (protetto da mutex)
RectangularParams rect_params;   // Callback thread
std::mutex rect_mutex;
bool rectangular_ready = false;
bool fes_enabled = false;
bool rectangular_exercise = false;
int repetitions_done = 0;

// =================== EXO ===================
exofes::control::ExoController Ally;
exofes::config::RobotConfig exo_config;
molla_parameters_t molla{};

float target_pos[4] = {90.0f, 0.0f, 90.0f, 90.0f};
bool torque_axis[4] = {false, false, false, true};

driver_working_mode_enum driver_position[4] = {
    POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER, POS_INT_PROFILER
};

driver_working_mode_enum driver_torque[4] = {
    STAND_STILL_AUTO, STAND_STILL_AUTO, STAND_STILL_AUTO, TORQUE
};

driver_working_mode_enum driver_stop[4] = {
    SOFT_STOP, SOFT_STOP, SOFT_STOP, SOFT_STOP
};

bool command_to_exo = false; // Richiesto da utils_exofes.cpp
int tick = 0;

bool positioning_active = false;
bool position_mode_ready = false;
bool position_reached = false;
bool exercise_active = false;
bool antig_90s_completed = false;

bool stop_pressed = false;
bool stop_initialized = false;
bool fes_stop_sent = false;
std::atomic<bool> stop_confirmed{false};

constexpr int POSITION_DURATION_MS = 3000;
constexpr int RETURN_DURATION_MS = 3000;
constexpr int FES_PAUSE_MS = 1000;

enum MotionSubState {
    TO_TORQUE,
    IMPEDANCE_CONTROL,
    FES_PAUSE,
    TO_POSITION,
    POSITION_CORRECTION,
    SUBTASK_COMPLETE
};
MotionSubState motion_sub_state = TO_TORQUE;

Timer rest_timer;
Timer state_timer;

// =================== LOGGER ===================
std::unique_ptr<DataLogger> logger;
LogData log_data{};
std::atomic<bool> init_flag{false};
long long int start_time_ms = 0;

long long int get_current_time_ms() {
    timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<long long int>(ts.tv_sec) * 1000LL + ts.tv_nsec / 1000000LL;
}

// =================== STIMULATOR FSM ===================
// Only this function sends ML updates. The exercise FSM requests ON/IDLE.
// A successful send is NOT proof that the P24 has applied the command.
bool update_stimulator()
{
    if (stim_mode == stim_mode_applied) return true;

    switch (stim_mode)
    {
    case STIM_WAIT:
        // Initial state only. Use STIM_IDLE to disable output between repeats.
        for (double &c : current) c = 0.0;
        stim_mode_applied = STIM_WAIT;
        return true;

    case STIM_IDLE:
    case STIM_ON:
        if (!P24_initialized)
        {
            std::cerr << "[FES ERROR] P24 ML is not initialized.\n";
            stim_mode = STIM_ERROR;
            stop_pressed = true;
            return false;
        }

        for (double &c : current) c = 0.0;
        if (stim_mode == STIM_ON)
        {
            if (!fes_enabled || !rectangular_ready)
            {
                std::cerr << "[FES ERROR] Stimulation not armed.\n";
                stim_mode = STIM_ERROR;
                stop_pressed = true;
                return false;
            }
            current[FES_CHANNEL] = rect_params.current_mA;
        }

        if (!p24_stimulator.stimulate(
                &p24_stimulator.stim_device,
                current, rect_params.period_ms, rect_params.pulse_width_us))
        {
            std::cerr << "[FES ERROR] ML Update send failed.\n";
            stim_mode = STIM_ERROR;
            stop_pressed = true;
            return false;
        }

        stim_mode_applied = stim_mode;
        return true;

    case STIM_OFF:
        for (double &c : current) c = 0.0;

        if (!P24_initialized)
        {
            fes_stop_sent = true;
            stim_mode_applied = STIM_OFF;
            return true;
        }

        fes_stop_sent = smpt_send_ml_stop(
            &p24_stimulator.stim_device, p24_stimulator.packet_number++);

        if (!fes_stop_sent)
        {
            std::cerr << "[FES ERROR] ML Stop send failed; retry pending.\n";
            return false;
        }

        stim_mode_applied = STIM_OFF;
        return true;

    case STIM_ERROR:
        // Do not attempt more ML updates; STOP will request STIM_OFF.
        stop_pressed = true;
        return false;
    }
    return false;
}

// =================== TERMINAL ===================
enum GUI_State {
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
};
std::atomic<GUI_State> requested_button{DEFAULT};
std::atomic<bool> stop_requested{false};

void print_menu() {
    std::cout << "\n========== EXO-FES WORKSHOP ==========\n"
              << "1 - ENABLE EXO / START LOG\n"
              << "2 - GO TO START POSITION\n"
              << "3 - ENABLE FES\n"
              << "4 - CONFIGURE RECTANGULAR FES\n"
              << "5 - TRAPEZOIDAL (not implemented)\n"
              << "6 - BIOMIMETIC (not implemented)\n"
              << "7 - START EXERCISE\n"
              << "8 - STOP EXERCISE\n"
              << "9 - EXIT / CLOSE LOG (after STOP)\n"
              << "SPACE + ENTER - SHOW COMMANDS\n"
              << "======================================\n";
}

template <typename T>
bool read_parameter(const std::string &prompt, T &value) {
    std::string line;
    std::cout << prompt << std::flush;
    if (!std::getline(std::cin, line)) return false;
    if (line == "STOP" || line == "stop") {
        stop_requested.store(true);
        return false;
    }
    std::stringstream ss(line);
    std::string extra;
    return static_cast<bool>(ss >> value) && !(ss >> extra);
}

bool configure_rectangle() {
    RectangularParams cfg;
    double duration_s = 0.0;
    if (!read_parameter("Frequency [Hz]: ", cfg.frequency_hz) ||
        !read_parameter("Pulse-width [us]: ", cfg.pulse_width_us) ||
        !read_parameter("Current [mA]: ", cfg.current_mA) ||
        !read_parameter("Movement duration [s]: ", duration_s) ||
        !read_parameter("Repetitions: ", cfg.repetitions)) {
        std::cout << "[WARNING] Configuration cancelled or invalid.\n";
        return false;
    }

    if (!std::isfinite(cfg.frequency_hz) || !std::isfinite(cfg.current_mA) ||
        !std::isfinite(duration_s) ||
        cfg.frequency_hz < 1.0 || cfg.frequency_hz > MAX_FES_FREQUENCY_HZ ||
        cfg.pulse_width_us < 1 || cfg.pulse_width_us > MAX_FES_PULSE_WIDTH_US ||
        cfg.current_mA <= 0.0 || cfg.current_mA > MAX_FES_CURRENT_MA ||
        duration_s <= 0.0 || duration_s * 1000.0 > MAX_MOVEMENT_MS ||
        cfg.repetitions < 1 || cfg.repetitions > MAX_REPETITIONS) {
        std::cout << "[WARNING] Parameters outside configured limits.\n";
        return false;
    }

    cfg.period_ms = static_cast<int>(std::lround(1000.0 / cfg.frequency_hz));
    cfg.movement_ms = static_cast<int>(std::lround(duration_s * 1000.0));
    if (cfg.movement_ms < 1 || cfg.period_ms < 1 ||
        (2 * cfg.pulse_width_us + 100) >= 1000 * cfg.period_ms) {
        std::cout << "[WARNING] Invalid pulse timing.\n";
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(rect_mutex);
        pending_rect = cfg;
    }
    requested_button.store(RECTANGULAR);
    std::cout << "[INFO] Rectangular configuration submitted (actual frequency: "
              << 1000.0 / cfg.period_ms << " Hz).\n";
    return true;
}

void terminal_input() {
    std::string input;
    while (true) {
        if (!std::getline(std::cin, input)) {
            stop_requested.store(true); // EOF: request controlled stop
            return;
        }
        if (!input.empty() && input.find_first_not_of(" \t") == std::string::npos) {
            print_menu();
            continue;
        }

        std::stringstream ss(input);
        int selection = 0;
        std::string extra;
        if (!(ss >> selection) || (ss >> extra)) {
            std::cout << "[WARNING] Invalid command.\n";
            continue;
        }

        switch (selection) {
        case STOP_EXERCISE:
            stop_requested.store(true);
            break;

        case EXIT:
            if (!stop_confirmed.load()) {
                std::cout << "[WARNING] Press 8 and wait for STOP before EXIT.\n";
                break;
            }
            init_flag.store(false);
            if (logger != nullptr) {
                logger.reset(); // Callback gia' bloccato nel ramo STOP
                std::cout << "[LOGGER] CSV closed.\n";
            }
            if (P24_port_open) {
                smpt_close_serial_port(&p24_stimulator.stim_device);
                P24_port_open = false;
                std::cout << "[FES] Serial port closed.\n";
            }
            std::cout << "[SYSTEM] Press Ctrl+C to terminate.\n";
            break;

        case RECTANGULAR:
            configure_rectangle();
            break;

        case ENABLE_EXO:
        case GO_TO_POSITION:
        case ENABLE_FES:
        case TRAPEZOIDAL:
        case BIOMIMETIC:
        case START_EXERCISE:
            requested_button.store(static_cast<GUI_State>(selection));
            break;

        default:
            std::cout << "[WARNING] Unknown command.\n";
            break;
        }
    }
}

// =================== CALLBACK ===================
void loop() {
    update_polimi_axis_data();

    GUI_State gui_button = requested_button.exchange(DEFAULT);
    if (stop_requested.exchange(false)) stop_pressed = true;

    if (stop_pressed) {
        if (!stop_initialized) {
            positioning_active = false;
            position_mode_ready = false;
            position_reached = false;
            exercise_active = false;
            rectangular_exercise = false;
            fes_enabled = false;
            command_to_exo = false;
            tick = 0;
            stim_mode = STIM_OFF;
            for (double &c : current) c = 0.0;
            Ally.reset();
            rest_timer.reset();
            state_timer.reset();

            // ML Stop immediately; if sending fails, retry every 200 ms.
            update_stimulator();
            stop_initialized = true;
            std::cout << "[STOP] Stopping Exo and FES...\n";
        } else if (!fes_stop_sent && rest_timer.has_elapsed(200)) {
            update_stimulator();
        }

        bool exo_stopped = set_global_working_mode(driver_stop);
        if (exo_stopped && fes_stop_sent && !stop_confirmed.load()) {
            stop_confirmed.store(true);
            std::cout << "[STOP] SOFT_STOP and FES stop command sent.\n";
            std::cout << ">>> Press 9 + ENTER to close the CSV.\n";
        }
        return;
    }

    if (!init_flag.load() && gui_button != DEFAULT &&
        gui_button != ENABLE_EXO) {
        std::cout << "[WARNING] Enable Exo first (command 1).\n";
        gui_button = DEFAULT;
    }

    switch (gui_button) {
    case ENABLE_EXO:
        if (!init_flag.load()) {
            std::string filename = "File_Log_" +
                std::to_string(std::time(nullptr)) + ".csv";
            logger = std::make_unique<DataLogger>(filename);
            start_time_ms = get_current_time_ms();
            init_flag.store(true);
            std::cout << "[READY] Exo enabled. CSV: " << filename << "\n";
            std::cout << ">>> Press 2 + ENTER to move to start position.\n";
        }
        break;

    case GO_TO_POSITION:
        if (!positioning_active && !exercise_active) {
            position_reached = false;
            positioning_active = true;
            position_mode_ready = false;
            command_to_exo = false;
            tick = 0;
            Ally.reset();
            state_timer.reset();
            state_timer.start();
            std::cout << "[INFO] Moving to [90, 0, 0, 90]...\n";
        } else {
            std::cout << "[WARNING] Exo already moving.\n";
        }
        break;

    case RECTANGULAR:
        if (!positioning_active && !exercise_active) {
            {
                std::lock_guard<std::mutex> lock(rect_mutex);
                rect_params = pending_rect;
            }
            rectangular_ready = true;
            fes_enabled = false; // nuova configurazione richiede nuova abilitazione
            std::cout << "[READY] Rectangular configured. Press 3 to enable FES.\n";
        } else {
            std::cout << "[WARNING] Cannot configure FES during movement.\n";
        }
        break;

    case ENABLE_FES:
        if (!P24_initialized || !rectangular_ready ||
                   positioning_active || exercise_active) {
            std::cout << "[WARNING] P24 not initialized, pattern missing, or Exo busy.\n";
        } else {
            fes_enabled = true;
            stim_mode = STIM_IDLE;
            std::cout << "[READY] FES armed; no pulses sent yet.\n";
        }
        break;

    case START_EXERCISE:
        if (!position_reached || positioning_active || exercise_active) {
            std::cout << "[WARNING] Reach starting position first.\n";
        } else if (rectangular_ready && (!fes_enabled || !P24_initialized)) {
            std::cout << "[WARNING] Rectangular selected: enable FES first.\n";
        } else {
            exercise_active = true;
            rectangular_exercise = rectangular_ready;
            repetitions_done = 0;
            antig_90s_completed = false;
            motion_sub_state = TO_TORQUE;
            Ally.reset();
            rest_timer.reset();
            state_timer.reset();
            state_timer.start();
            std::cout << "[RUNNING] "
                      << (rectangular_exercise ? "Rectangular FES exercise" : "AntiG only")
                      << " requested.\n";
        }
        break;

    case TRAPEZOIDAL:
    case BIOMIMETIC:
        std::cout << "[WARNING] Pattern not implemented yet.\n";
        break;

    case DEFAULT:
    case STOP_EXERCISE:
    case EXIT:
        break;
    }

    // Logica originale per l'handshake del profilatore di posizione.
    if (master_data.on_target_position && command_to_exo) {
        tick++;
        if (tick > 5) {
            command_to_exo = false;
            tick = 0;
        }
    }
    if (master_data.motion_ongoing == command_to_exo) {
        command_to_exo = false;
    }

    // -------- GO_TO_POSITION iniziale --------
    if (positioning_active && !stop_pressed) {
        if (!position_mode_ready) {
            if (set_global_working_mode(driver_position)) {
                position_mode_ready = true;
                std::cout << "[EXO] Position mode enabled.\n";
            }
        } else if (Ally.robotControl_Position(target_pos, POSITION_DURATION_MS)) {
            position_reached = true;
                std::cout << "[READY] Start position reached. Press 7 for AntiG.\n";
            positioning_active = false;
            state_timer.reset();
        }
    }

    // -------- FSM ESERCIZIO: AntiG o rettangolare --------
    if (exercise_active && !stop_pressed) {
        // Timeout di cambio modo e ritorno. Nessun limite durante AntiG-only.
        switch (motion_sub_state) {
            case TO_TORQUE:
                if (set_global_working_mode(driver_torque)) {
                    Ally.reset();
                    motion_sub_state = IMPEDANCE_CONTROL;
                    state_timer.reset();
                    std::cout << "[RUNNING] AntiG enabled on joint 4.\n";
                    if (!rectangular_exercise)
                        std::cout << ">>> Press 8 + ENTER to STOP.\n";
                }
                break;

            case IMPEDANCE_CONTROL:
                if (!rectangular_exercise) {
                    if (Ally.robotControl_AntiG(torque_axis, molla, 90000) &&
                        !antig_90s_completed) {
                        antig_90s_completed = true;
                        std::cout << "[INFO] 90 seconds elapsed; AntiG continues.\n";
                    }
                    break;
                }

                if (stim_mode != STIM_ON) {
                    stim_mode = STIM_ON; // update_stimulator() sends the ML update
                    std::cout << "[FES] Repetition " << repetitions_done + 1
                              << "/" << rect_params.repetitions << ": ON requested\n";
                }

                if (Ally.robotControl_AntiG(torque_axis, molla,
                                             rect_params.movement_ms)) {
                    stim_mode = STIM_IDLE; // update_stimulator disables channels
                    rest_timer.reset();
                    motion_sub_state = FES_PAUSE;
                    std::cout << "[FES] OFF. Pause 1 second.\n";
                }
                break;

            case FES_PAUSE:
                // Mantieni J4 in AntiG durante la pausa (senza FES).
                Ally.robotControl_AntiG(torque_axis, molla, 600000);
                if (stim_mode_applied == STIM_IDLE &&
                    rest_timer.has_elapsed(FES_PAUSE_MS)) {
                    motion_sub_state = TO_POSITION;
                    state_timer.reset();
                    state_timer.start();
                    std::cout << "[EXO] Returning to start position.\n";
                }
                break;

            case TO_POSITION:
                if (set_global_working_mode(driver_position)) {
                    Ally.reset();
                    command_to_exo = false;
                    tick = 0;
                    motion_sub_state = POSITION_CORRECTION;
                    state_timer.reset();
                    state_timer.start();
                }
                break;

            case POSITION_CORRECTION:
                if (Ally.robotControl_Position(target_pos, RETURN_DURATION_MS)) {
                    
                        motion_sub_state = SUBTASK_COMPLETE;
                    
                }
                break;

            case SUBTASK_COMPLETE:
                ++repetitions_done;
                std::cout << "[EXERCISE] Repetition " << repetitions_done
                          << "/" << rect_params.repetitions << " completed.\n";
                if (repetitions_done < rect_params.repetitions) {
                    Ally.reset();
                    motion_sub_state = TO_TORQUE;
                    state_timer.reset();
                    state_timer.start();
                } else {
                    exercise_active = false;
                    rectangular_exercise = false;
                    fes_enabled = false;
                    position_reached = true;
                    std::cout << "[READY] Exercise completed. Press 8 for STOP.\n";
                }
                break;
            }
        }
    


    // The stimulator FSM is the only sender of ML updates during exercise.
    if (!stop_pressed) update_stimulator();

    if (init_flag.load() && logger != nullptr) {
        log_data.timestamp = get_current_time_ms() - start_time_ms;
        for (int i = 0; i < 4; ++i) {
            log_data.working_mode[i] = polimi_axis_data[i].working_mode;
            log_data.position_deg[i] = polimi_axis_data[i].position_deg;
            log_data.speed_deg_per_sec[i] = polimi_axis_data[i].speed_deg_per_sec;
            log_data.torque_Nm[i] = polimi_axis_data[i].torque_Nm;
        }
        for (int i = 0; i < 5; ++i)
            log_data.stim_current[i] = (stim_mode == STIM_ON) ? current[i] : 0.0;
        if (!logger->write(log_data)) {
            std::cerr << "[ERROR] CSV not available; request STOP.\n";
            stop_pressed = true;
        }
    }
}

int main() {
    const char *port_name = "/dev/P24";
    p24_stimulator.stim_device = {};
    P24_port_open = smpt_open_serial_port(&p24_stimulator.stim_device, port_name);
    if (P24_port_open)
        P24_initialized = p24_stimulator.init_stimulation(&p24_stimulator.stim_device);
    else
        std::cerr << "[WARNING] Cannot open P24 serial port. FES unavailable.\n";
    std::cout << "P24 initialized: " << (P24_initialized ? "true" : "false") << "\n";

    Ally.reset();
    std::cout << "\n=== Init Control Params ===\n";
    exo_config = exofes::config::load_configuration("./exo_params.json");
    std::cout << "======================================\n";

    if (init(loop) != 0) {
        std::cerr << "[ERROR] Exo callback initialization failed.\n";
        return 1;
    }
    std::cout << "Shared memory reader started...\n";
    molla = get_molla_parameters();
    std::cout << "Rigidezza: " << molla.k << ", Offset: " << molla.offset
              << ", Mano: " << static_cast<int>(molla.hand) << "\n";

    print_menu();
    terminal_input();
    return 0;
}
