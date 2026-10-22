#pragma once
#include <atomic>
#include <memory>

#include "workshop_settings.hpp"
#include "data_logger.hpp"
#include "ml_stimulator.h"

// ============================================================================
// WORKSHOP INTERNAL API - used by the console and pattern modules.
// Students normally do NOT need to edit this file.
// ============================================================================
enum GUI_State
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
};

extern WorkshopSettings workshop_settings;
extern std::atomic<GUI_State> requested_button;
extern std::atomic<bool> stop_requested;
extern std::atomic<bool> stop_confirmed;
extern std::atomic<bool> logger_close_ready;
extern std::atomic<bool> fes_pattern_running;

extern std::unique_ptr<DataLogger> logger;
extern ml_stimulator p24_stimulator;
extern bool P24_port_open;

void print_menu();
void terminal_input();
int run_workshop(const WorkshopSettings &settings);
