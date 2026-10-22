#pragma once
#include <array>

// ============================================================================
// BOX 1 - STUDENT SETTINGS
// Only change these values in main.cpp during the workshop.
// The defaults exactly match the validated pre-refactor program.
// ============================================================================
struct WorkshopSettings
{
    // Joint order: J1, J2, J3, J4. Right arm starting pose.
    std::array<float, 4> start_position_deg{{90.0f, 0.0f, 90.0f, 90.0f}};

    // Position profiler duration (before starting AntiG).
    int position_move_ms = 3000;

    // Maximum duration of the AntiG exercise; independent of FES duration.
    int antig_duration_ms = 600000;

    // FES OFF interval = this factor * ON interval (all patterns).
    double off_to_on_ratio = 1.5;

    // Fixed path: never requested from terminal. Relative to working directory.
    const char *biomimetic_csv_path = "./patterns/biomimetic.csv";
};
