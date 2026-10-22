#include "workshop_runtime.hpp"

// ============================================================================
//                       EXO + FES STUDENT WORKSHOP
// ============================================================================
// EDIT THIS FILE for workshop-level settings.
// PATTERN PARAMETERS are entered at the terminal (commands 4, 5, and 6).
// BIOMIMETIC SHAPE is edited in patterns/biomimetic.csv.
//
// Do not change USB, ML_Init/ML_Update/ML_Stop, or the emergency STOP logic
// when experimenting with stimulation envelopes.
// ============================================================================
int main()
{
    WorkshopSettings settings;

    // ========================================================================
    // BOX 1 - EXOSKELETON START POSITION (degrees)
    // J1 = shoulder horizontal movement
    // J2 = shoulder elevation
    // J3 = humeral rotation
    // J4 = elbow flexion/extension
    // ========================================================================
    settings.start_position_deg = {{90.0f, 0.0f, 90.0f, 90.0f}};

    // ========================================================================
    // BOX 2 - EXOSKELETON TIMING
    // Keep these values identical to the previously tested implementation.
    // ========================================================================
    settings.position_move_ms = 3000;   // 3 s positioning
    settings.antig_duration_ms = 600000; // 10 min AntiG maximum

    // ========================================================================
    // BOX 3 - COMMON FES PROTOCOL
    // OFF = factor * ON (for rectangle / trapezoid / biomimetic).
    // Amplitude, PW, frequency, channel, reps and ON are entered in the menu.
    // ========================================================================
    settings.off_to_on_ratio = 1.5;

    // ========================================================================
    // BOX 4 - BIOMIMETIC CSV (fixed path, no terminal path input)
    // The CSV axes must both be normalized [0, 1].
    // ========================================================================
    settings.biomimetic_csv_path = "./patterns/biomimetic.csv";

    // ========================================================================
    // BOX 5 - RUN WORKSHOP
    // Internal modules handle the terminal, Exo, P24, safety and logging.
    // ========================================================================
    return run_workshop(settings);
}
