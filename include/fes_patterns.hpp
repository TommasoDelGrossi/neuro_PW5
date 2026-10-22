#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

// ============================================================================
// BOX 2 - FES PATTERNS: public data model and configurable envelopes.
// All timestamps within a pattern are expressed in milliseconds.
// ============================================================================
struct FesPatternConfig
{
    int channel = 1;          // Physical P24 channel: 1..5 in the present wrapper
    double current_mA = 0.0;  // Target/peak current
    double frequency_Hz = 30.0;
    int pulse_width_us = 400;
    int on_ms = 2000;
    int off_ms = 3000;        // Default: 1.5 * ON
    int repetitions = 1;
    int ramp_up_ms = 0;       // Trapezoid: included in ON
    int ramp_down_ms = 0;     // Trapezoid: included in ON
};

enum class FesPatternType
{
    NONE,
    RECTANGULAR,
    TRAPEZOIDAL,
    BIOMIMETIC
};

enum class FesPatternPhase
{
    IDLE,
    ON,
    OFF,
    COMPLETE
};

struct BiomimeticPoint
{
    double time_norm;
    double current_norm;
};

// Terminal-configured values. The runtime snapshots them at exercise start.
extern FesPatternConfig rectangular_config;
extern FesPatternConfig trapezoidal_config;
extern FesPatternConfig biomimetic_config;
extern std::mutex biomimetic_config_mutex;
extern std::vector<BiomimeticPoint> biomimetic_profile;
extern std::atomic<FesPatternType> selected_fes_pattern;

const char *fes_pattern_name(FesPatternType type);
bool load_biomimetic_csv(const char *path, std::vector<BiomimeticPoint> &result);
double biomimetic_amplitude(const std::vector<BiomimeticPoint> &points, double tau);
double current_during_fes_on(const FesPatternConfig &cfg,
    FesPatternType type, long long elapsed_ms,
    const std::vector<BiomimeticPoint> &active_biomimetic_profile);
void configure_fes_pattern(FesPatternType type);
