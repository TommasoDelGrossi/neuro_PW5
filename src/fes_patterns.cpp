#include "fes_patterns.hpp"
#include "workshop_runtime.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <utility>

// ============================================================================
// BOX A - PATTERN CONFIGURATION (written by the terminal thread)
// ============================================================================
FesPatternConfig rectangular_config;
FesPatternConfig trapezoidal_config;
FesPatternConfig biomimetic_config;
std::mutex biomimetic_config_mutex;
std::vector<BiomimeticPoint> biomimetic_profile;
std::atomic<FesPatternType> selected_fes_pattern{FesPatternType::NONE};

// ============================================================================
// BOX B - CURRENT ENVELOPES (pure waveform calculations; NO USB commands)
// ============================================================================
const char *fes_pattern_name(FesPatternType type)
{
    switch (type)
    {
    case FesPatternType::RECTANGULAR: return "Rectangular";
    case FesPatternType::TRAPEZOIDAL: return "Trapezoidal";
    case FesPatternType::BIOMIMETIC: return "Biomimetic";
    default: return "None";
    }
}

// Piecewise-linear interpolation: no overshoot between CSV samples.
// The CSV describes a single ON interval, normalized on both axes.
double biomimetic_amplitude(const std::vector<BiomimeticPoint> &points, double tau)
{
    if (points.size() < 2 || tau <= 0.0)
        return 0.0;
    if (tau >= 1.0)
        return 0.0;

    const auto right = std::lower_bound(
        points.begin(), points.end(), tau,
        [](const BiomimeticPoint &point, double t) {
            return point.time_norm < t;
        });
    if (right == points.begin())
        return right->current_norm;
    if (right == points.end())
        return points.back().current_norm;

    const auto &left = *(right - 1);
    const double fraction = (tau - left.time_norm) /
                            (right->time_norm - left.time_norm);
    return left.current_norm +
           fraction * (right->current_norm - left.current_norm);
}

// --- BOX B1: RECTANGULAR ENVELOPE ------------------------------------------
// The ON phase has constant amplitude; the sequencer owns ON/OFF timing.
static double rectangular_current(const FesPatternConfig &cfg)
{
    return cfg.current_mA;
}

// --- BOX B2: TRAPEZOIDAL ENVELOPE ------------------------------------------
// UP and DOWN are expressed in ms and are INCLUDED in ON duration.
static double trapezoidal_current(const FesPatternConfig &cfg, long long elapsed_ms)
{
    if (elapsed_ms >= cfg.on_ms)
        return 0.0;

    if (cfg.ramp_up_ms > 0 && elapsed_ms < cfg.ramp_up_ms)
        return cfg.current_mA * static_cast<double>(elapsed_ms) / cfg.ramp_up_ms;

    const int down_start_ms = cfg.on_ms - cfg.ramp_down_ms;
    if (cfg.ramp_down_ms > 0 && elapsed_ms >= down_start_ms)
        return cfg.current_mA *
               static_cast<double>(cfg.on_ms - elapsed_ms) / cfg.ramp_down_ms;

    return cfg.current_mA;
}

// --- BOX B3: BIOMIMETIC ENVELOPE ------------------------------------------
// Convert elapsed time to [0,1], interpolate the CSV, scale by peak current.
static double biomimetic_current(const FesPatternConfig &cfg,
    long long elapsed_ms, const std::vector<BiomimeticPoint> &profile)
{
    if (elapsed_ms >= cfg.on_ms || cfg.on_ms <= 0)
        return 0.0;
    const double tau = static_cast<double>(elapsed_ms) / cfg.on_ms;
    return cfg.current_mA * biomimetic_amplitude(profile, tau);
}

// --- BOX B4: SELECT ONE ENVELOPE -------------------------------------------
// Pure computation: communication is handled in workshop_runtime.cpp.
double current_during_fes_on(const FesPatternConfig &cfg,
                            FesPatternType type, long long elapsed_ms,
                            const std::vector<BiomimeticPoint> &active_biomimetic_profile)
{
    if (type == FesPatternType::RECTANGULAR)
        return rectangular_current(cfg);

    if (type == FesPatternType::TRAPEZOIDAL)
        return trapezoidal_current(cfg, elapsed_ms);

    if (type == FesPatternType::BIOMIMETIC)
        return biomimetic_current(cfg, elapsed_ms, active_biomimetic_profile);

    return 0.0;
}


// ============================================================================
// BOX C - BIOMIMETIC CSV (one read on command 6; never in control loop)
// ============================================================================
// CSV is loaded on command 6 in the terminal thread, not in the control loop.
// Format: time_norm,current_norm; strictly increasing x, both axes in [0,1].
// A zero start/end and a unit peak ensure exactly 0..Ipeak scaling.
std::string trim_csv_field(const std::string &s)
{
    const auto first = s.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return "";
    const auto last = s.find_last_not_of(" \t\r\n");
    return s.substr(first, last - first + 1);
}

bool parse_csv_number(const std::string &field, double &value)
{
    const std::string token = trim_csv_field(field);
    if (token.empty())
        return false;
    try
    {
        size_t consumed = 0;
        value = std::stod(token, &consumed);
        return consumed == token.size() && std::isfinite(value);
    }
    catch (const std::exception &)
    {
        return false;
    }
}

bool load_biomimetic_csv(const char *path, std::vector<BiomimeticPoint> &result)
{
    std::ifstream file(path);
    if (!file.is_open())
    {
        std::cerr << "[FES] Cannot open biomimetic CSV: " << path << "\n";
        return false;
    }

    std::string line;
    if (!std::getline(file, line))
    {
        std::cerr << "[FES] Empty biomimetic CSV.\n";
        return false;
    }
    // Accept UTF-8 BOM and CRLF in addition to plain UTF-8/LF.
    if (line.size() >= 3 &&
        static_cast<unsigned char>(line[0]) == 0xEF &&
        static_cast<unsigned char>(line[1]) == 0xBB &&
        static_cast<unsigned char>(line[2]) == 0xBF)
        line.erase(0, 3);
    if (trim_csv_field(line) != "time_norm,current_norm")
    {
        std::cerr << "[FES] Expected CSV header: time_norm,current_norm\n";
        return false;
    }

    std::vector<BiomimeticPoint> parsed;
    parsed.reserve(501);
    size_t line_number = 1;
    double peak = 0.0;
    while (std::getline(file, line))
    {
        ++line_number;
        if (trim_csv_field(line).empty())
            continue;
        const auto comma = line.find(',');
        if (comma == std::string::npos || line.find(',', comma + 1) != std::string::npos)
        {
            std::cerr << "[FES] CSV line " << line_number << ": expected exactly two columns.\n";
            return false;
        }
        BiomimeticPoint point{};
        if (!parse_csv_number(line.substr(0, comma), point.time_norm) ||
            !parse_csv_number(line.substr(comma + 1), point.current_norm) ||
            point.time_norm < 0.0 || point.time_norm > 1.0 ||
            point.current_norm < 0.0 || point.current_norm > 1.0 ||
            (!parsed.empty() && point.time_norm <= parsed.back().time_norm))
        {
            std::cerr << "[FES] CSV line " << line_number
                      << ": invalid normalized values or time order.\n";
            return false;
        }
        parsed.push_back(point);
        peak = std::max(peak, point.current_norm);
        if (parsed.size() > 10001)
        {
            std::cerr << "[FES] Biomimetic CSV exceeds 10001 points.\n";
            return false;
        }
    }
    if (file.bad())
    {
        std::cerr << "[FES] Error while reading biomimetic CSV.\n";
        return false;
    }

    constexpr double eps = 1e-6;
    if (parsed.size() < 2 ||
        std::abs(parsed.front().time_norm) > eps ||
        std::abs(parsed.front().current_norm) > eps ||
        std::abs(parsed.back().time_norm - 1.0) > eps ||
        std::abs(parsed.back().current_norm) > eps ||
        std::abs(peak - 1.0) > eps)
    {
        std::cerr << "[FES] CSV requires (0,0), (1,0) and a peak equal to 1.\n";
        return false;
    }

    result = std::move(parsed);
    return true;
}

bool read_terminal_double(const std::string &prompt, double &value)
{
    std::string line;
    std::cout << prompt;
    if (!std::getline(std::cin, line))
        return false;
    std::stringstream ss(line);
    char extra;
    return (ss >> value) && !(ss >> extra);
}

bool read_terminal_int(const std::string &prompt, int &value)
{
    std::string line;
    std::cout << prompt;
    if (!std::getline(std::cin, line))
        return false;
    std::stringstream ss(line);
    char extra;
    return (ss >> value) && !(ss >> extra);
}

// Pattern configuration runs in the terminal thread, NOT in the Exo callback.
// Values are copied to active_fes_config once at START_EXERCISE.
void configure_fes_pattern(FesPatternType type)
{
    if (fes_pattern_running.load())
    {
        std::cout << "[WARNING] Cannot reconfigure FES during an active pattern.\n";
        return;
    }

    FesPatternConfig cfg;
    std::vector<BiomimeticPoint> loaded_biomimetic;
    if (type == FesPatternType::BIOMIMETIC)
    {
        // A failed command 6 must not leave a stale pattern selected for 7.
        selected_fes_pattern.store(FesPatternType::NONE);
        if (!load_biomimetic_csv(workshop_settings.biomimetic_csv_path, loaded_biomimetic))
        {
            std::cout << "[WARNING] Biomimetic pattern not selected. Command 7 remains AntiG-only.\n";
            return;
        }
        std::cout << "[FES] Loaded " << loaded_biomimetic.size()
                  << " biomimetic points from " << workshop_settings.biomimetic_csv_path << ".\n";
    }
    double on_s = 0.0;
    double ramp_up_s = 0.0;
    double ramp_down_s = 0.0;

    std::cout << "\n--- " << (type == FesPatternType::RECTANGULAR
        ? "RECTANGULAR" : (type == FesPatternType::TRAPEZOIDAL
        ? "TRAPEZOIDAL" : "BIOMIMETIC")) << " FES CONFIGURATION ---\n";
    if (!read_terminal_int("Physical P24 channel [1-5]: ", cfg.channel) ||
        cfg.channel < 1 || cfg.channel > 5)
    {
        std::cout << "[WARNING] Invalid channel. Configuration cancelled.\n";
        return;
    }
    if (!read_terminal_double("Target current [mA, >0 and <=150]: ", cfg.current_mA) ||
        !std::isfinite(cfg.current_mA) || cfg.current_mA <= 0.0 || cfg.current_mA > 150.0)
    {
        std::cout << "[WARNING] Invalid current. Configuration cancelled.\n";
        return;
    }
    if (!read_terminal_double("Frequency [Hz, 1-1000]: ", cfg.frequency_Hz) ||
        !std::isfinite(cfg.frequency_Hz) || cfg.frequency_Hz < 1.0 || cfg.frequency_Hz > 1000.0)
    {
        std::cout << "[WARNING] Invalid frequency. Configuration cancelled.\n";
        return;
    }
    if (!read_terminal_int("Pulse width per phase [us, 10-4095]: ", cfg.pulse_width_us) ||
        cfg.pulse_width_us < 10 || cfg.pulse_width_us > 4095)
    {
        std::cout << "[WARNING] Invalid pulse width. Configuration cancelled.\n";
        return;
    }
    if (!read_terminal_double("ON duration [s, >0]: ", on_s) ||
        !std::isfinite(on_s) || on_s < 0.001 || on_s > 3600.0)
    {
        std::cout << "[WARNING] Invalid ON duration (1 ms to 3600 s).\n";
        return;
    }

    cfg.on_ms = static_cast<int>(std::lround(on_s * 1000.0));
    cfg.off_ms = static_cast<int>(std::lround(workshop_settings.off_to_on_ratio * cfg.on_ms));

    if (type == FesPatternType::TRAPEZOIDAL)
    {
        std::cout << "[FES] Ramp times are in seconds and INCLUDED in ON duration.\n";
        if (!read_terminal_double("Ramp UP duration [s, >=0]: ", ramp_up_s) ||
            !std::isfinite(ramp_up_s) || ramp_up_s < 0.0 || ramp_up_s > on_s)
        {
            std::cout << "[WARNING] Invalid ramp UP duration.\n";
            return;
        }
        if (!read_terminal_double("Ramp DOWN duration [s, >=0]: ", ramp_down_s) ||
            !std::isfinite(ramp_down_s) || ramp_down_s < 0.0 || ramp_down_s > on_s)
        {
            std::cout << "[WARNING] Invalid ramp DOWN duration.\n";
            return;
        }
        cfg.ramp_up_ms = static_cast<int>(std::lround(ramp_up_s * 1000.0));
        cfg.ramp_down_ms = static_cast<int>(std::lround(ramp_down_s * 1000.0));
        if (cfg.ramp_up_ms + cfg.ramp_down_ms > cfg.on_ms)
        {
            std::cout << "[WARNING] UP + DOWN cannot exceed ON duration.\n";
            return;
        }
    }

    if (!read_terminal_int("Number of repetitions [>=1]: ", cfg.repetitions) ||
        cfg.repetitions < 1)
    {
        std::cout << "[WARNING] Invalid repetition count. Configuration cancelled.\n";
        return;
    }

    if (type == FesPatternType::RECTANGULAR)
        rectangular_config = cfg;
    else if (type == FesPatternType::TRAPEZOIDAL)
        trapezoidal_config = cfg;
    else if (type == FesPatternType::BIOMIMETIC)
    {
        std::lock_guard<std::mutex> guard(biomimetic_config_mutex);
        biomimetic_config = cfg;
        biomimetic_profile = std::move(loaded_biomimetic);
    }
    selected_fes_pattern.store(type);

    std::cout << "[FES] " << fes_pattern_name(type) << " selected: channel "
              << cfg.channel << ", " << cfg.current_mA << " mA, "
              << cfg.frequency_Hz << " Hz, PW " << cfg.pulse_width_us
              << " us, ON " << cfg.on_ms / 1000.0 << " s, OFF "
              << cfg.off_ms / 1000.0 << " s, repetitions " << cfg.repetitions;
    if (type == FesPatternType::TRAPEZOIDAL)
        std::cout << ", ramp UP " << cfg.ramp_up_ms / 1000.0
                  << " s, ramp DOWN " << cfg.ramp_down_ms / 1000.0
                  << " s, plateau "
                  << (cfg.on_ms - cfg.ramp_up_ms - cfg.ramp_down_ms) / 1000.0
                  << " s";
    std::cout << ".\n";
}
