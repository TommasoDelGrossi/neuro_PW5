// Offline tests: no P24, Exo or serial port is needed.
#include "fes_patterns.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
#include <vector>

static void near(double got, double expected)
{
    if (std::abs(got - expected) > 1e-8) {
        std::cerr << "Expected " << expected << ", got " << got << '\n';
        std::abort();
    }
}

int main()
{
    FesPatternConfig cfg;
    cfg.current_mA = 10;
    cfg.on_ms = 3000;
    cfg.ramp_up_ms = 500;
    cfg.ramp_down_ms = 700;
    const std::vector<BiomimeticPoint> empty;

    // RECTANGULAR: exactly constant current throughout ON.
    near(current_during_fes_on(cfg, FesPatternType::RECTANGULAR, 0, empty), 10);
    near(current_during_fes_on(cfg, FesPatternType::RECTANGULAR, 2999, empty), 10);

    // TRAPEZOIDAL: UP 0.5 s, plateau 1.8 s, DOWN 0.7 s.
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 0, empty), 0);
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 250, empty), 5);
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 500, empty), 10);
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 2300, empty), 10);
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 2650, empty), 5);
    near(current_during_fes_on(cfg, FesPatternType::TRAPEZOIDAL, 3000, empty), 0);

    // BIOMIMETIC: use the same CSV that students edit in patterns/.
    std::vector<BiomimeticPoint> profile;
    assert(load_biomimetic_csv("./patterns/biomimetic.csv", profile));
    assert(profile.size() == 501);
    cfg.on_ms = 2000;
    near(current_during_fes_on(cfg, FesPatternType::BIOMIMETIC, 0, profile), 0);
    near(current_during_fes_on(cfg, FesPatternType::BIOMIMETIC, 800, profile), 10);
    near(current_during_fes_on(cfg, FesPatternType::BIOMIMETIC, 2000, profile), 0);
    near(biomimetic_amplitude(profile, 1.0), 0);

    // Interpolator: a triangular three-point profile.
    const std::vector<BiomimeticPoint> triangle{{0,0},{0.5,1},{1,0}};
    near(biomimetic_amplitude(triangle, 0.25), 0.5);
    near(biomimetic_amplitude(triangle, 0.75), 0.5);

    std::cout << "All waveform and CSV tests passed.\n";
    return 0;
}
