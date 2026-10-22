# Exo–FES workshop — student guide

The program has been divided by responsibility. **The `src/main.cpp` file is the
student-facing entry point**; it contains only configurable settings and the call
to start the workshop. Communication and safety code live in separate modules.

## Where to work

```text
neuro_PW5/
|
+-- src/
|   +-- main.cpp                 [BOX 1] Student settings + program entry point
|   +-- fes_patterns.cpp         [BOX 2] Envelope equations + CSV loader
|   +-- workshop_console.cpp     [BOX 3] Menu + terminal input
|   +-- workshop_runtime.cpp     [BOX 4] Exo FSM, P24, STOP, cyclic loop, logger
|   +-- ml_stimulator.cpp        [DEVICE] Existing three-point biphasic ML_Update
|   +-- utils_exofes.cpp         [DEVICE] Existing Exo controller
|
+-- include/
|   +-- workshop_settings.hpp    Settings structure used by main.cpp
|   +-- fes_patterns.hpp         Pattern structures and function declarations
|   +-- workshop_runtime.hpp     Internal communication between modules
|
+-- patterns/
|   +-- biomimetic.csv           [DATA] Normalized current envelope
|
+-- tests/
|   +-- test_fes_patterns.cpp    Offline numerical tests (no patient/device)
|
+-- CMakeLists.txt               Builds all modules into the same ./main
```

### BOX 1 — `src/main.cpp`

The only *source-code* settings students normally need to change are:

- `settings.start_position_deg` = `[J1,J2,J3,J4]` in degrees;
- `settings.position_move_ms` (3000 by default);
- `settings.antig_duration_ms` (600000 by default);
- `settings.off_to_on_ratio` (1.5 by default);
- `settings.biomimetic_csv_path` (fixed file path, not asked in the terminal).

**Do not edit the P24 initialization, serial transport, emergency STOP, or Exo
communication to customize the stimulation envelope.**

### BOX 2 — `src/fes_patterns.cpp`

The waveform generation is isolated into four small boxes:

- **B1** `rectangular_current`: constant target current during ON.
- **B2** `trapezoidal_current`: linear UP, plateau, linear DOWN within ON.
- **B3** `biomimetic_current`: CSV interpolation scaled by target current.
- **B4** `current_during_fes_on`: selects B1, B2, or B3.

The **CSV reader** and **parameter prompts** are in separate boxes below the
equations. They run in the terminal thread, not the 100 Hz control callback.

The shared FES protocol remains unchanged: one physical P24 channel in the
range 1–5, biphasic pulses, frequency and PW constant during each exercise,
`OFF = off_to_on_ratio × ON`, and current zero during OFF.

### BOX 3 — `src/workshop_console.cpp`

This file contains only `print_menu` and `terminal_input`. Controls:

| Command | Action |
|---|---|
| `1` | Enable Exo and start the CSV logger |
| `2` | Move to start position |
| `3` | Reinitialize FES after STOP, if needed |
| `4` | Configure and select rectangular FES |
| `5` | Configure and select trapezoidal FES |
| `6` | Load the fixed biomimetic CSV; configure and select biomimetic FES |
| `7` | Start AntiG, with FES only if selected and initialized |
| `8` | STOP Exo + FES (`ML_Stop`) |
| `9` | Close a stopped session and its CSV log |
| Space + Enter | Reprint the menu |

`7` is **independent of FES**. A configured pattern is not required for AntiG.

### BOX 4 — `src/workshop_runtime.cpp`

This is for instructors/maintainers rather than student editing:

```text
STARTUP:  open /dev/P24 -> ML_Init -> initialize Exo (original order)
    |
    v
LOOP:     commands -> safety STOP -> Exo position/AntiG
    |                          |
    |                          +-> cyclic FES command -> ML_Update
    |                                              |
    +----------------------------------------------+-> CSV logger
```

The P24/STOP logic, current dispatch, periodic `ML_Get_Current_Data`, and
position/AntiG state transitions are carried over from the previous program.
Successful writing of `ML_Stop` is **not** proof that hardware output has stopped.
Keep an independent hardware emergency stop and perform device-level safety
checks before connecting a participant.

## Biomimetic CSV

Located at `./patterns/biomimetic.csv` (relative to the working directory):

```csv
time_norm,current_norm
0.000,0.000
...
1.000,0.000
```

The CSV must contain at least two samples, strictly increasing normalized
times, a maximum normalized current of 1, and 0 current at both endpoints.
The supplied 501-point file is a **synthetic test profile**, not clinical data.
The loader interpolates linearly; 501 points are recommended but not required.

## Build and run

From the *project root*:

```bash
cmake -S . -B build
cmake --build build --target main -j4
./main
```

The existing project still expects `utils.h` and `utils.c` in the parent/system
location as specified by `CMakeLists.txt`, and requires Eigen3 and the included
SMPT library. The refactoring does not change these dependencies.

## Safety and regression checklist

1. `1 -> 2 -> 7`: Exo AntiG without any FES pattern.
2. `1 -> 2 -> 4 -> 7`: rectangular current, zero-current OFF.
3. `1 -> 2 -> 5 -> 7`: trapezoid with linear ramps and plateau.
4. `1 -> 2 -> 6 -> 7`: CSV envelope and normalized scaling.
5. `8`: no more pattern updates and STOP for Exo/FES.
6. `3` after STOP: reinitialization; `2 -> 7` restarts the Exo protocol.
7. `9` after confirmed STOP: close the logger and serial port.

Perform initial validation using an appropriate test load, **not a person**.

The refactor has undergone an offline build and numerical profile tests, but
it has not been validated with the real P24 and Exo hardware after splitting
source files.
