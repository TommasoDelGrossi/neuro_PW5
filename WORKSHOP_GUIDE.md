# Exo–FES Workshop
## Student guide: operate the system and design stimulation patterns

In this workshop, you will use an upper-limb exoskeleton together with functional electrical stimulation (FES). Your task is to explore **how the shape of the stimulation-current envelope affects the movement**. You can run three patterns—rectangular, trapezoidal, and biomimetic—and compare the resulting elbow motion using the recorded data.

You do not need to understand or modify the low-level communication with the exoskeleton or the stimulator. Focus on the terminal commands, the pattern parameters, and the files described below.

---

## 1. Your workspace: what to open

```text
neuro_PW5/
|
+-- src/
|   +-- main.cpp                General workshop settings
|   +-- fes_patterns.cpp        Equations of the three current envelopes
|   +-- workshop_console.cpp    Terminal menu (read only)
|   +-- workshop_runtime.cpp    Exercise execution and safety (do not edit)
|
+-- patterns/
|   +-- biomimetic.csv          Shape of the biomimetic envelope
|
+-- WORKSHOP_GUIDE.md           This guide
```

**Where you can work:**

| What you want to do | Where to do it |
|---|---|
| Choose a stimulation pattern | Terminal: `4`, `5`, or `6` |
| Change current, frequency, pulse width, ON time, or repetitions | Terminal, when configuring the pattern |
| Change trapezoidal ramp-up and ramp-down times | Terminal: command `5` |
| Change the biomimetic envelope | `patterns/biomimetic.csv` |
| Explore or edit how the envelopes are calculated | `src/fes_patterns.cpp` |
| Change the Exo start position or shared timing settings | `src/main.cpp` |

You normally **do not need to edit** `workshop_console.cpp` or `workshop_runtime.cpp`. The latter contains device communication, exercise sequencing, and STOP handling.

## 2. Start the program and use the terminal

From the project directory, run:

```bash
./main
```

Enter a **number followed by Enter** to execute a command. Enter **Space + Enter** to display the menu again.

| Command | What it does | What you should expect |
|---|---|---|
| `1` — ENABLE EXO | Enables the Exo session and starts a CSV recording. | `[EXO] Enabled` and the log filename. |
| `2` — GO TO POSITION | Moves the Exo to the start position defined in `main.cpp`. | `[EXO] Target position reached!` when ready. |
| `3` — ENABLE FES | Initializes the P24 if needed. | It may report *already initialized* because initialization normally occurs at program startup. |
| `4` — RECTANGULAR | Configures and selects constant-current ON periods. | Prompts for stimulation settings. |
| `5` — TRAPEZOIDAL | Configures and selects ramps plus a plateau. | Prompts for the same settings and both ramp durations. |
| `6` — BIOMIMETIC | Loads `patterns/biomimetic.csv`, then configures and selects the pattern. | Reports the loaded points and prompts for stimulation settings. |
| `7` — START EXERCISE | Starts AntiG, with the selected FES pattern if available. | AntiG starts first; stimulation starts once the Exo enters AntiG. |
| `8` — STOP EXO + FES | Requests the STOP procedure for both systems. | STOP messages for Exo and FES. |
| `9` — EXIT | Closes the session and the log **after STOP**. | A closure message; follow the terminal instructions to terminate the process. |

**Important:** You can use `7` **without selecting any FES pattern**. This starts an **AntiG-only** exercise. If a pattern has been selected but the P24 is not initialized, AntiG can still start without stimulation, accompanied by a warning.

### Recommended sequence for a trial

```text
1  Enable Exo and start logging
2  Move to the start position
4 / 5 / 6  Select and configure a pattern (optional)
7  Start AntiG and the selected FES pattern
8  Stop Exo and FES when instructed
9  Close the session
```

After a STOP, to start another exercise in the same session, follow the program's restart sequence: **`3` if FES needs reinitialization, then `2` to reposition and `7` to start**. Do not assume FES remains available after `8`.

---

## 3. The three stimulation patterns

All three patterns use the same basic settings:

| Terminal input | Meaning |
|---|---|
| **Physical P24 channel** (`1`–`5`) | The output connected to the muscle you intend to stimulate. |
| **Target current** (mA) | Constant ON current for the rectangle; peak current for trapezoidal and biomimetic patterns. |
| **Frequency** (Hz) | Repetition frequency of the electrical stimulation pulses, held constant during the trial. |
| **Pulse width per phase** (µs) | Duration of each phase of the biphasic stimulation pulse, held constant during the trial. |
| **ON duration** (s) | Length of one active-envelope interval. |
| **Number of repetitions** | Number of ON/OFF cycles to perform. |

The duration of OFF is shared across all patterns:

\[
T_{\mathrm{OFF}}=1.5\,T_{\mathrm{ON}}
\]

During **OFF**, the software commands **zero current**. The Exo remains in AntiG during both ON and OFF. When the last OFF interval finishes, the FES pattern ends but **AntiG continues** until the Exo exercise ends or you issue STOP.

### A. Rectangular — command `4`

During ON, the current stays at the value you entered; during OFF, it is zero. There are **no current ramps**.

```text
Current
  Ipeak  +------------+             +------------+
         |            |             |            |
     0 --+            +-------------+            +----> time
              ON             OFF          ON
```

**Use this pattern** as a simple baseline to compare against time-varying envelopes.

### B. Trapezoidal — command `5`

In addition to the shared settings, enter:

- **Ramp UP duration** (s): time to rise linearly from 0 to the target current.
- **Ramp DOWN duration** (s): time to fall linearly from the target current to 0.

Both ramps are **part of ON**, not extra time added to it:

\[
T_{\mathrm{ON}}=T_{\mathrm{UP}}+T_{\mathrm{PLATEAU}}+T_{\mathrm{DOWN}}
\]

For example, `ON = 3 s`, `UP = 0.5 s`, and `DOWN = 0.7 s` give a **1.8 s plateau**. The program rejects ramp settings whose sum exceeds ON.

```text
Current
  Ipeak       +----------+
            /              \
           /                \
     0 ---+                  +----------------------> time
           UP     PLATEAU   DOWN          OFF
```

**Try changing the ramp durations** while keeping the peak current and total ON time fixed. This changes the shape of the envelope without changing the overall ON duration.

### C. Biomimetic — command `6`

The biomimetic envelope is read from the fixed file:

```text
patterns/biomimetic.csv
```

The file defines the **shape**, not the absolute current or duration. You enter the **peak current** and **ON duration** in the terminal; the program scales the CSV accordingly.

For a normalized envelope \(f(\tau)\):

\[
\tau=\frac{t}{T_{\mathrm{ON}}},\qquad
I(t)=I_{\mathrm{peak}}\,f(\tau)
\]

For instance, with `ON = 2.5 s` and `peak = 20 mA`, the CSV point `(0.4, 0.8)` corresponds to **1.0 s and 16 mA**.

#### How to write the CSV

Use exactly two comma-separated columns and this header:

```csv
time_norm,current_norm
0.00,0.00
0.10,0.20
0.30,0.85
0.40,1.00
0.65,0.60
0.85,0.15
1.00,0.00
```


The rules are simple:

- `time_norm` must increase strictly from **0 to 1**.
- `current_norm` must remain between **0 and 1**.
- Start at `(0,0)`, finish at `(1,0)`, and include a peak of exactly **1**.
- You may include **multiple peaks** if you want to test more complex envelopes.
- **501 evenly spaced samples** are recommended, but fewer or more points are accepted. The program interpolates linearly between them.

You can edit the CSV in a spreadsheet editor or with Python. Save it as a plain CSV with a **comma** between columns and a **dot** as the decimal separator. The file is read **when you press `6`**: after changing it, press `6` again to load the updated shape. You do **not** need to recompile.

The provided CSV is a **synthetic demonstration profile**, not a profile extracted from experimental EMG.

---

## 4. Where to change the code

### `src/main.cpp` — workshop settings

Open this file to see the main configurable boxes:

| Setting | What it changes |
|---|---|
| `settings.start_position_deg` | Starting angles in degrees, ordered as `[J1, J2, J3, J4]`; **J4 is the elbow**. |
| `settings.position_move_ms` | Time assigned to move to the start position, in milliseconds. |
| `settings.antig_duration_ms` | Maximum duration of the AntiG exercise, in milliseconds. |
| `settings.off_to_on_ratio` | OFF duration divided by ON duration, for all patterns. Default: `1.5`. |
| `settings.biomimetic_csv_path` | Fixed biomimetic file location. Normally leave unchanged. |

These settings affect future program runs. If you modify C++ code, **rebuild** before restarting the program.

### `src/fes_patterns.cpp` — envelope equations

This is the file to explore when you want to understand or develop new stimulation shapes. Look for:

- `rectangular_current(...)`: returns a constant target current during ON.
- `trapezoidal_current(...)`: calculates the linear rise, plateau, and fall.
- `biomimetic_current(...)`: scales the normalized, interpolated CSV envelope.
- `current_during_fes_on(...)`: chooses which of the three functions to use.

These functions **calculate commanded current only**. They do not handle individual pulses or USB communication. That separation lets you work on envelope design without modifying the stimulator interface.

If you are only experimenting with the **shape of a biomimetic profile**, edit `patterns/biomimetic.csv` rather than changing C++.

To rebuild after editing C++ (from the project directory):

```bash
cmake --build build --target main -j4
./main
```

Do not recompile while a trial is running. Stop the session and follow the instructor's procedure before restarting.

---

## 5. Observe and compare your results

When you press `1`, the program creates a recording named similarly to:

```text
File_Log_1792598715.csv
```

Look in the directory where you launched `./main`. The useful recorded signals include:

| Signal | What you can investigate |
|---|---|
| `timestamp` | When events and movements occurred (milliseconds). |
| `pos_4_deg` | Measured elbow angle (J4). |
| `speed_4_deg_per_s` | Elbow angular velocity. |
| `stim_current_1` … `stim_current_5` | **Commanded** current envelope for each configured channel. |
| `working_mode_1` … `working_mode_4` | Exoskeleton operating modes during the exercise. |

A useful first plot has **two panels sharing the same time axis**: **J4 angle above**, **FES current below**. Compare the timing of the movement against the ON/OFF phases and the shape of the stimulation envelope.

**What to look for:** Does a rapid current onset differ from a gradual ramp? Does the elbow reach a different angle? Does it move differently during OFF? Keep the task, posture, stimulation settings, and assistance conditions comparable when evaluating different envelopes.

Remember: `stim_current_*` stores the **software command**, not an independent measurement of delivered electrical current. A difference in elbow motion alone does not prove that FES caused the change.

---

## 6. Before finishing

Stop the Exo and FES using **command `8`**, confirm the expected STOP indications with your instructor, and then use **command `9`** to close the session and its recording. Follow the displayed instruction to terminate the process.

**Your goal is to design, run, and compare stimulation envelopes—not to change the robot's safety or communication layers.**
