#pragma once

# define M_PI		3.14159265358979323846	/* pi */

constexpr int NUM_MUSCLES = 5;
constexpr int NUM_JOINTS  = 4;

constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;   // utile in futuro


// 🦾 INDICI MUSCOLI → usati per pattern, stimolazione, ILC...
constexpr int MUSC_DELTOIDE_ANT = 0;   // J1 - DA  (anterior deltoid)
constexpr int MUSC_DELTOIDE_MED = 1;   // J2 - MED (medial deltoid)
constexpr int MUSC_DELTOIDE_POS = 2;   // J1 - DP  (posterior deltoid)
constexpr int MUSC_TRICEPS      = 3;   // J4 - TRICIPITE
constexpr int MUSC_BICEPS      = 4;   // J4 - BICIPITE



enum Direction { GO=0, BACK=1 };



enum StimDirection {
    NO_EXERCISE       = -1,
    CONTROLATERAL_M   = 0,  // m1
    IPSILATERAL_M     = 1,  // m3
    CONTROLATERAL_T   = 2,  // t1
    IPSILATERAL_T     = 3,  // t3
    ELEV_LATERALE     = 4,
    REACH_CONTROLATERALE = 5,
    REACH_FRONTALE = 6,
    REACH_IPSILATERALE   = 7,
    MANO_ALLA_BOCCA_C      = 8,
    MANO_ALLA_BOCCA_F      = 9,
    MANO_ALLA_BOCCA_I      = 10,
};