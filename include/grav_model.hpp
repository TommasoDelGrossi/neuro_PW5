#ifndef GRAV_MODEL_H
#define GRAV_MODEL_H

#include "utils.h"
#include "utils_exofes.hpp"
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include "constants.h"

// #define DEG2RAD (M_PI / 180.0)
// #define RAD2DEG (180.0 / M_PI)

#define N_JOINT 4

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    J1 = 0,
    J2 = 1,
    J3 = 2,
    J4 = 3
} joint_t;

typedef struct
{
    float q_model[N_JOINT];

    float tau_gravity_Nm[N_JOINT];

} antig_output_t;

int antig_compute_from_pos(
    const float pos_rad[N_JOINT],
    antig_output_t *out
);

// Struttura interna per i parametri paziente formattati per la cinematica
typedef struct {
    float patient_weight_kg;
    float patient_height_m;
} human_params_t;

/* * @brief: Salva i parametri ricevuti dal pacchetto TCP
 * @param weight_tcp: peso kg
 * @param height_tcp: altezza cm
 */
void antig_set_parameters(uint16_t weight_tcp, uint16_t height_tcp);

/* * @brief: Restituisce una copia dei parametri attuali del paziente
 * @return: struttura human_params_t con i valori in float (kg e metri)
 */
human_params_t antig_get_parameters(void);

/*
 * @brief: Ricalcola le masse del modello unendo il robot al braccio del paziente
 * @param alpha_assistance: fattore di compensazione (es. 0.5 per 50%, 1.0 per 100%)
 * @return: 0 in caso di successo, -1 in caso di errore (es. parametri mancanti)
 */
int antig_init_human_coupling(float alpha_assistance);


#ifdef __cplusplus
}
#endif

#endif