#include "grav_model.hpp"
#include <string.h>
#include <Eigen/Dense>

using namespace Eigen;

/* ============================================================
 * Internal constants
 * ============================================================ */

#define N_MDH 6
#define N_BODY 7

#define MDH_FIXED -1

/* Body indices internal to the MDH chain */

#define BODY_BASE 0
#define BODY_J1 1
#define BODY_J2 2
#define BODY_ALIGNMENT 3
#define BODY_J3 4
#define BODY_J4 5
#define BODY_HAND 6

/* ============================================================
 * Internal types
 * ============================================================ */

typedef struct // riga trasformazione MDH
{
    float a;
    float alpha;
    float d;
    float theta;
    int joint_id;
} mdh_row_t;

/* ============================================================
 * Modified Denavitt-Hartenberg model
 * ============================================================ */

/*
 * MDH convention:
 *
 * row = [a, alpha, d, theta]
 *
 */

static const mdh_row_t mdh_table[N_MDH] =
    {
        /* a      alpha        d      theta        joint */
        {0.0f, M_PI, 0.0f, 0.0f, J1},
        {0.0f, -M_PI_2, 0.0f, 0.0f, J2},
        {0.0f, 0.0f, 0.0f, M_PI_2, MDH_FIXED},
        {0.0f, M_PI_2, 0.30f, 0.0f, J3},
        {0.0f, -M_PI_2, 0.0f, 0.0f, J4},
        {0.0f, M_PI_2, 0.25f, 0.0f, MDH_FIXED}};

/*
 * Each MDH row creates one body frame.
 *
 * row 0 -> BODY_J1
 * row 1 -> BODY_J2
 * row 2 -> BODY_ALIGNMENT
 * row 3 -> BODY_J3
 * row 4 -> BODY_J4
 * row 5 -> BODY_HAND
 */

static const int mdh_body_id[N_MDH] =
    {
        BODY_J1,
        BODY_J2,
        BODY_ALIGNMENT,
        BODY_J3,
        BODY_J4,
        BODY_HAND};

/* ============================================================
 * Masses and CoM
 * ============================================================ */

static const Vector3f gravity_world(0.0f, 0.0f, -9.81f);

static float body_mass[N_BODY] =
    {
        0.0f,  // BODY_BASE
        2.89f, // BODY_J1
        0.0f,  // BODY_J2
        0.0f,  // BODY_ALIGNMENT
        3.00f, // BODY_J3
        0.0f,  // BODY_J4
        0.556f // BODY_HAND
};

static Vector3f body_com[N_BODY] = {

    Vector3f(0.0f, 0.0f, 0.0f),             // BODY_BASE
    Vector3f(0.0f, 0.0f, 0.0f),             // BODY_J1
    Vector3f(0.0f, 0.0f, 0.0f),             // BODY_J2
    Vector3f(0.0f, 0.0f, 0.0f),             // BODY_ALIGNMENT
    Vector3f(-0.0212f, -0.0399f, -0.2018f), // BODY_J3 --> ARM
    Vector3f(0.0f, 0.0f, 0.0f),             // BODY_J4
    Vector3f(-0.0335f, -0.0276f, -0.1730f)  // BODY_HAND --> FOREARM
};

static const float robot_mass_nominal[N_BODY] =
    {
        0.0f, 2.89f, 0.0f, 0.0f, 3.00f, 0.0f, 0.556f};

static const Vector3f robot_com_nominal[N_BODY] = {
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(-0.0212f, -0.0399f, -0.2018f),
    Vector3f(0.0f, 0.0f, 0.0f),
    Vector3f(-0.0335f, -0.0276f, -0.1730f)};

/* ============================================================
 * Joint-body dependency
 * ============================================================
 *
 * dependency[joint][body] = true se il body è a valle del giunto.
 *
 * Catena:
 * J1 -> J2 -> ALIGNMENT -> J3 -> J4 -> HAND
 *
 * Masse utili:
 * BODY_J1   = massa spalla/base
 * BODY_J3   = massa arm
 * BODY_HAND = massa forearm
 */

static const bool joint_body_dependency[N_JOINT][N_BODY] =
    {
        /*        BODY_BASE BODY_J1 BODY_J2 BODY_ALIGNMENT  BODY_J3 BODY_J4 BODY_HAND */

        /* J1 */ {false, true, true, true, true, true, true},
        /* J2 */ {false, false, true, true, true, true, true},
        /* J3 */ {false, false, false, false, true, true, true},
        /* J4 */ {false, false, false, false, false, true, true}};

/* ============================================================
 * MDH transform
 * ============================================================ */

/*
 * @brief: funzione per costruire la matrice omogenea associata a una riga MDH
 * @param: float a: parametro a della convenzione MDH
 * @param: float alpha: parametro alpha della convenzione MDH, in radianti
 * @param: float d: parametro d della convenzione MDH
 * @param: float theta: parametro theta totale della convenzione MDH, in radianti
 * @param: Matrix4f T: matrice omogenea risultante
 * @return: void
 */

static Matrix4f mdh_transform(float a, float alpha, float d, float theta)
{
    float ca = cosf(alpha), sa = sinf(alpha);
    float ct = cosf(theta), st = sinf(theta);

    Matrix4f T;

    T << ct, -st, 0.0f, a,
        st * ca, ct * ca, -sa, -d * sa,
        st * sa, ct * sa, ca, d * ca,
        0.0f, 0.0f, 0.0f, 1.0f;

    return T;
}

/* ============================================================
 * Forward kinematics from MDH
 * ============================================================ */

/*
 * @brief: funzione per calcolare le trasformazioni world-body usando la catena MDH
 * @param: const float q_model[N_JOINT]: angoli dei giunti nelle convenzioni del modello, in radianti
 * @param: Matrix4f T_world_body[N_BODY]: trasformazioni omogenee dei body rispetto al frame world
 * @return: void
 */

static void mdh_forward_kinematics(const float q_model[N_JOINT], Matrix4f T_world_body[N_BODY])
{
    Matrix4f T_current = Matrix4f::Identity();

    for (int i = 0; i < N_BODY; ++i)
    {
        T_world_body[i] = Matrix4f::Identity();
    }

    T_world_body[BODY_BASE] = T_current;

    for (int i = 0; i < N_MDH; ++i)
    {
        float theta = mdh_table[i].theta;

        if (mdh_table[i].joint_id != MDH_FIXED)
        {
            theta += q_model[mdh_table[i].joint_id];
        }

        Matrix4f T_step = mdh_transform(mdh_table[i].a, mdh_table[i].alpha, mdh_table[i].d, theta);

        T_current = T_current * T_step;

        T_world_body[mdh_body_id[i]] = T_current;
    }
}

static const int joint_body_frame[N_JOINT] =
    {
        BODY_J1,
        BODY_J2,
        BODY_J3,
        BODY_J4};

/* ============================================================
 * Human-Robot Dynamic Coupling
 * ============================================================ */

int antig_init_human_coupling(float alpha_assistance)
{
    // 1. Recupero dei parametri (peso e altezza)
    human_params_t pz = antig_get_parameters();

    // Controllo di sicurezza
    if (pz.patient_weight_kg <= 0.0f || pz.patient_height_m <= 0.0f)
    {
        return -1;
    }

    // Questo previene l'accumulo infinito di masse se la funzione viene chiamata più volte.
    for (int i = 0; i < N_BODY; ++i)
    {
        body_mass[i] = robot_mass_nominal[i];
        body_com[i] = robot_com_nominal[i];
    }

    // Se si richiede assistenza nulla, Fine.
    if (alpha_assistance <= 0.0f)
    {
        return 0;
    }

    // 3. STIMA ANTROPOMETRICA
    // Masse (Coefficienti di Winter)
    float m_human_arm = pz.patient_weight_kg * 0.028f;
    float m_human_forearm = pz.patient_weight_kg * 0.022f;

    // Lunghezze stimate (Coefficienti di Drillis & Contini)
    float l_human_arm = pz.patient_height_m * 0.186f;
    float l_human_forearm = pz.patient_height_m * 0.146f;

    // Centri di massa locali dell'arto umano (lungo asse Z negativo)
    // Moltiplicatori prossimali di Winter
    Vector3f p_com_human_arm(0.0f, 0.0f, -(l_human_arm * 0.564f));
    Vector3f p_com_human_forearm(0.0f, 0.0f, -(l_human_forearm * 0.570f));

    // 4. COMPOSIZIONE DEI CORPI RIGIDI

    // --> BODY_J3 (Braccio)
    float m_eq_arm = robot_mass_nominal[BODY_J3] + (alpha_assistance * m_human_arm);
    Vector3f p_com_eq_arm = ((robot_mass_nominal[BODY_J3] * robot_com_nominal[BODY_J3]) + (alpha_assistance * m_human_arm * p_com_human_arm)) / m_eq_arm;

    // --> BODY_HAND (Avambraccio)
    float m_eq_forearm = robot_mass_nominal[BODY_HAND] + (alpha_assistance * m_human_forearm);
    Vector3f p_com_eq_forearm = ((robot_mass_nominal[BODY_HAND] * robot_com_nominal[BODY_HAND]) + (alpha_assistance * m_human_forearm * p_com_human_forearm)) / m_eq_forearm;

    // 5. AGGIORNAMENTO DEL MODELLO
    body_mass[BODY_J3] = m_eq_arm;
    body_com[BODY_J3] = p_com_eq_arm;

    body_mass[BODY_HAND] = m_eq_forearm;
    body_com[BODY_HAND] = p_com_eq_forearm;

    return 0; // Inizializzazione completata con successo
}

/* ============================================================
 * Gravity torque from moments
 * ============================================================
 * F = m * g

 * r = p_com_world - p_joint_world

 * M = r x F

 * tau_j = axis_j dot M */

/*
 * @brief: calcola le coppie gravitazionali usando i momenti delle forze peso
 * @param: const float q_model[N_JOINT]: angoli dei giunti nelle convenzioni del modello [rad]
 * @param: float tau_gravity_Nm[N_JOINT]: coppie gravitazionali calcolate [Nm]
 * @return: void
 */
static void compute_gravity_torque(const float q_model[N_JOINT], float tau_gravity_Nm[N_JOINT])
{
    Matrix4f T_world_body[N_BODY];
    mdh_forward_kinematics(q_model, T_world_body);

    for (int j = 0; j < N_JOINT; ++j)
        tau_gravity_Nm[j] = 0.0f;

    for (int j = 0; j < N_JOINT; ++j)
    {
        int joint_frame = joint_body_frame[j];

        Vector3f p_joint = T_world_body[joint_frame].block<3, 1>(0, 3); // Colonna origine (indice 3)
        Vector3f z_joint = T_world_body[joint_frame].block<3, 1>(0, 2); // Colonna asse Z (indice 2)

        for (int b = 0; b < N_BODY; ++b)
        {
            if (!joint_body_dependency[j][b] || body_mass[b] == 0.0f)
                continue;

            // Trasforma il centro di massa (3D) in coordinate omogenee (4D)
            Vector4f p_com_local(body_com[b].x(), body_com[b].y(), body_com[b].z(), 1.0f);

            // Moltiplica la matrice per il vettore e riprendi solo le prime 3 componenti (x, y, z)
            Vector3f p_com = (T_world_body[b] * p_com_local).head<3>();

            Vector3f r = p_com - p_joint;
            Vector3f F = gravity_world * body_mass[b];
            Vector3f M = r.cross(F);

            tau_gravity_Nm[j] += z_joint.dot(M);
        }
    }
}

/* ============================================================
 * Encoder to model conversion -->TODO verificare se cambia qualcosa tra destro e sinistro
 * ============================================================ */

static const float q_sign[N_JOINT] =
    {
        1.0f,  // J1
        1.0f,  // J2
        -1.0f, // J3
        1.0f   // J4
};

static const float q_offset[N_JOINT] =
    {
        -M_PI_2, // J1
        0.0f,    // J2
        0.0f,    // J3
        0.0f     // J4
};

/*
 * @brief: converte gli angoli encoder negli angoli usati dal modello
 * @param: q_encoder: angoli letti dagli encoder [rad]
 * @param: q_model: angoli convertiti nella convenzione del modello [rad]
 * @return: void
 */
static void encoder_to_model(const float q_encoder[N_JOINT], float q_model[N_JOINT])
{
    for (int i = 0; i < N_JOINT; ++i)
    {
        q_model[i] = q_sign[i] * q_encoder[i] + q_offset[i];
    }
}

/* ====================================
    FUNZIONE FINALE
   ====================================*/

/*
 * @brief: calcola le coppie antiG in Nm a partire dalle posizioni encoder
 * @param: const float pos_rad[N_JOINT]: posizioni correnti degli encoder [rad]
 * @param: antig_output_t *out: output del modello antiG
 * @return: int: 0 se ok, valore negativo in caso di errore
 */
int antig_compute_from_pos(
    const float pos_rad[N_JOINT],
    antig_output_t *out)
{
    if ((pos_rad == NULL) || (out == NULL))
    {
        return -1;
    }

    memset(out, 0, sizeof(*out));

    encoder_to_model(pos_rad, out->q_model);

    compute_gravity_torque(
        out->q_model,
        out->tau_gravity_Nm);

    return 0;
}

/* ============================================================
 * Human Subject Parameters (Global Encapsulated State)
 * ============================================================ */

static human_params_t current_human_params = {0.0f, 0.0f};

void antig_set_parameters(uint16_t weight_tcp, uint16_t height_tcp)
{
    // Salvataggio e conversione

    current_human_params.patient_weight_kg = (float)weight_tcp / 10.0f;
    current_human_params.patient_height_m = (float)height_tcp / 100.0f;
}

human_params_t antig_get_parameters(void)
{
    // Restituisce una copia pulita e sicura della struttura
    return current_human_params;
}