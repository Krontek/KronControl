/*===========================================================================
 * KronControl — Advanced Control Function Block Implementations
 *===========================================================================*/

#include "kroncontrol.h"

/* 2π constant — no external math dependency */
#define KRON_TWO_PI 6.28318530f

/*===========================================================================
 * PID
 *===========================================================================*/
void PID_Call(PID *inst, bool Enable, float Setpoint, float Feedback, float Dt)
{
    /* Detect rising edge of Enable for bumpless start */
    bool risingEdge = Enable && !inst->_prevEnable;
    inst->_prevEnable = Enable;

    if (!Enable || Dt <= 0.0f) {
        inst->_integral      = 0.0f;
        inst->_derivFiltered = 0.0f;
        inst->Out            = 0.0f;
        return;
    }

    /* On rising edge: seed prevFeedback so first derivative = 0 */
    if (risingEdge) {
        inst->_prevFeedback  = Feedback;
        inst->_derivFiltered = 0.0f;
    }

    float error = Setpoint - Feedback;

    /* --- Proportional --- */
    float P = inst->Kp * error;

    /* --- Derivative on measurement (no setpoint kick) with 1st-order filter ---
     * raw D = -Kd * d(Feedback)/dt
     * filtered via: D += alpha * (rawD - D),  alpha = N*Dt/(1+N*Dt)          */
    float D = 0.0f;
    if (inst->Kd != 0.0f && inst->DerivFilterCoeff > 0.0f) {
        float rawD  = -inst->Kd * (Feedback - inst->_prevFeedback) / Dt;
        float N     = inst->DerivFilterCoeff;
        float alpha = N * Dt / (1.0f + N * Dt);
        inst->_derivFiltered += alpha * (rawD - inst->_derivFiltered);
        D = inst->_derivFiltered;
    }

    /* --- Integral with clamping anti-windup ---
     * Clamp the integral so that (P + I + D) cannot exceed [OutMin, OutMax]. */
    inst->_integral += inst->Ki * error * Dt;
    float I_max = inst->OutMax - P - D;
    float I_min = inst->OutMin - P - D;
    if (inst->_integral > I_max) inst->_integral = I_max;
    if (inst->_integral < I_min) inst->_integral = I_min;

    /* --- Output --- */
    float out = P + inst->_integral + D;
    if (out > inst->OutMax) out = inst->OutMax;
    if (out < inst->OutMin) out = inst->OutMin;

    inst->Out         = out;
    inst->_prevFeedback = Feedback;
}

/*===========================================================================
 * LOW_PASS
 *===========================================================================*/
void LOW_PASS_Call(LOW_PASS *inst, float In)
{
    inst->Out = inst->Alpha * In + (1.0f - inst->Alpha) * inst->Out;
}

float KRON_LP_Alpha(float cutoff_hz, float dt_s)
{
    /* alpha = wc*Ts / (1 + wc*Ts),  wc = 2π·fc */
    float wc_dt = KRON_TWO_PI * cutoff_hz * dt_s;
    return wc_dt / (1.0f + wc_dt);
}

/*===========================================================================
 * HIGH_PASS
 *===========================================================================*/
void HIGH_PASS_Call(HIGH_PASS *inst, float In)
{
    /* Compute difference first to avoid catastrophic cancellation.
     * When In == _prevIn, (In - _prevIn) = 0.0f exactly, so Out decays
     * cleanly by Alpha each call instead of losing precision in
     * the form (Out + In) - prevIn when Out << In. */
    inst->Out     = inst->Alpha * inst->Out + inst->Alpha * (In - inst->_prevIn);
    inst->_prevIn = In;
}

float KRON_HP_Alpha(float cutoff_hz, float dt_s)
{
    /* alpha = 1 / (1 + wc*Ts),  wc = 2π·fc */
    return 1.0f / (1.0f + KRON_TWO_PI * cutoff_hz * dt_s);
}

/*===========================================================================
 * RATE_LIMITER
 *===========================================================================*/
void RATE_LIMITER_Call(RATE_LIMITER *inst, float In, float Dt)
{
    float maxRise = inst->RiseRate * Dt;
    float maxFall = inst->FallRate * Dt;
    float diff    = In - inst->Out;

    if (diff >  maxRise) diff =  maxRise;
    if (diff < -maxFall) diff = -maxFall;

    inst->Out += diff;
}

/*===========================================================================
 * DEADBAND
 *===========================================================================*/
float DEADBAND_Call(float In, float Width)
{
    if (In >  Width) return In;
    if (In < -Width) return In;
    return 0.0f;
}

/*===========================================================================
 * HYSTERESIS
 *===========================================================================*/
void HYSTERESIS_Call(HYSTERESIS *inst, float In)
{
    if (inst->Q) {
        if (In < inst->LoThreshold) inst->Q = false;
    } else {
        if (In > inst->HiThreshold) inst->Q = true;
    }
}

/*===========================================================================
 * MOVING_AVG
 *===========================================================================*/
void MOVING_AVG_Call(MOVING_AVG *inst, float In)
{
    uint8_t n = inst->N;
    if (n < 1u)               n = 1u;
    if (n > KRON_MA_MAX_SIZE) n = KRON_MA_MAX_SIZE;

    if (inst->_count < n) {
        /* Filling phase: buffer not yet full */
        inst->_buf[inst->_head] = In;
        inst->_sum             += In;
        inst->_head             = (uint8_t)((inst->_head + 1u) % n);
        inst->_count++;
        inst->Out = inst->_sum / (float)inst->_count;
    } else {
        /* Steady state: evict oldest sample, add new */
        inst->_sum             -= inst->_buf[inst->_head];
        inst->_buf[inst->_head] = In;
        inst->_sum             += In;
        inst->_head             = (uint8_t)((inst->_head + 1u) % n);
        inst->Out               = inst->_sum / (float)n;
    }
}

/*===========================================================================
 * INTEGRATOR
 *===========================================================================*/
void INTEGRATOR_Call(INTEGRATOR *inst, bool Enable, bool Reset, float In, float Dt)
{
    if (Reset) {
        inst->Out = 0.0f;
        return;
    }
    if (!Enable || Dt <= 0.0f) return;

    inst->Out += In * Dt;
    if (inst->Out > inst->OutMax) inst->Out = inst->OutMax;
    if (inst->Out < inst->OutMin) inst->Out = inst->OutMin;
}

/*===========================================================================
 * DIFFERENTIATOR
 *===========================================================================*/
void DIFFERENTIATOR_Call(DIFFERENTIATOR *inst, float In, float Dt)
{
    /* First call: seed previous input, output zero — no spike */
    if (!inst->_initialized) {
        inst->_prevIn      = In;
        inst->Out          = 0.0f;
        inst->_initialized = true;
        return;
    }
    if (Dt <= 0.0f) return;

    float rawD = (In - inst->_prevIn) / Dt;
    inst->Out  = inst->Alpha * rawD + (1.0f - inst->Alpha) * inst->Out;
    inst->_prevIn = In;
}
