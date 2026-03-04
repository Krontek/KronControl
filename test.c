/*===========================================================================
 * KronControl — Tests
 * Build: tcc test.c kroncontrol.c -o test_app && ./test_app
 *===========================================================================*/

#include "kroncontrol.h"
#include <stdio.h>

static int pass_count = 0;
static int fail_count = 0;

static void check(const char *name, int condition)
{
    if (condition) {
        pass_count++;
    } else {
        fail_count++;
        printf("FAIL: %s\n", name);
    }
}

/* Approximate float equality (1% tolerance) */
static int feq(float a, float b)
{
    float diff = a - b;
    if (diff < 0.0f) diff = -diff;
    float ref  = (a < 0.0f ? -a : a);
    if (ref < 1e-6f) return diff < 1e-5f;
    return diff / ref < 0.01f;
}

int main(void)
{
    /* -----------------------------------------------------------------------
     * PID
     * --------------------------------------------------------------------- */

    /* Disabled → Out = 0 */
    {
        PID p = {0};
        p.Kp = 1.0f; p.Ki = 1.0f; p.OutMin = -100.0f; p.OutMax = 100.0f;
        PID_Call(&p, false, 10.0f, 0.0f, 0.1f);
        check("PID: Enable=false → Out=0", p.Out == 0.0f);
    }

    /* P-only: Kp=1, error=10 → Out=10 */
    {
        PID p = {0};
        p.Kp = 1.0f; p.OutMin = -100.0f; p.OutMax = 100.0f;
        PID_Call(&p, true, 10.0f, 0.0f, 0.1f);
        check("PID: P-only, Kp=1, error=10 → Out=10", feq(p.Out, 10.0f));
    }

    /* I-only: Ki=1, error=5, Dt=0.2 → integral = 1.0 after 1 call */
    {
        PID p = {0};
        p.Ki = 1.0f; p.OutMin = -100.0f; p.OutMax = 100.0f;
        PID_Call(&p, true, 5.0f, 0.0f, 0.2f);
        check("PID: I-only, 1st call → Out=1.0", feq(p.Out, 1.0f));
        PID_Call(&p, true, 5.0f, 0.0f, 0.2f);
        check("PID: I-only, 2nd call → Out=2.0", feq(p.Out, 2.0f));
    }

    /* Output clamping: Kp=100, error=5 → raw=500, clamped to OutMax=50 */
    {
        PID p = {0};
        p.Kp = 100.0f; p.OutMin = -50.0f; p.OutMax = 50.0f;
        PID_Call(&p, true, 5.0f, 0.0f, 0.1f);
        check("PID: output clamped to OutMax=50", feq(p.Out, 50.0f));
    }

    /* Anti-windup: integral should not grow beyond clamp range */
    {
        PID p = {0};
        p.Ki = 1.0f; p.OutMin = -10.0f; p.OutMax = 10.0f;
        int i;
        for (i = 0; i < 200; i++) PID_Call(&p, true, 5.0f, 0.0f, 0.1f);
        check("PID: anti-windup → Out clamped at OutMax", feq(p.Out, 10.0f));
        check("PID: integral not unbounded", p._integral <= 10.0f);
    }

    /* Bumpless enable: rising edge seeds prevFeedback → no derivative spike */
    {
        PID p = {0};
        p.Kd = 1.0f; p.DerivFilterCoeff = 100.0f;
        p.OutMin = -1000.0f; p.OutMax = 1000.0f;
        PID_Call(&p, false, 0.0f, 50.0f, 0.1f); /* disabled with Feedback=50 */
        PID_Call(&p, true,  0.0f, 50.0f, 0.1f); /* enable: prevFeedback seeded to 50 */
        check("PID: bumpless enable, D≈0 on rising edge", feq(p._derivFiltered, 0.0f));
    }

    /* Dt <= 0 → no update */
    {
        PID p = {0};
        p.Kp = 1.0f; p.OutMin = -100.0f; p.OutMax = 100.0f;
        PID_Call(&p, true, 10.0f, 0.0f, 0.0f);
        check("PID: Dt=0 → Out=0", p.Out == 0.0f);
    }

    /* -----------------------------------------------------------------------
     * LOW_PASS
     * --------------------------------------------------------------------- */

    /* Alpha=1 → instant tracking */
    {
        LOW_PASS lp = {0};
        lp.Alpha = 1.0f;
        LOW_PASS_Call(&lp, 42.0f);
        check("LP: Alpha=1 → Out==In", feq(lp.Out, 42.0f));
    }

    /* Alpha=0.5 → halfway each step */
    {
        LOW_PASS lp = {0};
        lp.Alpha = 0.5f;
        LOW_PASS_Call(&lp, 10.0f);
        check("LP: Alpha=0.5, In=10 → Out=5", feq(lp.Out, 5.0f));
        LOW_PASS_Call(&lp, 10.0f);
        check("LP: Alpha=0.5, 2nd call → Out=7.5", feq(lp.Out, 7.5f));
    }

    /* Alpha=0 → output frozen at initial value */
    {
        LOW_PASS lp = {0};
        lp.Alpha = 0.0f;
        LOW_PASS_Call(&lp, 99.0f);
        check("LP: Alpha=0 → Out stays at 0", feq(lp.Out, 0.0f));
    }

    /* KRON_LP_Alpha helper: very low cutoff → alpha ≈ 0 */
    {
        float a = KRON_LP_Alpha(1.0f, 0.001f); /* fc=1Hz, Ts=1ms */
        check("LP_Alpha: fc=1Hz, Ts=1ms → alpha < 0.01", a < 0.01f);
    }

    /* -----------------------------------------------------------------------
     * HIGH_PASS
     * --------------------------------------------------------------------- */

    /* Constant input → steady-state Out → 0 (DC blocked) */
    {
        HIGH_PASS hp = {0};
        hp.Alpha = 0.9f;
        int i;
        for (i = 0; i < 500; i++) HIGH_PASS_Call(&hp, 5.0f);
        check("HP: constant input → Out ≈ 0 (DC blocked)", feq(hp.Out, 0.0f));
    }

    /* Alpha=1 → full pass-through of step */
    {
        HIGH_PASS hp = {0};
        hp.Alpha = 1.0f;
        HIGH_PASS_Call(&hp, 0.0f); /* seed */
        HIGH_PASS_Call(&hp, 10.0f);
        check("HP: Alpha=1, step of 10 → Out=10", feq(hp.Out, 10.0f));
    }

    /* KRON_HP_Alpha helper: very high cutoff → alpha ≈ 1 */
    {
        float a = KRON_HP_Alpha(1000.0f, 0.001f); /* fc=1kHz, Ts=1ms */
        check("HP_Alpha: fc=1kHz, Ts=1ms → alpha < 0.5", a < 0.5f);
    }

    /* -----------------------------------------------------------------------
     * RATE_LIMITER
     * --------------------------------------------------------------------- */

    /* Rise limited: In=100, RiseRate=10, Dt=0.1 → Out=1.0 */
    {
        RATE_LIMITER rl = {0};
        rl.RiseRate = 10.0f; rl.FallRate = 10.0f;
        RATE_LIMITER_Call(&rl, 100.0f, 0.1f);
        check("RL: rise limited → Out=1.0", feq(rl.Out, 1.0f));
    }

    /* Fall limited: Out starts at 10, In=-100, FallRate=20, Dt=0.1 → Out=10-2=-... */
    {
        RATE_LIMITER rl = {0};
        rl.Out = 10.0f;
        rl.RiseRate = 20.0f; rl.FallRate = 20.0f;
        RATE_LIMITER_Call(&rl, -100.0f, 0.1f);
        check("RL: fall limited → Out=10-2=8", feq(rl.Out, 8.0f));
    }

    /* Within limit: In reachable in one step */
    {
        RATE_LIMITER rl = {0};
        rl.RiseRate = 100.0f; rl.FallRate = 100.0f;
        RATE_LIMITER_Call(&rl, 5.0f, 0.1f);
        check("RL: within limit → Out==In", feq(rl.Out, 5.0f));
    }

    /* Asymmetric: different rise and fall rates */
    {
        RATE_LIMITER rl = {0};
        rl.Out = 0.0f;
        rl.RiseRate = 5.0f; rl.FallRate = 20.0f;
        RATE_LIMITER_Call(&rl, 100.0f, 1.0f);
        check("RL: asymmetric rise → Out=5", feq(rl.Out, 5.0f));
        RATE_LIMITER_Call(&rl, -100.0f, 1.0f);
        check("RL: asymmetric fall → Out=5-20=-15", feq(rl.Out, -15.0f));
    }

    /* -----------------------------------------------------------------------
     * DEADBAND
     * --------------------------------------------------------------------- */

    check("DB: In within band → 0",   DEADBAND_Call(3.0f, 5.0f) == 0.0f);
    check("DB: In on edge → 0",       DEADBAND_Call(5.0f, 5.0f) == 0.0f);
    check("DB: In above band → In",   feq(DEADBAND_Call(8.0f,  5.0f),  8.0f));
    check("DB: In below -band → In",  feq(DEADBAND_Call(-8.0f, 5.0f), -8.0f));
    check("DB: In=0 → 0",             DEADBAND_Call(0.0f, 5.0f) == 0.0f);
    check("DB: Width=0 → pass-through", feq(DEADBAND_Call(3.0f, 0.0f), 3.0f));

    /* -----------------------------------------------------------------------
     * HYSTERESIS
     * --------------------------------------------------------------------- */

    {
        HYSTERESIS h = {0};
        h.HiThreshold = 5.0f;
        h.LoThreshold = 3.0f;

        /* Below Hi: stays false */
        HYSTERESIS_Call(&h, 4.0f);
        check("HYS: below Hi → Q=false", h.Q == false);

        /* Above Hi: switches true */
        HYSTERESIS_Call(&h, 6.0f);
        check("HYS: above Hi → Q=true", h.Q == true);

        /* Between Lo and Hi: stays true */
        HYSTERESIS_Call(&h, 4.0f);
        check("HYS: between thresholds → Q stays true", h.Q == true);

        /* Below Lo: switches false */
        HYSTERESIS_Call(&h, 2.0f);
        check("HYS: below Lo → Q=false", h.Q == false);

        /* Between Lo and Hi again: stays false */
        HYSTERESIS_Call(&h, 4.0f);
        check("HYS: between thresholds → Q stays false", h.Q == false);
    }

    /* -----------------------------------------------------------------------
     * MOVING_AVG
     * --------------------------------------------------------------------- */

    /* N=4: fill with {2, 4, 6, 8} → mean = 5.0 */
    {
        MOVING_AVG ma = {0};
        ma.N = 4;
        MOVING_AVG_Call(&ma, 2.0f);
        MOVING_AVG_Call(&ma, 4.0f);
        MOVING_AVG_Call(&ma, 6.0f);
        MOVING_AVG_Call(&ma, 8.0f);
        check("MA: N=4, sum={2,4,6,8} → Out=5.0", feq(ma.Out, 5.0f));
    }

    /* Sliding window: 5th sample evicts 1st */
    {
        MOVING_AVG ma = {0};
        ma.N = 4;
        MOVING_AVG_Call(&ma, 2.0f);
        MOVING_AVG_Call(&ma, 4.0f);
        MOVING_AVG_Call(&ma, 6.0f);
        MOVING_AVG_Call(&ma, 8.0f);
        MOVING_AVG_Call(&ma, 10.0f); /* evicts 2 → {4,6,8,10} mean=7.0 */
        check("MA: 5th sample → Out=7.0", feq(ma.Out, 7.0f));
    }

    /* Partial fill: 2 samples into N=4 window */
    {
        MOVING_AVG ma = {0};
        ma.N = 4;
        MOVING_AVG_Call(&ma, 3.0f);
        MOVING_AVG_Call(&ma, 7.0f);
        check("MA: partial fill 2/4 → Out=5.0", feq(ma.Out, 5.0f));
    }

    /* N=1 → Out == last In */
    {
        MOVING_AVG ma = {0};
        ma.N = 1;
        MOVING_AVG_Call(&ma, 42.0f);
        MOVING_AVG_Call(&ma, 99.0f);
        check("MA: N=1 → Out==last In", feq(ma.Out, 99.0f));
    }

    /* -----------------------------------------------------------------------
     * INTEGRATOR
     * --------------------------------------------------------------------- */

    /* Basic: In=10, Dt=0.1 → Out=1.0 per call */
    {
        INTEGRATOR intg = {0};
        intg.OutMin = -100.0f; intg.OutMax = 100.0f;
        INTEGRATOR_Call(&intg, true, false, 10.0f, 0.1f);
        check("INTG: 1st call → Out=1.0", feq(intg.Out, 1.0f));
        INTEGRATOR_Call(&intg, true, false, 10.0f, 0.1f);
        check("INTG: 2nd call → Out=2.0", feq(intg.Out, 2.0f));
    }

    /* Reset */
    {
        INTEGRATOR intg = {0};
        intg.OutMin = -100.0f; intg.OutMax = 100.0f;
        intg.Out = 55.0f;
        INTEGRATOR_Call(&intg, true, true, 10.0f, 0.1f);
        check("INTG: Reset=true → Out=0", intg.Out == 0.0f);
    }

    /* Clamp at OutMax */
    {
        INTEGRATOR intg = {0};
        intg.OutMin = -5.0f; intg.OutMax = 5.0f;
        int i;
        for (i = 0; i < 100; i++) INTEGRATOR_Call(&intg, true, false, 10.0f, 0.1f);
        check("INTG: clamped at OutMax=5", feq(intg.Out, 5.0f));
    }

    /* Enable=false → hold */
    {
        INTEGRATOR intg = {0};
        intg.OutMin = -100.0f; intg.OutMax = 100.0f;
        intg.Out = 3.0f;
        INTEGRATOR_Call(&intg, false, false, 10.0f, 0.1f);
        check("INTG: Enable=false → hold", feq(intg.Out, 3.0f));
    }

    /* Negative integration */
    {
        INTEGRATOR intg = {0};
        intg.OutMin = -100.0f; intg.OutMax = 100.0f;
        INTEGRATOR_Call(&intg, true, false, -5.0f, 0.2f);
        check("INTG: negative In → Out=-1.0", feq(intg.Out, -1.0f));
    }

    /* -----------------------------------------------------------------------
     * DIFFERENTIATOR
     * --------------------------------------------------------------------- */

    /* First call: no spike */
    {
        DIFFERENTIATOR d = {0};
        d.Alpha = 1.0f;
        DIFFERENTIATOR_Call(&d, 100.0f, 0.1f);
        check("DIFF: 1st call → Out=0 (no spike)", d.Out == 0.0f);
    }

    /* Alpha=1, step of 10 in Dt=0.1 → Out = 100 */
    {
        DIFFERENTIATOR d = {0};
        d.Alpha = 1.0f;
        DIFFERENTIATOR_Call(&d, 0.0f, 0.1f);  /* seed */
        DIFFERENTIATOR_Call(&d, 10.0f, 0.1f);
        check("DIFF: Alpha=1, step 10 in Dt=0.1 → Out=100", feq(d.Out, 100.0f));
    }

    /* Alpha=0.5: output is half the raw derivative */
    {
        DIFFERENTIATOR d = {0};
        d.Alpha = 0.5f;
        DIFFERENTIATOR_Call(&d, 0.0f, 0.1f);  /* seed */
        DIFFERENTIATOR_Call(&d, 10.0f, 0.1f); /* rawD=100, Out=0.5*100=50 */
        check("DIFF: Alpha=0.5, 1st step → Out=50", feq(d.Out, 50.0f));
    }

    /* Constant input → derivative goes to 0 */
    {
        DIFFERENTIATOR d = {0};
        d.Alpha = 0.5f;
        DIFFERENTIATOR_Call(&d, 5.0f, 0.1f);
        int i;
        for (i = 0; i < 50; i++) DIFFERENTIATOR_Call(&d, 5.0f, 0.1f);
        check("DIFF: constant input → Out ≈ 0", feq(d.Out, 0.0f));
    }

    /* Dt=0 → no update */
    {
        DIFFERENTIATOR d = {0};
        d.Alpha = 1.0f;
        DIFFERENTIATOR_Call(&d, 0.0f, 0.1f);  /* seed */
        d.Out = 42.0f;
        DIFFERENTIATOR_Call(&d, 10.0f, 0.0f); /* Dt=0, no update */
        check("DIFF: Dt=0 → Out held", feq(d.Out, 42.0f));
    }

    /* -----------------------------------------------------------------------
     * Summary
     * --------------------------------------------------------------------- */
    printf("%d passed, %d failed\n", pass_count, fail_count);
    return fail_count == 0 ? 0 : 1;
}
