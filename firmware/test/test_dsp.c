/* Host check for csi_dsp.h.  Build & run:  gcc -O2 -o t test_dsp.c -lm && ./t
 * Feeds synthetic CSI frames and asserts the score separates quiet from moving. */
#include "../main/csi_dsp.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

static unsigned seed = 1;
static float urand(void) { seed = seed * 1103515245 + 12345; return ((seed >> 16) & 0x7fff) / 32767.0f; }

/* One synthetic LLTF frame: a fixed per-subcarrier profile plus noise.
 * `wobble` shifts the profile shape, which is what a moving body does. */
static void frame(int8_t *buf, float noise, float wobble)
{
    for (int i = 0; i < 64; i++) {
        float base = 30.0f + 12.0f * sinf(i * 0.4f + wobble);
        float amp  = base + noise * (urand() - 0.5f);
        buf[i * 2]     = (int8_t) (amp * 0.6f);
        buf[i * 2 + 1] = (int8_t) (amp * 0.8f);
    }
}

static void feed(csi_dsp_t *d, int n, float noise, float wobble_rate)
{
    int8_t buf[128];
    static float w = 0.0f;
    for (int i = 0; i < n; i++) {
        w += wobble_rate;
        frame(buf, noise, w);
        csi_dsp_push(d, buf, 128);
        if (i % 50 == 0) csi_dsp_publish(d);
    }
}

int main(void)
{
    csi_dsp_t d;

    /* 1. calibrate on a still room, then confirm still stays near zero */
    csi_dsp_init(&d, 0.0f);
    assert(!d.calibrated);
    csi_dsp_cal_start(&d);
    feed(&d, 6000, 1.0f, 0.0f);
    float base = csi_dsp_cal_finish(&d);
    printf("baseline      = %.3e\n", base);
    assert(base > 0.0f && d.calibrated);

    feed(&d, 2000, 1.0f, 0.0f);
    int quiet = csi_dsp_score(&d);
    printf("quiet score   = %d\n", quiet);
    assert(quiet < 15);
    assert(csi_dsp_pattern(&d) == DSP_QUIET);

    /* 2. a moving scatterer must push the score up */
    feed(&d, 4000, 1.0f, 0.05f);
    int moving = csi_dsp_score(&d);
    printf("moving score  = %d\n", moving);
    assert(moving > quiet + 20);

    /* 3. sustained movement must read as "sustained" */
    for (int i = 0; i < 60; i++) { feed(&d, 100, 1.0f, 0.05f); csi_dsp_publish(&d); }
    printf("pattern       = %s\n", csi_dsp_pattern_name(csi_dsp_pattern(&d)));
    assert(csi_dsp_pattern(&d) == DSP_SUSTAINED);

    /* 4. score must return to quiet once movement stops */
    for (int i = 0; i < 60; i++) { feed(&d, 100, 1.0f, 0.0f); csi_dsp_publish(&d); }
    int after = csi_dsp_score(&d);
    printf("settled score = %d\n", after);
    assert(after < 15);

    /* 5. calibration refuses to commit on too few samples */
    csi_dsp_t e;
    csi_dsp_init(&e, 0.0f);
    csi_dsp_cal_start(&e);
    feed(&e, 200, 1.0f, 0.0f);
    assert(csi_dsp_cal_finish(&e) == 0.0f);

    printf("OK\n");
    return 0;
}
