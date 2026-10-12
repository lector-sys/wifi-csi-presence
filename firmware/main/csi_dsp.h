/*
 * csi_dsp.h - CSI -> activity score pipeline. Pure C, no ESP-IDF deps,
 * so it compiles and runs on a host (see ../test/test_dsp.c).
 *
 * Pipeline adapted from:
 *   espectre (francescopace) - MVS: spatial turbulence per packet, outlier
 *      rejection, then moving variance over a window as the motion metric.
 *      CV normalisation (std/mean) is used here because a plain ESP32 cannot
 *      lock AGC/FFT gain (espectre's gain_controller is S3/C3/C5/C6 only),
 *      so raw amplitude drifts with gain and only the ratio is stable.
 *   WiFi-Sense (Saad-Imtiaz) - EMA smoothing of the motion level.
 */
#ifndef CSI_DSP_H
#define CSI_DSP_H

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#define DSP_MEDIAN_W      7     /* impulse (AGC step) rejection */
#define DSP_VAR_W         100   /* moving-variance window ~1 s at 100 pkt/s */
#define DSP_EVAL_EVERY    10    /* recompute moving variance every N packets */
#define DSP_HIST_W        60    /* pattern window: 60 publishes @2 Hz = 30 s */
#define DSP_SPAN_DECADES  2.0f  /* baseline .. baseline*100 maps to score 0..100 */
#define DSP_ACTIVE_SCORE  15    /* score above this counts as "moving" */
#define DSP_EMA_ALPHA     0.15f

/* Rough guess for an uncalibrated quiet room. Only used until the user runs
 * calibration; every room differs by more than this is worth arguing about.
 * ponytail: single global default, per-room value comes from calibration. */
#define DSP_DEFAULT_BASELINE 2.0e-5f

/* LLTF 20 MHz layout in the ESP32 CSI buffer: index 0 = DC, 1..26 =
 * subcarriers +1..+26, 27..37 = guard (always zero), 38..63 = -26..-1.
 * Start at 2 so we are never inside the "first word invalid" 4 bytes. */
#define DSP_SC_LO_START 2
#define DSP_SC_LO_END   26
#define DSP_SC_HI_START 38
#define DSP_SC_HI_END   63

typedef enum { DSP_QUIET = 0, DSP_BRIEF = 1, DSP_SUSTAINED = 2 } dsp_pattern_t;

typedef struct {
    float    med_buf[DSP_MEDIAN_W];
    int      med_n;
    float    var_buf[DSP_VAR_W];
    int      var_i, var_n;

    float    baseline;      /* quiet-room moving variance */
    bool     calibrated;
    float    score_ema;
    float    last_mv;

    bool     calibrating;
    double   cal_sum, cal_sum2;
    uint32_t cal_n;

    uint8_t  hist[DSP_HIST_W];
    int      hist_i, hist_n;

    uint32_t packets;
    uint32_t since_eval;
} csi_dsp_t;

static inline void csi_dsp_init(csi_dsp_t *d, float baseline)
{
    memset(d, 0, sizeof(*d));
    d->calibrated = (baseline > 0.0f);
    d->baseline   = d->calibrated ? baseline : DSP_DEFAULT_BASELINE;
}

/* --- helpers ------------------------------------------------------------ */

static inline float dsp_median(const float *src, int n)
{
    float s[DSP_MEDIAN_W];
    memcpy(s, src, n * sizeof(float));
    for (int i = 1; i < n; i++) {           /* insertion sort, n <= 7 */
        float v = s[i]; int j = i - 1;
        while (j >= 0 && s[j] > v) { s[j + 1] = s[j]; j--; }
        s[j + 1] = v;
    }
    return s[n / 2];
}

static inline float dsp_variance(const float *v, int n)
{
    if (n < 2) return 0.0f;
    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += v[i];
    mean /= n;
    float var = 0.0f;
    for (int i = 0; i < n; i++) { float dv = v[i] - mean; var += dv * dv; }
    return var / n;
}

/* Spatial turbulence: coefficient of variation of subcarrier amplitude
 * within one packet. Gain-invariant, which matters on plain ESP32. */
static inline float dsp_turbulence(const int8_t *buf, int len)
{
    float amp[64];
    int   n = 0;
    int   pairs = len / 2;

    for (int i = DSP_SC_LO_START; i <= DSP_SC_HI_END && i < pairs; i++) {
        if (i > DSP_SC_LO_END && i < DSP_SC_HI_START) continue;   /* guard band */
        float q = (float) buf[i * 2];          /* Espressif order: [imag, real] */
        float r = (float) buf[i * 2 + 1];
        amp[n++] = sqrtf(q * q + r * r);
    }
    if (n < 8) return 0.0f;

    float mean = 0.0f;
    for (int i = 0; i < n; i++) mean += amp[i];
    mean /= n;
    if (mean < 1.0f) return 0.0f;              /* dead/clipped frame */

    return sqrtf(dsp_variance(amp, n)) / mean;
}

/* --- main entry points -------------------------------------------------- */

/* Feed one CSI frame. buf/len are the raw I/Q bytes from wifi_csi_info_t. */
static inline void csi_dsp_push(csi_dsp_t *d, const int8_t *buf, int len)
{
    float t = dsp_turbulence(buf, len);
    if (t <= 0.0f) return;
    d->packets++;

    /* median-of-7 kills single-packet AGC spikes (espectre uses Hampel;
     * ponytail: median is shorter and enough here, swap in Hampel/MAD if
     * spike rejection proves too blunt) */
    if (d->med_n < DSP_MEDIAN_W) {
        d->med_buf[d->med_n++] = t;
        if (d->med_n < DSP_MEDIAN_W) return;
    } else {
        memmove(d->med_buf, d->med_buf + 1, (DSP_MEDIAN_W - 1) * sizeof(float));
        d->med_buf[DSP_MEDIAN_W - 1] = t;
    }
    float tf = dsp_median(d->med_buf, DSP_MEDIAN_W);

    d->var_buf[d->var_i] = tf;
    d->var_i = (d->var_i + 1) % DSP_VAR_W;
    if (d->var_n < DSP_VAR_W) d->var_n++;

    if (d->var_n < DSP_VAR_W) return;
    if (++d->since_eval < DSP_EVAL_EVERY) return;
    d->since_eval = 0;

    float mv = dsp_variance(d->var_buf, DSP_VAR_W);
    d->last_mv = mv;

    if (d->calibrating) {
        d->cal_sum  += mv;
        d->cal_sum2 += (double) mv * mv;
        d->cal_n++;
    }

    float ratio = mv / (d->baseline > 0.0f ? d->baseline : DSP_DEFAULT_BASELINE);
    float score = (ratio > 1.0f)
                ? 100.0f * log10f(ratio) / DSP_SPAN_DECADES
                : 0.0f;
    if (score > 100.0f) score = 100.0f;

    d->score_ema += DSP_EMA_ALPHA * (score - d->score_ema);
}

static inline int csi_dsp_score(const csi_dsp_t *d)
{
    int s = (int) (d->score_ema + 0.5f);
    return s < 0 ? 0 : (s > 100 ? 100 : s);
}

/* Call at publish rate (2 Hz). Records the score for the pattern heuristic. */
static inline void csi_dsp_publish(csi_dsp_t *d)
{
    d->hist[d->hist_i] = (uint8_t) csi_dsp_score(d);
    d->hist_i = (d->hist_i + 1) % DSP_HIST_W;
    if (d->hist_n < DSP_HIST_W) d->hist_n++;
}

/* Soft hint only: how much of the recent window was above the activity
 * threshold. Says nothing about what caused it. */
static inline dsp_pattern_t csi_dsp_pattern(const csi_dsp_t *d)
{
    if (d->hist_n < 10) return DSP_QUIET;
    int active = 0;
    for (int i = 0; i < d->hist_n; i++)
        if (d->hist[i] > DSP_ACTIVE_SCORE) active++;
    float frac = (float) active / d->hist_n;
    if (frac < 0.05f) return DSP_QUIET;
    if (frac > 0.50f) return DSP_SUSTAINED;
    return DSP_BRIEF;
}

static inline const char *csi_dsp_pattern_name(dsp_pattern_t p)
{
    return p == DSP_SUSTAINED ? "sustained" : (p == DSP_BRIEF ? "brief" : "quiet");
}

/* --- calibration -------------------------------------------------------- */

static inline void csi_dsp_cal_start(csi_dsp_t *d)
{
    d->calibrating = true;
    d->cal_sum = d->cal_sum2 = 0.0;
    d->cal_n = 0;
}

/* Baseline = mean + 3*sd of quiet-room moving variance. Deliberately
 * generous: a single-node link is noisy, and a baseline set too low turns
 * every AGC wobble into "activity". Returns the new baseline, 0 on failure. */
static inline float csi_dsp_cal_finish(csi_dsp_t *d)
{
    d->calibrating = false;
    if (d->cal_n < 30) return 0.0f;

    double mean = d->cal_sum / d->cal_n;
    double var  = d->cal_sum2 / d->cal_n - mean * mean;
    if (var < 0.0) var = 0.0;
    float base = (float) (mean + 3.0 * sqrt(var));
    if (!(base > 0.0f)) return 0.0f;

    d->baseline   = base;
    d->calibrated = true;
    d->score_ema  = 0.0f;
    return base;
}

#endif /* CSI_DSP_H */
