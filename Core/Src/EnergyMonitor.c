/**
 * @file    EnergyMonitor.c
 * @brief   SCT-013 + Grove AC voltage energy monitor (all app logic).
 *
 * Hardware assumed (as previously recommended):
 *   PA0 / ADC1_IN0  <- Grove AC voltage SIG  (biased ~1.65 V midrail)
 *   PA1 / ADC1_IN1  <- SCT-013 via burden + bias + MCP6002
 *   Shared GND, sensors / op-amp on 3V3
 *
 * DMA buffer order matches CubeMX ranks:
 *   Rank 1 = IN0 (voltage) , Rank 2 = IN1 (current)
 *   sample pair = [V, I]
 *
 * =============================================================================
 * CubeMX checklist
 * =============================================================================
 * Required: Scan ON, Continuous ON, 2 conversions, EOC end of sequence,
 *           DMA circular half-word, DMA IRQ enabled.
 * Strongly recommended in CubeMX: DMA Continuous Requests = Enabled
 *   (EnergyMonitor_Start also forces this so regen cannot leave DDS off.)
 * Optional later: PLL SYSCLK, TIM trigger for fixed sample rate.
 * =============================================================================
 */

#include "EnergyMonitor.h"
#include "main.h"
#include <math.h>
#include <string.h>

/* -------------------------------------------------------------------------- */
/* Calibration — tune these after measuring a known load                      */
/* -------------------------------------------------------------------------- */

/** ADC reference / VDDA (V). Measure VDDA if accuracy matters. */
#define EM_VREF_V              3.3f

/** 12-bit full scale */
#define EM_ADC_FS              4095.0f

/**
 * Voltage scale: true mains Vrms / ADC signal Vrms (after conditioning).
 * Start ~230–800 depending on Grove module; calibrate with a DMM.
 * Example: if DMM says 230 V and raw ADC rms volts = 0.29 V -> scale = 793.
 */
#define EM_VOLTAGE_SCALE       800.0f

/**
 * Current scale: amps per volt at the ADC pin (after burden + MCP6002).
 * I = Vadc / (Rburden * gain). Example: 33 Ω burden, gain 1 -> ~0.0303 A/V
 * for 1 V peak; set for YOUR burden and gain, then fine-tune with a known load.
 */
#define EM_CURRENT_SCALE       30.0f

/**
 * Number of interleaved sample pairs in the DMA buffer.
 * Buffer layout: [V0,I0, V1,I1, ...]. Must be even. Half-buffer = N/2 pairs.
 */
#define EM_PAIR_COUNT          256U

/** Ignore first few DMA halves while midrail / op-amp settle. */
#define EM_SETTLE_HALVES       4U

/* -------------------------------------------------------------------------- */
/* Internals                                                                  */
/* -------------------------------------------------------------------------- */

#define EM_BUF_LEN             (EM_PAIR_COUNT * 2U)
#define EM_HALF_PAIRS          (EM_PAIR_COUNT / 2U)

static ADC_HandleTypeDef *em_hadc;

/* DMA destination: interleaved V, I */
static uint16_t em_dma_buf[EM_BUF_LEN];

typedef enum {
  EM_HALF_NONE = 0,
  EM_HALF_FIRST,
  EM_HALF_SECOND
} EmHalf_t;

static volatile EmHalf_t em_half_ready = EM_HALF_NONE;
static uint32_t em_settle_left = EM_SETTLE_HALVES;

static EnergyMonitor_Readings_t em_readings;
static uint32_t em_last_tick_ms;
static float em_energy_ws; /* watt-seconds accumulator */

static float em_counts_to_volts(float counts)
{
  return (counts * EM_VREF_V) / EM_ADC_FS;
}

/**
 * Process one half of the DMA buffer (EM_HALF_PAIRS pairs).
 * Removes DC (bias midrail) per channel, then RMS / power.
 */
static void em_process_half(const uint16_t *buf, uint32_t pairs)
{
  float sum_v = 0.0f;
  float sum_i = 0.0f;
  uint32_t n = pairs;
  uint32_t i;

  /* Mean = DC bias estimate (midrail ~2048). Layout: [V, I] per pair. */
  for (i = 0; i < n; i++) {
    sum_v += (float)buf[i * 2U];
    sum_i += (float)buf[i * 2U + 1U];
  }
  const float mean_v = sum_v / (float)n;
  const float mean_i = sum_i / (float)n;

  float acc_v2 = 0.0f;
  float acc_i2 = 0.0f;
  float acc_p  = 0.0f;

  for (i = 0; i < n; i++) {
    const float v_c = (float)buf[i * 2U]     - mean_v;
    const float i_c = (float)buf[i * 2U + 1U] - mean_i;
    const float v_v = em_counts_to_volts(v_c) * EM_VOLTAGE_SCALE;
    const float i_a = em_counts_to_volts(i_c) * EM_CURRENT_SCALE;
    acc_v2 += v_v * v_v;
    acc_i2 += i_a * i_a;
    acc_p  += v_v * i_a;
  }

  const float v_rms = sqrtf(acc_v2 / (float)n);
  const float i_rms = sqrtf(acc_i2 / (float)n);
  const float p_w   = acc_p / (float)n;
  const float s_va  = v_rms * i_rms;
  float pf = 0.0f;
  if (s_va > 1e-3f) {
    pf = p_w / s_va;
    if (pf > 1.0f)  pf = 1.0f;
    if (pf < -1.0f) pf = -1.0f;
  }

  /* Energy via wall clock so Wh does not depend on exact ADC rate */
  const uint32_t now = HAL_GetTick();
  if (em_last_tick_ms != 0U) {
    const float dt_s = (float)(now - em_last_tick_ms) * 0.001f;
    if (dt_s > 0.0f && dt_s < 2.0f) {
      em_energy_ws += p_w * dt_s;
    }
  }
  em_last_tick_ms = now;

  em_readings.voltage_rms  = v_rms;
  em_readings.current_rms  = i_rms;
  em_readings.power_w      = p_w;
  em_readings.apparent_va  = s_va;
  em_readings.power_factor = pf;
  em_readings.energy_wh    = em_energy_ws / 3600.0f;
  em_readings.update_count++;
}

/* -------------------------------------------------------------------------- */
/* Public API                                                                 */
/* -------------------------------------------------------------------------- */

void EnergyMonitor_Init(ADC_HandleTypeDef *hadc)
{
  em_hadc = hadc;
  em_half_ready = EM_HALF_NONE;
  em_settle_left = EM_SETTLE_HALVES;
  em_last_tick_ms = 0U;
  em_energy_ws = 0.0f;
  memset(&em_readings, 0, sizeof(em_readings));
  memset(em_dma_buf, 0, sizeof(em_dma_buf));
}

void EnergyMonitor_Start(void)
{
  if (em_hadc == NULL) {
    return;
  }
  em_half_ready = EM_HALF_NONE;
  em_settle_left = EM_SETTLE_HALVES;
  em_last_tick_ms = 0U;

  /*
   * Cube sometimes leaves DMAContinuousRequests = DISABLE. With circular DMA
   * that stops new requests after the first fill — force DDS on before start.
   */
  em_hadc->Init.DMAContinuousRequests = ENABLE;

  /* Length is number of ADC transfers (each half-word = one channel sample). */
  if (HAL_ADC_Start_DMA(em_hadc, (uint32_t *)em_dma_buf, EM_BUF_LEN) != HAL_OK) {
    Error_Handler();
  }
}

void EnergyMonitor_Process(void)
{
  EmHalf_t half;

  __disable_irq();
  half = em_half_ready;
  em_half_ready = EM_HALF_NONE;
  __enable_irq();

  if (half == EM_HALF_NONE) {
    return;
  }

  if (em_settle_left > 0U) {
    em_settle_left--;
    return;
  }

  if (half == EM_HALF_FIRST) {
    em_process_half(&em_dma_buf[0], EM_HALF_PAIRS);
  } else {
    em_process_half(&em_dma_buf[EM_PAIR_COUNT], EM_HALF_PAIRS);
  }
}

EnergyMonitor_Readings_t EnergyMonitor_Get(void)
{
  return em_readings;
}

void EnergyMonitor_ResetEnergy(void)
{
  em_energy_ws = 0.0f;
  em_readings.energy_wh = 0.0f;
}

/* -------------------------------------------------------------------------- */
/* HAL weak callbacks — keep ISRs tiny                                        */
/* -------------------------------------------------------------------------- */

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == em_hadc) {
    em_half_ready = EM_HALF_FIRST;
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc == em_hadc) {
    em_half_ready = EM_HALF_SECOND;
  }
}
