/**
 * @file    EnergyMonitor.h
 * @brief   Public API for the energy monitor application logic.
 *
 * All measurement code lives in EnergyMonitor.c.
 * main.c should only call Init / Start / Process.
 *
 * CubeMX prerequisites (must match or this module will not work):
 *   - ADC1 Rank1 = IN0 (PA0) = voltage
 *   - ADC1 Rank2 = IN1 (PA1) = current
 *   - Scan mode ON, Continuous ON, DMA Continuous Requests ON
 *   - DMA circular, half-word, 2 conversions per sequence
 * See EnergyMonitor.c header comment for the full checklist.
 */
#ifndef ENERGY_MONITOR_H
#define ENERGY_MONITOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

typedef struct {
  float voltage_rms;   /* Vrms  */
  float current_rms;   /* Arms  */
  float power_w;       /* active power W */
  float apparent_va;   /* Vrms * Irms */
  float power_factor;  /* -1..1 */
  float energy_wh;     /* accumulated Wh since Init */
  uint32_t update_count;
} EnergyMonitor_Readings_t;

void EnergyMonitor_Init(ADC_HandleTypeDef *hadc);
void EnergyMonitor_Start(void);
void EnergyMonitor_Process(void);
EnergyMonitor_Readings_t EnergyMonitor_Get(void);
void EnergyMonitor_ResetEnergy(void);

#ifdef __cplusplus
}
#endif

#endif /* ENERGY_MONITOR_H */
