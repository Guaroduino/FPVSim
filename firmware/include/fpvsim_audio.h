#pragma once
#include <stdint.h>

// ============================================================================
// AUDIO I2S (MAX98357A u otro DAC/amplificador I2S mono)
// Mismo sintetizador que el simulador web: 2 osciladores (diente de sierra + cuadrada)
// + ruido filtrado + pitidos de eventos. Corre en una tarea propia del Core 0 a 22.05 kHz
// (~2-3 % de un núcleo). Pines en fpvsim_hal.h: PIN_I2S_BCLK / PIN_I2S_LRCK / PIN_I2S_DOUT.
// ============================================================================
bool audioBegin();
// Motor / viento continuo (se llama una vez por frame desde el bucle principal)
void audioSetEngine(float freq1Hz, float gain1, float freq2Hz, float gain2, float noiseCutHz, float noiseGain);
// Pitido de evento (puerta superada, menú, etc.). wave: 0 = cuadrada, 1 = seno aproximado, 2 = sierra
void audioBeep(float freqHz, int durationMs, int wave = 0, float vol = 0.25f);
void audioSetMasterVolume(float vol); // 0..1
