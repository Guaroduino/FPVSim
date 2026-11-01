#include "fpvsim_hal.h"
#include "fpvsim_audio.h"
#include <driver/i2s.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static constexpr int   AUDIO_RATE   = 22050;
static constexpr int   AUDIO_FRAMES = 256;   // 11.6 ms por bloque
static constexpr i2s_port_t AUDIO_PORT = I2S_NUM_0;

struct EngineParams {
  volatile uint32_t inc1 = 0, inc2 = 0;     // incremento de fase Q32 por muestra
  volatile int32_t  gain1 = 0, gain2 = 0;   // Q15
  volatile int32_t  noiseA = 0;             // coeficiente del filtro paso bajo Q15
  volatile int32_t  noiseGain = 0;          // Q15
};
static EngineParams eng;

struct BeepState {
  volatile uint32_t inc = 0;
  volatile int32_t  samplesLeft = 0;
  volatile int32_t  gain = 0;
  volatile int      wave = 0;
};
static BeepState beep;
static volatile int32_t masterQ15 = 26000;

static inline uint32_t hzToInc(float hz) {
  if (hz < 0.0f) hz = 0.0f;
  return (uint32_t)((double)hz * 4294967296.0 / AUDIO_RATE);
}
static inline int32_t toQ15(float g) {
  if (g < 0.0f) g = 0.0f; else if (g > 1.0f) g = 1.0f;
  return (int32_t)(g * 32767.0f);
}

void audioSetEngine(float f1, float g1, float f2, float g2, float noiseCutHz, float noiseGain) {
  eng.inc1 = hzToInc(f1);
  eng.inc2 = hzToInc(f2);
  // Ganancias del web (0..0.05) se escalan x10 para el amplificador I2S
  eng.gain1 = toQ15(g1 * 10.0f);
  eng.gain2 = toQ15(g2 * 10.0f);
  // Paso bajo de 1 polo: a = 1 - exp(-2*pi*fc/fs) (calculado aquí, nunca por muestra)
  const float a = 1.0f - expf(-6.2831853f * noiseCutHz / (float)AUDIO_RATE);
  eng.noiseA = toQ15(a);
  eng.noiseGain = toQ15(noiseGain * 10.0f);
}

void audioBeep(float freqHz, int durationMs, int wave, float vol) {
  beep.samplesLeft = 0;
  beep.inc  = hzToInc(freqHz);
  beep.wave = wave;
  beep.gain = toQ15(vol);
  beep.samplesLeft = (int32_t)((int64_t)durationMs * AUDIO_RATE / 1000);
}

void audioSetMasterVolume(float vol) { masterQ15 = toQ15(vol); }

static void audioTask(void*) {
  static int16_t block[AUDIO_FRAMES];
  uint32_t ph1 = 0, ph2 = 0, phB = 0;
  uint32_t lfsr = 0xACE1u;
  int32_t  noiseLp = 0;
  for (;;) {
    const uint32_t inc1 = eng.inc1, inc2 = eng.inc2, incB = beep.inc;
    const int32_t g1 = eng.gain1, g2 = eng.gain2, nA = eng.noiseA, nG = eng.noiseGain;
    const int32_t master = masterQ15;
    for (int i = 0; i < AUDIO_FRAMES; i++) {
      ph1 += inc1; ph2 += inc2;
      const int32_t saw = (int32_t)(ph1 >> 16) - 32768;              // diente de sierra
      const int32_t sq  = (ph2 & 0x80000000u) ? 16384 : -16384;       // cuadrada
      lfsr ^= lfsr << 13; lfsr ^= lfsr >> 17; lfsr ^= lfsr << 5;      // xorshift32
      const int32_t white = (int32_t)(lfsr >> 16) - 32768;
      noiseLp += ((white - noiseLp) * nA) >> 15;
      int32_t mix = ((saw * g1) >> 16) + ((sq * g2) >> 15) + ((noiseLp * nG) >> 16);

      if (beep.samplesLeft > 0) {
        phB += incB;
        int32_t b;
        const int32_t t = (int32_t)(phB >> 16);                     // 0..65535
        if (beep.wave == 1)      b = ((t < 32768) ? (2 * t - 32768) : (32767 - 2 * (t - 32768))) >> 1; // triángulo ~ seno
        else if (beep.wave == 2) b = (t - 32768) >> 1;
        else                     b = (phB & 0x80000000u) ? 16384 : -16384;
        mix += (b * beep.gain) >> 15;
        beep.samplesLeft = beep.samplesLeft - 1;
      }
      mix = (mix * master) >> 15;
      if (mix > 32767) mix = 32767; else if (mix < -32768) mix = -32768;
      block[i] = (int16_t)mix;
    }
    size_t written = 0;
    i2s_write(AUDIO_PORT, block, sizeof(block), &written, portMAX_DELAY); // bloquea hasta que haya hueco en DMA
  }
}

bool audioBegin() {
  i2s_config_t cfg = {};
  cfg.mode                 = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate          = AUDIO_RATE;
  cfg.bits_per_sample      = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format       = I2S_CHANNEL_FMT_ONLY_LEFT;
  cfg.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  cfg.intr_alloc_flags     = 0;
  cfg.dma_buf_count        = 4;
  cfg.dma_buf_len          = AUDIO_FRAMES;
  cfg.use_apll             = false;
  cfg.tx_desc_auto_clear   = true;
  if (i2s_driver_install(AUDIO_PORT, &cfg, 0, nullptr) != ESP_OK) return false;

  i2s_pin_config_t pins = {};
  pins.mck_io_num   = I2S_PIN_NO_CHANGE;
  pins.bck_io_num   = PIN_I2S_BCLK;
  pins.ws_io_num    = PIN_I2S_LRCK;
  pins.data_out_num = PIN_I2S_DOUT;
  pins.data_in_num  = I2S_PIN_NO_CHANGE;
  if (i2s_set_pin(AUDIO_PORT, &pins) != ESP_OK) return false;

  // Prioridad mayor que pantalla/raycast: solo despierta cada 11.6 ms y calcula 256 muestras (~50 us)
  xTaskCreatePinnedToCore(audioTask, "audio", 3072, nullptr, 4, nullptr, 0);
  return true;
}
