#pragma once
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <string.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>

// ============================================================================
// 1. MAPA DE PINES OFICIAL: RETRO-GO FPVSim (ESP32-S3 N16R8)
// ¡PROHIBIDO USAR GPIO 33..37 (Reservados Bus PSRAM Octal 8MB)!
// ¡PROHIBIDO USAR GPIO 19..20 (Reservados USB OTG / Serial JTAG Nativo)!
// ============================================================================

// Pantalla IPS 2.8" ILI9341V (Bus SPI2 Nativo IOMUX @ 60-80 MHz DMA)
#define PIN_TFT_MOSI    11
#define PIN_TFT_MISO    -1  // No se lee la pantalla: GPIO 13 queda libre para el audio I2S
#define PIN_TFT_SCLK    12
#define PIN_TFT_CS      10
#define PIN_TFT_DC       9
#define PIN_TFT_RST     14
#define PIN_TFT_BL      21

// Audio I2S -> amplificador MAX98357A (mono, 3 W). GPIO 43/44 = UART0 libre porque el
// monitor serie va por el USB nativo (ARDUINO_USB_CDC_ON_BOOT=1). Al arrancar, la ROM
// escribe su log por GPIO 43: es normal un chasquido corto en el altavoz al encender.
#define PIN_I2S_BCLK    43
#define PIN_I2S_LRCK    44
#define PIN_I2S_DOUT    13

// Panel Touch Capacitivo FT6336U (I2C @ 400 kHz)
#define PIN_TOUCH_SDA    8
#define PIN_TOUCH_SCL   18
#define PIN_TOUCH_INT   17
#define PIN_TOUCH_RST   14  // Compartido con PIN_TFT_RST

// Joysticks Analógicos Duales Modo 2 (100% en ADC1 de 12 bits)
#define PIN_ADC_LX       1  // ADC1_CH0 -> Yaw
#define PIN_ADC_LY       2  // ADC1_CH1 -> Throttle
#define PIN_ADC_RX       4  // ADC1_CH3 -> Roll
#define PIN_ADC_RY       5  // ADC1_CH4 -> Pitch
#define PIN_BTN_L3       6  // Click Stick Izquierdo (INPUT_PULLUP)
#define PIN_BTN_R3       7  // Click Stick Derecho   (INPUT_PULLUP)

// Cruceta D-Pad + Sistema + Diamante de Acción (INPUT_PULLUP -> GND)
#define PIN_BTN_UP      15
#define PIN_BTN_DOWN    38
#define PIN_BTN_LEFT    39
#define PIN_BTN_RIGHT   40
#define PIN_BTN_SELECT  41
#define PIN_BTN_MENU    42
#define PIN_BTN_A       48  // Derecha (GAS / OK)
#define PIN_BTN_B       47  // Abajo   (BRK / BACK)
#define PIN_BTN_X       16  // Arriba  (RST)
#define PIN_BTN_Y        3  // Izq.    (MOD: ACRO/HOR)

// ============================================================================
// 2. RESOLUCIÓN Y CONSTANTES DE MEMORIA (AGENTS.md)
// ============================================================================
constexpr int DISP_W    = 320;
constexpr int DISP_H    = 240;
constexpr int VOX_W     = 256;
constexpr int VOX_H     = 256;

constexpr int MAP_W      = 2048;
constexpr int MAP_H      = 1024;
constexpr int MAP_MASK_X = 2047;
constexpr int MAP_MASK_Z = 1023;
constexpr int MAP_SHIFT_X = 11; // mz << 11 == mz * 2048

constexpr int SKY_DIM    = 512;
constexpr int SKY_MASK   = 511;
constexpr int SKY_SHIFT  = 9;   // cmz << 9 == cmz * 512

// Bandas de transferencia DMA (SRAM interna): 240 / 40 = 6 bandas por frame
constexpr int DMA_BAND_H = 40;

static inline constexpr uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return ((uint16_t)(r & 0xF8) << 8) | ((uint16_t)(g & 0xFC) << 3) | ((uint16_t)(b & 0xF8) >> 3);
}

static inline uint16_t blend565Half(uint16_t c1, uint16_t c2) {
  return (uint16_t)(((c1 & 0xF7DEu) >> 1) + ((c2 & 0xF7DEu) >> 1));
}

// Paleta rápida de 15 colores RGB565 (idéntica al prototipo web)
namespace C565 {
  constexpr uint16_t BG      = rgb565(8, 12, 22);
  constexpr uint16_t WHITE   = rgb565(240, 250, 255);
  constexpr uint16_t GOLD    = rgb565(255, 190, 0);
  constexpr uint16_t GREEN   = rgb565(0, 235, 118);
  constexpr uint16_t RED     = rgb565(245, 55, 65);
  constexpr uint16_t CYAN    = rgb565(0, 229, 255);
  constexpr uint16_t GREY    = rgb565(90, 102, 125);
  constexpr uint16_t DARKGRN = rgb565(18, 85, 48);
  constexpr uint16_t NAVY    = rgb565(20, 34, 68);
  constexpr uint16_t ORANGE  = rgb565(255, 120, 20);
  constexpr uint16_t MAGENTA = rgb565(245, 60, 210);
}

// ============================================================================
// 3. DRIVER LOVYANGFX CON DMA PARA ILI9341V (SPI2) + FT6336U (I2C)
// ============================================================================
class LGFX_RetroGo : public lgfx::LGFX_Device {
  lgfx::Panel_ILI9341 _panel;
  lgfx::Bus_SPI       _bus;
  lgfx::Light_PWM     _light;
  lgfx::Touch_FT5x06  _touch;

public:
  LGFX_RetroGo(void) {
    {
      auto cfg = _bus.config();
      cfg.spi_host    = SPI2_HOST;
      cfg.spi_mode    = 0;
      cfg.freq_write  = 60000000; // 60 MHz DMA (subible a 80000000 en PCB corto)
      cfg.freq_read   = 16000000;
      cfg.spi_3wire   = false;
      cfg.use_lock    = true;
      cfg.dma_channel = SPI_DMA_CH_AUTO;
      cfg.pin_sclk    = PIN_TFT_SCLK;
      cfg.pin_mosi    = PIN_TFT_MOSI;
      cfg.pin_miso    = PIN_TFT_MISO;
      cfg.pin_dc      = PIN_TFT_DC;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    {
      auto cfg = _panel.config();
      cfg.pin_cs          = PIN_TFT_CS;
      cfg.pin_rst         = PIN_TFT_RST;
      cfg.pin_busy        = -1;
      cfg.panel_width     = 240;
      cfg.panel_height    = 320;
      cfg.memory_width    = 240;
      cfg.memory_height   = 320;
      cfg.offset_rotation = 1;    // Horizontal 320x240
      cfg.invert          = true; // IPS activo
      cfg.rgb_order       = false;
      _panel.config(cfg);
    }
    {
      auto cfg = _light.config();
      cfg.pin_bl      = PIN_TFT_BL;
      cfg.invert      = false;
      cfg.freq        = 12000;
      cfg.pwm_channel = 7;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    {
      auto cfg = _touch.config();
      cfg.x_min           = 0;
      cfg.x_max           = 239;
      cfg.y_min           = 0;
      cfg.y_max           = 319;
      cfg.pin_int         = PIN_TOUCH_INT;
      cfg.pin_rst         = PIN_TOUCH_RST;
      cfg.bus_shared      = false;
      cfg.offset_rotation = 1;
      cfg.i2c_port        = 0;
      cfg.i2c_addr        = 0x38;
      cfg.pin_sda         = PIN_TOUCH_SDA;
      cfg.pin_scl         = PIN_TOUCH_SCL;
      cfg.freq            = 400000;
      _touch.config(cfg);
      _panel.setTouch(&_touch);
    }
    setPanel(&_panel);
  }
};

// ============================================================================
// 4. PUNTEROS GLOBALES EN SRAM INTERNA vs PSRAM OCTAL (AGENTS.md Regla 1)
// ============================================================================
extern LGFX_RetroGo tft;

// Doble framebuffer completo en PSRAM Octal (2 x 150 KB): el núcleo de render dibuja
// en el buffer trasero mientras la tarea de pantalla envía el delantero por DMA en bandas.
// vram565 apunta siempre al buffer trasero del frame en curso (RGB565 nativo little-endian).
extern uint16_t* fbPsram[2];  // 2 * 320 * 240 * 2 = 300 KB PSRAM
extern uint16_t* vram565;     // = fbPsram[backIdx]

// SRAM Interna (MALLOC_CAP_INTERNAL): acceso aleatorio por columna y DMA
extern uint16_t* voxBuf;      // 256 * 256 * 2 = 131,072 B (128 KB SRAM)
extern uint16_t* dmaBand[2];  // 2 * 320 * DMA_BAND_H * 2 B (ping-pong DMA en SRAM)

// PSRAM Octal (MALLOC_CAP_SPIRAM): mapas en bloques tileados 8x8 (igual que el simulador web)
extern uint16_t* voxMap;    // 2048 * 1024 * 2 = 4.0 MB PSRAM: (altura << 8) | índice de paleta
extern uint16_t  voxPal[256]; // paleta RGB565 del mapa activo (SRAM)
extern uint16_t* voxSkyMap; //  512 *  512 * 2 = 0.5 MB PSRAM

// Índice tileado 8x8 de los mapas PSRAM (idéntico a VOX_TILE_Z_LUT[mz] | VOX_TILE_X_LUT[mx] del web)
static inline uint32_t voxTileIdx(uint32_t mx, uint32_t mz) {
  return ((mz >> 3) << 14) | ((mx >> 3) << 6) | ((mz & 7u) << 3) | (mx & 7u);
}

// ROM de Glifos OSD de 2 bits pre-bordeados estilo MAX7456 en SRAM Interna
constexpr int OSD_SM_W = 5;
constexpr int OSD_SM_H = 7;
extern uint8_t OSD_SMALL_ROM[128 * OSD_SM_W * OSD_SM_H]; // 4,480 B en SRAM

// ============================================================================
// 5. ESTADO DE CONTROLES FÍSICOS (ADC1 + 12 BOTONES + TOUCH FT6336)
// ============================================================================
struct HwInputState {
  // Lecturas crudas ADC1 de 12 bits (0..4095) y centros calibrados
  int rawLX = 2048, rawLY = 2048, rawRX = 2048, rawRY = 2048;
  int centerLX = 2048, centerLY = 2048, centerRX = 2048, centerRY = 2048;

  // Ejes normalizados [-1.0f .. +1.0f] con deadzone aplicada
  float lx = 0.0f; // Yaw
  float ly = 0.0f; // Throttle
  float rx = 0.0f; // Roll
  float ry = 0.0f; // Pitch

  // 12 Botones digitales (actual y frame anterior para detección de flanco)
  bool up = false, down = false, left = false, right = false;
  bool select = false, menu = false;
  bool a = false, b = false, x = false, y = false;
  bool l3 = false, r3 = false;

  bool prevSelect = false, prevMenu = false;
  bool prevA = false, prevB = false, prevX = false, prevY = false;

  // Touch Capacitivo FT6336
  bool touchDown = false;
  int16_t touchX = 0, touchY = 0;
};

extern HwInputState hwInput;

// ============================================================================
// 5B. MEDICIÓN POR ETAPA (microsegundos, último frame) — visible con SELECT
// ============================================================================
struct FrameStats {
  uint32_t inputUs = 0, physicsUs = 0;
  uint32_t rayCore1Us = 0, rayCore0Us = 0;
  uint32_t blitUs = 0, overlayUs = 0;
  uint32_t waitDisplayUs = 0;   // tiempo que el render esperó a que la pantalla quedara libre
  uint32_t displayPushUs = 0;   // duración de la transferencia DMA completa del último frame
  uint32_t frameUs = 0;
  int      physicsSteps = 0;
};
extern FrameStats frameStats;

// ============================================================================
// 6. PRIMITIVAS DE DIBUJO EN VRAM565 Y TEXTO 2-BIT MAX7456 (1 SOLA PASADA)
// ============================================================================
static inline void fbSetPixel320(int px, int py, uint16_t c565) {
  if ((unsigned)px < DISP_W && (unsigned)py < DISP_H) {
    vram565[py * DISP_W + px] = c565;
  }
}

void buildMax7456CharacterRom();
void drawInavSmallText320(const char* str, int x320, int y240, uint16_t fg565 = 0xFFFF);
void fbDrawLine320(float x0, float y0, float x1, float y1, uint16_t c565);
bool halInitConsole();
void halCalibrateSticksCenter();
void halPollInputs();
