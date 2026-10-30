#include "fpvsim_hal.h"
#include <stdio.h>

// ============================================================================
// 1. BUFFERS GLOBALES (SRAM INTERNA DMA vs PSRAM OCTAL 8MB)
// ============================================================================
LGFX_RetroGo tft;

uint16_t* vram565   = nullptr; // 150 KB en SRAM Interna (DMA)
uint16_t* voxBuf    = nullptr; // 128 KB en SRAM Interna (Raycasting vertical)
uint8_t*  voxHeight = nullptr; // 2.0 MB en PSRAM Octal (Alturas 2048x1024)
uint16_t* voxColor  = nullptr; // 4.0 MB en PSRAM Octal (Color RGB565 2048x1024)
uint16_t* voxSkyMap = nullptr; // 0.5 MB en PSRAM Octal (Domo de Cielo 512x512)

uint8_t OSD_SMALL_ROM[128 * OSD_SM_W * OSD_SM_H];
HwInputState hwInput;

// ============================================================================
// 2. FUENTE BASE 3x5 Y COMPILADOR DE GLIFOS 2-BIT ESTILO MAX7456 (AGENTS.md #3)
//    0 = Transparente, 1 = Borde Negro 0x0000, 2 = Relleno Blanco/Color
// ============================================================================
static const uint8_t* getGlyph3x5(char ch) {
  static const struct { char c; uint8_t rows[5]; } TABLE[] = {
    {'0',{7,5,5,5,7}}, {'1',{2,6,2,2,7}}, {'2',{7,1,7,4,7}}, {'3',{7,1,7,1,7}},
    {'4',{5,5,7,1,1}}, {'5',{7,4,7,1,7}}, {'6',{7,4,7,5,7}}, {'7',{7,1,1,2,2}},
    {'8',{7,5,7,5,7}}, {'9',{7,5,7,1,7}},
    {'A',{2,5,7,5,5}}, {'B',{6,5,6,5,6}}, {'C',{7,4,4,4,7}}, {'D',{6,5,5,5,6}},
    {'E',{7,4,6,4,7}}, {'F',{7,4,6,4,4}}, {'G',{7,4,5,5,7}}, {'H',{5,5,7,5,5}},
    {'I',{7,2,2,2,7}}, {'J',{1,1,1,5,7}}, {'K',{5,5,6,5,5}}, {'L',{4,4,4,4,7}},
    {'M',{5,7,7,5,5}}, {'N',{5,7,7,7,5}}, {'O',{7,5,5,5,7}}, {'P',{7,5,7,4,4}},
    {'Q',{7,5,5,7,3}}, {'R',{6,5,6,5,5}}, {'S',{7,4,7,1,7}}, {'T',{7,2,2,2,2}},
    {'U',{5,5,5,5,7}}, {'V',{5,5,5,5,2}}, {'W',{5,5,7,7,5}}, {'X',{5,5,2,5,5}},
    {'Y',{5,5,2,2,2}}, {'Z',{7,1,2,4,7}},
    {':',{0,2,0,2,0}}, {'-',{0,0,7,0,0}}, {'/',{1,1,2,4,4}}, {'.',{0,0,0,0,2}},
    {'+',{0,2,7,2,0}}, {'>',{4,2,1,2,4}}, {'<',{1,2,4,2,1}}, {'%',{5,1,2,4,5}},
    {'[',{7,4,4,4,7}}, {']',{7,1,1,1,7}}, {'*',{5,2,7,2,5}}, {'(',{2,4,4,4,2}},
    {')',{2,1,1,1,2}}, {'|',{2,2,2,2,2}}, {',',{0,0,0,2,4}}, {'=',{0,7,0,7,0}}
  };
  for (size_t i = 0; i < sizeof(TABLE) / sizeof(TABLE[0]); i++) {
    if (TABLE[i].c == ch) return TABLE[i].rows;
  }
  return nullptr;
}

void buildMax7456CharacterRom() {
  memset(OSD_SMALL_ROM, 0, sizeof(OSD_SMALL_ROM));
  for (int code = 32; code < 128; code++) {
    const uint8_t* g = getGlyph3x5((char)code);
    if (!g) continue;
    const int smBase = code * (OSD_SM_W * OSD_SM_H);
    for (int r = 0; r < 5; r++) {
      const uint8_t rowBits = g[r];
      for (int c = 0; c < 3; c++) {
        if ((rowBits >> (2 - c)) & 1) {
          const int px = 1 + c;
          const int py = 1 + r;
          for (int oy = -1; oy <= 1; oy++) {
            for (int ox = -1; ox <= 1; ox++) {
              const int gx = px + ox, gy = py + oy;
              const int idx = smBase + gy * OSD_SM_W + gx;
              if (ox == 0 && oy == 0) {
                OSD_SMALL_ROM[idx] = 2; // Relleno 1px nativo
              } else if (OSD_SMALL_ROM[idx] == 0) {
                OSD_SMALL_ROM[idx] = 1; // Borde negro 1px
              }
            }
          }
        }
      }
    }
  }
}

// Dibuja texto OSD pre-bordeado de 2 bits en UNA SOLA PASADA LINEAL (sin bucles vecinos)
void drawInavSmallText320(const char* str, int x320, int y240, uint16_t fg565) {
  int cx = x320 - 1;
  const int cy = y240 - 1;
  for (int i = 0; str[i] != '\0'; i++) {
    char ch = str[i];
    if (ch >= 'a' && ch <= 'z') ch -= 32;
    const int code = ((unsigned char)ch) & 127;
    const int baseOff = code * (OSD_SM_W * OSD_SM_H);
    for (int gy = 0; gy < OSD_SM_H; gy++) {
      const int py = cy + gy;
      if ((unsigned)py >= DISP_H) continue;
      const int dstRow = py * DISP_W;
      const int srcRow = baseOff + gy * OSD_SM_W;
      for (int gx = 0; gx < OSD_SM_W; gx++) {
        const uint8_t v = OSD_SMALL_ROM[srcRow + gx];
        if (v != 0) {
          const int px = cx + gx;
          if ((unsigned)px < DISP_W) {
            if (v == 2) {
              vram565[dstRow + px] = fg565;
            } else if (vram565[dstRow + px] != fg565) {
              vram565[dstRow + px] = 0x0000;
            }
          }
        }
      }
    }
    cx += 5; // Avance compacto 5px (4px núcleo + 1px borde compartido)
  }
}

// ============================================================================
// 3. RASTERIZADOR ENTERO BRESENHAM + RECORTE COHEN-SUTHERLAND EN 320x240
// ============================================================================
constexpr int CLIP_LEFT = 1, CLIP_RIGHT = 2, CLIP_BOTTOM = 4, CLIP_TOP = 8;

static inline int computeOutCode320(float x, float y) {
  int c = 0;
  if (x < 0.0f) c |= CLIP_LEFT; else if (x >= (float)DISP_W) c |= CLIP_RIGHT;
  if (y < 0.0f) c |= CLIP_TOP;  else if (y >= (float)DISP_H) c |= CLIP_BOTTOM;
  return c;
}

void fbDrawLine320(float x0, float y0, float x1, float y1, uint16_t c565) {
  int out0 = computeOutCode320(x0, y0);
  int out1 = computeOutCode320(x1, y1);
  bool accept = false;

  for (int iter = 0; iter < 12; iter++) {
    if (!(out0 | out1)) { accept = true; break; }
    if (out0 & out1) break;
    float x = 0.0f, y = 0.0f;
    const int out = out0 ? out0 : out1;
    const float dx = x1 - x0, dy = y1 - y0;
    if (out & CLIP_BOTTOM) {
      x = fabsf(dy) > 1e-5f ? x0 + dx * ((DISP_H - 1) - y0) / dy : x0;
      y = (float)(DISP_H - 1);
    } else if (out & CLIP_TOP) {
      x = fabsf(dy) > 1e-5f ? x0 + dx * (0.0f - y0) / dy : x0;
      y = 0.0f;
    } else if (out & CLIP_RIGHT) {
      y = fabsf(dx) > 1e-5f ? y0 + dy * ((DISP_W - 1) - x0) / dx : y0;
      x = (float)(DISP_W - 1);
    } else if (out & CLIP_LEFT) {
      y = fabsf(dx) > 1e-5f ? y0 + dy * (0.0f - x0) / dx : y0;
      x = 0.0f;
    }
    if (out == out0) { x0 = x; y0 = y; out0 = computeOutCode320(x0, y0); }
    else             { x1 = x; y1 = y; out1 = computeOutCode320(x1, y1); }
  }
  if (!accept) return;

  int ix0 = (int)lroundf(x0); if (ix0 < 0) ix0 = 0; else if (ix0 >= DISP_W) ix0 = DISP_W - 1;
  int iy0 = (int)lroundf(y0); if (iy0 < 0) iy0 = 0; else if (iy0 >= DISP_H) iy0 = DISP_H - 1;
  int ix1 = (int)lroundf(x1); if (ix1 < 0) ix1 = 0; else if (ix1 >= DISP_W) ix1 = DISP_W - 1;
  int iy1 = (int)lroundf(y1); if (iy1 < 0) iy1 = 0; else if (iy1 >= DISP_H) iy1 = DISP_H - 1;

  const int dx = abs(ix1 - ix0), sx = ix0 < ix1 ? 1 : -1;
  const int dy = -abs(iy1 - iy0), sy = iy0 < iy1 ? 1 : -1;
  int err = dx + dy;

  for (int step = 0; step < 640; step++) {
    fbSetPixel320(ix0, iy0, c565);
    if (ix0 + 1 < DISP_W) fbSetPixel320(ix0 + 1, iy0, c565);
    if (ix0 == ix1 && iy0 == iy1) break;
    const int e2 = err << 1;
    if (e2 >= dy) { err += dy; ix0 += sx; }
    if (e2 <= dx) { err += dx; iy0 += sy; }
  }
}

// ============================================================================
// 4. INICIALIZACIÓN DE CONSOLA, CALIBRACIÓN Y LECTURA DE HARDWARE
// ============================================================================
static float applyDeadzoneNorm(int rawVal, int centerVal, float dz) {
  const float span = rawVal >= centerVal ? (float)(4095 - centerVal) : (float)centerVal;
  const float safeSpan = span < 200.0f ? 2048.0f : span;
  float v = (float)(rawVal - centerVal) / safeSpan;
  if (v < -1.0f) v = -1.0f; else if (v > 1.0f) v = 1.0f;
  const float av = fabsf(v);
  if (av <= dz) return 0.0f;
  const float sign = v >= 0.0f ? 1.0f : -1.0f;
  return sign * ((av - dz) / (1.0f - dz));
}

void halCalibrateSticksCenter() {
  long sumLX = 0, sumLY = 0, sumRX = 0, sumRY = 0;
  for (int i = 0; i < 16; i++) {
    sumLX += analogRead(PIN_ADC_LX);
    sumLY += analogRead(PIN_ADC_LY);
    sumRX += analogRead(PIN_ADC_RX);
    sumRY += analogRead(PIN_ADC_RY);
    delay(2);
  }
  hwInput.centerLX = (int)(sumLX >> 4);
  hwInput.centerLY = (int)(sumLY >> 4);
  hwInput.centerRX = (int)(sumRX >> 4);
  hwInput.centerRY = (int)(sumRY >> 4);
}

void halPollInputs() {
  hwInput.prevSelect = hwInput.select;
  hwInput.prevMenu   = hwInput.menu;
  hwInput.prevA      = hwInput.a;
  hwInput.prevB      = hwInput.b;
  hwInput.prevX      = hwInput.x;
  hwInput.prevY      = hwInput.y;

  // 1. Lectura ADC1 de 12 bits (con promedio rápido de 2 muestras para estabilidad)
  hwInput.rawLX = (analogRead(PIN_ADC_LX) + analogRead(PIN_ADC_LX)) >> 1;
  hwInput.rawLY = (analogRead(PIN_ADC_LY) + analogRead(PIN_ADC_LY)) >> 1;
  hwInput.rawRX = (analogRead(PIN_ADC_RX) + analogRead(PIN_ADC_RX)) >> 1;
  hwInput.rawRY = (analogRead(PIN_ADC_RY) + analogRead(PIN_ADC_RY)) >> 1;

  hwInput.lx = applyDeadzoneNorm(hwInput.rawLX, hwInput.centerLX, 0.06f);
  hwInput.ly = applyDeadzoneNorm(hwInput.rawLY, hwInput.centerLY, 0.06f);
  hwInput.rx = applyDeadzoneNorm(hwInput.rawRX, hwInput.centerRX, 0.06f);
  hwInput.ry = applyDeadzoneNorm(hwInput.rawRY, hwInput.centerRY, 0.06f);

  // 2. Lectura de los 12 botones digitales (Activos en LOW por INPUT_PULLUP)
  hwInput.up     = (digitalRead(PIN_BTN_UP)     == LOW);
  hwInput.down   = (digitalRead(PIN_BTN_DOWN)   == LOW);
  hwInput.left   = (digitalRead(PIN_BTN_LEFT)   == LOW);
  hwInput.right  = (digitalRead(PIN_BTN_RIGHT)  == LOW);
  hwInput.select = (digitalRead(PIN_BTN_SELECT) == LOW);
  hwInput.menu   = (digitalRead(PIN_BTN_MENU)   == LOW);
  hwInput.a      = (digitalRead(PIN_BTN_A)      == LOW);
  hwInput.b      = (digitalRead(PIN_BTN_B)      == LOW);
  hwInput.x      = (digitalRead(PIN_BTN_X)      == LOW);
  hwInput.y      = (digitalRead(PIN_BTN_Y)      == LOW);
  hwInput.l3     = (digitalRead(PIN_BTN_L3)     == LOW);
  hwInput.r3     = (digitalRead(PIN_BTN_R3)     == LOW);

  // Si no se mueve el stick izquierdo, la cruceta D-Pad también aporta mando
  if (fabsf(hwInput.lx) < 0.01f && (hwInput.left || hwInput.right)) {
    hwInput.lx = (hwInput.right ? 0.75f : 0.0f) - (hwInput.left ? 0.75f : 0.0f);
  }
  if (fabsf(hwInput.ly) < 0.01f && (hwInput.up || hwInput.down)) {
    hwInput.ly = (hwInput.up ? 0.75f : 0.0f) - (hwInput.down ? 0.75f : 0.0f);
  }

  // 3. Lectura del panel Touch Capacitivo FT6336U
  uint16_t tx = 0, ty = 0;
  hwInput.touchDown = tft.getTouch(&tx, &ty);
  if (hwInput.touchDown) {
    hwInput.touchX = (int16_t)tx;
    hwInput.touchY = (int16_t)ty;
  }
}

bool halInitConsole() {
  vram565   = (uint16_t*)heap_caps_malloc(DISP_W * DISP_H * sizeof(uint16_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
  voxBuf    = (uint16_t*)heap_caps_malloc(VOX_W * VOX_H * sizeof(uint16_t),   MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  voxHeight = (uint8_t*) heap_caps_malloc(MAP_W * MAP_H * sizeof(uint8_t),    MALLOC_CAP_SPIRAM);
  voxColor  = (uint16_t*)heap_caps_malloc(MAP_W * MAP_H * sizeof(uint16_t),   MALLOC_CAP_SPIRAM);
  voxSkyMap = (uint16_t*)heap_caps_malloc(SKY_DIM * SKY_DIM * sizeof(uint16_t), MALLOC_CAP_SPIRAM);

  if (!vram565 || !voxBuf || !voxHeight || !voxColor || !voxSkyMap) {
    return false;
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_LX, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_LY, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_RX, ADC_11db);
  analogSetPinAttenuation(PIN_ADC_RY, ADC_11db);

  const int btnPins[12] = {
    PIN_BTN_UP, PIN_BTN_DOWN, PIN_BTN_LEFT, PIN_BTN_RIGHT,
    PIN_BTN_SELECT, PIN_BTN_MENU, PIN_BTN_A, PIN_BTN_B,
    PIN_BTN_X, PIN_BTN_Y, PIN_BTN_L3, PIN_BTN_R3
  };
  for (int p : btnPins) {
    pinMode(p, INPUT_PULLUP);
  }

  buildMax7456CharacterRom();
  halCalibrateSticksCenter();
  return true;
}

// ============================================================================
// 5. MOTOR VOXEL COMANCHE EN PUNTO FIJO INT32 (AGENTS.md Regla 2)
//    Cero divisiones flotantes o funciones trascendentales en bucles de rayos/píxeles
// ============================================================================
constexpr float WORLD_W_M    = 1500.0f;
constexpr float WORLD_H_M    = 1000.0f;
constexpr float MAP_SCALE_X  = (float)MAP_W / WORLD_W_M;
constexpr float MAP_SCALE_Z  = (float)MAP_H / WORLD_H_M;
constexpr float HEIGHT_SCALE = 0.125f;
constexpr int   SPAWN_MX     = 1198;
constexpr int   SPAWN_MZ     = 650;

constexpr float FOCAL_240    = 255.0f;
constexpr float BLIT_SCALE   = 0.635f;
constexpr float FOCAL_VOX    = FOCAL_240 * BLIT_SCALE;

// Tablas LUT en SRAM Interna (< 4.5 KB)
constexpr int NUM_RAY_STEPS = 312;
static int32_t RAY_S_Q8[NUM_RAY_STEPS];
static int32_t BASE_V_Q8[NUM_RAY_STEPS];
static int32_t PROJ_SCALE_Q8[NUM_RAY_STEPS];
static int32_t CLOUD_DIST_Q8[512];
static uint8_t MICRO_TEX_LUT[256];
static float   sinLut512[512];

static float lastRollAngle = 0.0f;
static float cloudDriftX   = 0.0f;
static uint16_t fogCol565  = rgb565(166, 198, 228);

static void initVoxelFixedPointLutsAndWorld() {
  for (int i = 0; i < 512; i++) {
    sinLut512[i]     = sinf((i * 2.0f * (float)M_PI) / 512.0f);
    CLOUD_DIST_Q8[i] = (int32_t)lroundf((2400.0f / (float)(i + 10)) * 256.0f);
  }
  for (int i = 0; i < 256; i++) {
    const int h = (i * 73 + (i >> 4) * 37) ^ (i * 29);
    MICRO_TEX_LUT[i] = (uint8_t)(h & 3);
  }

  // 1. Generar Domo de Cielo + Nubes 512x512 en PSRAM Octal
  for (int mz = 0; mz < SKY_DIM; mz++) {
    const int zOff = mz << SKY_SHIFT;
    for (int mx = 0; mx < SKY_DIM; mx++) {
      const float n1 = sinLut512[(mx + mz) & 511] * sinLut512[(mx - mz + 128) & 511];
      const float n2 = sinLut512[(mx * 2 + mz * 3 + 40) & 511] * sinLut512[(mz * 2 - mx + 190) & 511];
      const float cn = n1 * 0.60f + n2 * 0.40f;
      float d = (cn - 0.08f) * 1.85f;
      if (d < 0.0f) d = 0.0f; else if (d > 1.0f) d = 1.0f;
      const float cSmooth = d * d * (3.0f - 2.0f * d);
      const uint8_t sr = (uint8_t)(44.0f  * (1.0f - cSmooth) + 246.0f * cSmooth);
      const uint8_t sg = (uint8_t)(92.0f  * (1.0f - cSmooth) + 248.0f * cSmooth);
      const uint8_t sb = (uint8_t)(178.0f * (1.0f - cSmooth) + 252.0f * cSmooth);
      voxSkyMap[zOff | mx] = rgb565(sr, sg, sb);
    }
  }

  // 2. Generar Terreno Voxel 2048x1024 en PSRAM Octal (con Pista A.I.A Chorobobo plana en el centro)
  for (int mz = 0; mz < MAP_H; mz++) {
    const int rowOff = mz << MAP_SHIFT_X;
    for (int mx = 0; mx < MAP_W; mx++) {
      const float dxSpawn = (float)(mx - SPAWN_MX);
      const float dzSpawn = (float)(mz - SPAWN_MZ);
      const float distSq  = dxSpawn * dxSpawn + dzSpawn * dzSpawn;

      const float w1 = sinLut512[(mx >> 2) & 511] * sinLut512[(mz >> 2) & 511];
      const float w2 = sinLut512[(mx >> 1) & 511] * sinLut512[((mz >> 1) + 90) & 511];
      float hRaw = 24.0f + w1 * 38.0f + w2 * 18.0f;

      // Aplanar la zona de la pista de aterrizaje cerca de SPAWN_MX, SPAWN_MZ
      if (distSq < 18000.0f) {
        const float flatFactor = distSq / 18000.0f;
        hRaw = 16.0f + (hRaw - 16.0f) * flatFactor;
      }
      if (hRaw < 4.0f) hRaw = 4.0f; else if (hRaw > 220.0f) hRaw = 220.0f;
      voxHeight[rowOff | mx] = (uint8_t)hRaw;

      // Color de pradera, parcelas agrícolas y pista de asfalto central
      uint8_t r = (uint8_t)(46 + ((int)hRaw >> 2));
      uint8_t g = (uint8_t)(94 + ((int)hRaw >> 2));
      uint8_t b = (uint8_t)(42 + ((int)hRaw >> 3));
      if (abs(mx - SPAWN_MX) < 85 && abs(mz - SPAWN_MZ) < 12) {
        r = 82; g = 86; b = 94; // Pista principal
      } else if (((mx >> 6) + (mz >> 6)) & 1) {
        r = (uint8_t)(r + 14); g = (uint8_t)(g + 8);
      }
      voxColor[rowOff | mx] = rgb565(r, g, b);
    }
  }
}

static inline float getTerrainHeightMeters(float wx, float wz) {
  const int mx = ((int)floorf(SPAWN_MX + wx * MAP_SCALE_X)) & MAP_MASK_X;
  const int mz = ((int)floorf(SPAWN_MZ + wz * MAP_SCALE_Z)) & MAP_MASK_Z;
  return voxHeight[(mz << MAP_SHIFT_X) | mx] * HEIGHT_SCALE;
}

static void renderVoxelTerrainAndSky(
  float camX, float camY, float camZ,
  float cRx, float cRy, float cRz,
  float cUx, float cUy, float cUz,
  float cFx, float cFy, float cFz,
  float dt
) {
  cloudDriftX = fmodf(cloudDriftX + dt * 1.8f, 512.0f);

  // 1. Extraer ángulo de Roll y base des-rotada (1 sola vez por frame fuera de los bucles)
  const float upMag = hypotf(cRy, cUy);
  if (upMag > 0.04f) {
    lastRollAngle = atan2f(cRy, cUy);
  }
  const float cosR = cosf(lastRollAngle);
  const float sinR = sinf(lastRollAngle);

  float R0x = cosR * cRx - sinR * cUx;
  float R0z = cosR * cRz - sinR * cUz;
  const float rLen = hypotf(R0x, R0z);
  if (rLen > 1e-5f) { R0x /= rLen; R0z /= rLen; }

  const float H0x = -R0z;
  const float H0z =  R0x;

  const float U0x = sinR * cRx + cosR * cUx;
  const float U0y = sinR * cRy + cosR * cUy;
  const float U0z = sinR * cRz + cosR * cUz;

  const float uH = U0x * H0x + U0z * H0z;
  const float fH = cFx * H0x + cFz * H0z;

  float horizonFloat = fH > 0.04f
    ? (128.0f - (uH / fH) * FOCAL_VOX)
    : (uH > 0.0f ? -256.0f : 512.0f);
  if (horizonFloat < -256.0f) horizonFloat = -256.0f;
  if (horizonFloat >  512.0f) horizonFloat =  512.0f;

  const int   horizonV         = (int)lroundf(horizonFloat);
  const float subPxShift       = (float)horizonV - horizonFloat;
  const int32_t horizonSubPxQ16 = (int32_t)lroundf(subPxShift * 65536.0f);

  // 2. Precalcular intersección por fila (1 vez por frame)
  const float invFocalVox = 1.0f / FOCAL_VOX;
  const float groundHere  = getTerrainHeightMeters(camX, camZ);
  const int   groundTex   = (int)lroundf(groundHere / HEIGHT_SCALE);
  const float aglHere     = fmaxf(0.6f, camY - groundHere);
  const float dy0         = -aglHere;
  const int   vBotStart   = aglHere < 24.0f ? 304 : 262;
  const int   vTopLim     = horizonV + 1 < 0 ? 0 : (horizonV + 1 > 256 ? 256 : horizonV + 1);

  int activeSteps = 0;
  for (int vRef = vBotStart; vRef >= vTopLim && activeSteps < NUM_RAY_STEPS; vRef--) {
    const float eta   = (128.0f + subPxShift - (float)vRef) * invFocalVox;
    const float denom = uH - fH * eta;
    if (denom <= 1e-4f) break;
    const float num = aglHere * (U0y - cFy * eta);
    float s = num / denom;
    if (s < -12000.0f) s = -12000.0f; else if (s > 12000.0f) s = 12000.0f;
    const float cz = s * fH + dy0 * cFy;
    if (cz <= 0.20f) continue;
    RAY_S_Q8[activeSteps]      = (int32_t)lroundf(s * 256.0f);
    BASE_V_Q8[activeSteps]     = vRef << 8;
    PROJ_SCALE_Q8[activeSteps] = (int32_t)lroundf(((HEIGHT_SCALE * U0y * FOCAL_VOX) / cz) * 256.0f);
    activeSteps++;
  }

  const int microMaxK = aglHere < 16.0f ? (int)lroundf((16.0f - aglHere) * 5.0f) : 0;
  const float baseMx  = fmodf(fmodf(SPAWN_MX + camX * MAP_SCALE_X, (float)MAP_W) + (float)MAP_W, (float)MAP_W);
  const float baseMz  = fmodf(fmodf(SPAWN_MZ + camZ * MAP_SCALE_Z, (float)MAP_H) + (float)MAP_H, (float)MAP_H);
  const int32_t startX_Q19    = (int32_t)lroundf(baseMx * 524288.0f);
  const int32_t startZ_Q19    = (int32_t)lroundf(baseMz * 524288.0f);
  const int32_t skyStartX_Q16 = (int32_t)lroundf((camX * 0.45f + cloudDriftX) * 65536.0f);
  const int32_t skyStartZ_Q16 = (int32_t)lroundf((camZ * 0.45f) * 65536.0f);
  const float dy0_cFy         = dy0 * cFy;

  // 3. Bucle de Raycasting por columna u (0..255): 100% Punto Fijo Int32 y LUTs
  for (int u = 0; u < VOX_W; u++) {
    const float xi0  = (float)(u - 128) * invFocalVox;
    const float lat0 = xi0 * dy0_cFy;
    const int32_t colStartX_Q19 = startX_Q19 + (int32_t)lroundf(lat0 * R0x * MAP_SCALE_X * 524288.0f);
    const int32_t colStartZ_Q19 = startZ_Q19 + (int32_t)lroundf(lat0 * R0z * MAP_SCALE_Z * 524288.0f);

    const float dirW_X = H0x + (xi0 * fH) * R0x;
    const float dirW_Z = H0z + (xi0 * fH) * R0z;

    const int32_t dirX_Q11 = (int32_t)lroundf(dirW_X * MAP_SCALE_X * 2048.0f);
    const int32_t dirZ_Q11 = (int32_t)lroundf(dirW_Z * MAP_SCALE_Z * 2048.0f);
    const int32_t dirWX_Q8 = (int32_t)lroundf(dirW_X * 256.0f);
    const int32_t dirWZ_Q8 = (int32_t)lroundf(dirW_Z * 256.0f);

    int yMax = VOX_H;

    for (int k = 0; k < activeSteps && yMax > 0; k++) {
      const int32_t sQ8  = RAY_S_Q8[k];
      const int32_t xQ19 = colStartX_Q19 + dirX_Q11 * sQ8;
      const int32_t zQ19 = colStartZ_Q19 + dirZ_Q11 * sQ8;
      const int mx       = (xQ19 >> 19) & MAP_MASK_X;
      const int mz       = (zQ19 >> 19) & MAP_MASK_Z;
      const int mIdx     = (mz << MAP_SHIFT_X) | mx;

      const int rawDh = (int)voxHeight[mIdx] - groundTex;
      const int dhTex = k < 16 ? ((rawDh * k) >> 4) : rawDh;
      const int v     = (BASE_V_Q8[k] - dhTex * PROJ_SCALE_Q8[k]) >> 8;

      if (v < yMax) {
        const int vTop = v < 0 ? 0 : v;
        uint16_t col = voxColor[mIdx];
        if (k < microMaxK) {
          const int subIdx = ((xQ19 >> 15) & 15) | (((zQ19 >> 15) & 15) << 4);
          const uint8_t mVal = MICRO_TEX_LUT[subIdx];
          if (mVal == 2)      col = col - ((col >> 4) & 0x0861u);
          else if (mVal == 3) col = col + (((col ^ 0xFFFFu) >> 4) & 0x0861u);
        } else if (k > activeSteps - 5 && activeSteps > 24) {
          col = blend565Half(col, fogCol565);
        }
        int ptr = (vTop << 8) | u;
        for (int y = vTop; y < yMax; y++, ptr += VOX_W) {
          voxBuf[ptr] = col;
        }
        yMax = vTop;
      }
    }

    // Cielo + Nubes usando LUT entera CLOUD_DIST_Q8 (cero divisiones)
    if (yMax > 0) {
      int ptr = u;
      for (int y = 0; y < yMax; y++, ptr += VOX_W) {
        const int dyHoriz = horizonV - y;
        if (dyHoriz <= 2) {
          voxBuf[ptr] = fogCol565;
        } else {
          const int lutIdx       = dyHoriz < 511 ? dyHoriz : 511;
          const int32_t sCloudQ8 = CLOUD_DIST_Q8[lutIdx];
          const int cmx = ((skyStartX_Q16 + dirWX_Q8 * sCloudQ8) >> 16) & SKY_MASK;
          const int cmz = ((skyStartZ_Q16 + dirWZ_Q8 * sCloudQ8) >> 16) & SKY_MASK;
          uint16_t skyCol = voxSkyMap[(cmz << SKY_SHIFT) | cmx];
          if (dyHoriz < 36) {
            skyCol = blend565Half(skyCol, fogCol565);
            if (dyHoriz < 16) skyCol = blend565Half(skyCol, fogCol565);
          }
          voxBuf[ptr] = skyCol;
        }
      }
    }
  }

  // 4. Blit Rotado en Punto Fijo 16.16 hacia vram565 (320x240)
  const int32_t du_dx = (int32_t)( cosR * BLIT_SCALE * 65536.0f);
  const int32_t dv_dx = (int32_t)(-sinR * BLIT_SCALE * 65536.0f);
  const int32_t du_dy = (int32_t)( sinR * BLIT_SCALE * 65536.0f);
  const int32_t dv_dy = (int32_t)( cosR * BLIT_SCALE * 65536.0f);

  const int32_t centerU = 128 * 65536;
  const int32_t centerV = 128 * 65536 + horizonSubPxQ16;

  int32_t rowU = centerU - 160 * du_dx - 120 * du_dy;
  int32_t rowV = centerV - 160 * dv_dx - 120 * dv_dy;

  int dstIdx = 0;
  for (int sy = 0; sy < DISP_H; sy++) {
    int32_t uFP = rowU;
    int32_t vFP = rowV;
    for (int sx = 0; sx < DISP_W; sx++) {
      const int uRaw = uFP >> 16;
      const int vRaw = vFP >> 16;
      const int uu   = uRaw < 0 ? 0 : (uRaw > 255 ? 255 : uRaw);
      const int vv   = vRaw < 0 ? 0 : (vRaw > 255 ? 255 : vRaw);
      vram565[dstIdx++] = voxBuf[(vv << 8) | uu];
      uFP += du_dx;
      vFP += dv_dx;
    }
    rowU += du_dy;
    rowV += dv_dy;
  }
}

// ============================================================================
// 6. FÍSICA FPV ACRO / HORIZON CON CUATERNIONES A 240 Hz + GATES 3D
// ============================================================================
struct DroneState {
  float px = 0.0f, py = 12.0f, pz = 0.0f;
  float vx = 0.0f, vy = 0.0f,  vz = 0.0f;
  float qw = 1.0f, qx = 0.0f,  qy = 0.0f, qz = 0.0f;
  bool  horizonMode = false;
  int   activeGate  = 0;
  float lapTime     = 0.0f;
};

static DroneState drone;
static bool showHwDiagOverlay = false;

struct Gate3D { float x, y, z, w, h; };
static const Gate3D GATES[8] = {
  {   0.0f, 9.0f,  -45.0f, 12.0f, 8.5f },
  { -35.0f, 9.5f, -105.0f, 12.0f, 8.5f },
  { -95.0f, 11.0f,-130.0f, 12.0f, 8.5f },
  {-145.0f, 10.0f, -75.0f, 12.0f, 8.5f },
  {-135.0f, 9.0f,    0.0f, 12.0f, 8.5f },
  { -80.0f, 12.0f,  55.0f, 12.0f, 8.5f },
  { -15.0f, 10.0f,  45.0f, 12.0f, 8.5f },
  {  15.0f, 8.5f,    5.0f, 12.0f, 8.5f }
};

static void resetDrone() {
  drone.px = 0.0f;
  drone.pz = 25.0f;
  drone.py = getTerrainHeightMeters(drone.px, drone.pz) + 6.0f;
  drone.vx = drone.vy = drone.vz = 0.0f;
  drone.qw = 1.0f; drone.qx = drone.qy = drone.qz = 0.0f;
  drone.activeGate = 0;
  drone.lapTime    = 0.0f;
}

static float applyExpoCurve(float v, float expo) {
  return v * (1.0f - expo) + (v * v * v) * expo;
}

static void updateDronePhysics(float dt) {
  if (hwInput.x && !hwInput.prevX) {
    resetDrone();
  }
  if (hwInput.y && !hwInput.prevY) {
    drone.horizonMode = !drone.horizonMode;
  }
  if (hwInput.select && !hwInput.prevSelect) {
    showHwDiagOverlay = !showHwDiagOverlay;
  }

  // 4 Sub-pasos de integración de cuaterniones a 240 Hz
  const int subSteps = 4;
  const float subDt  = dt / (float)subSteps;

  // Throttle en stick autocentrado (centro = 42% hover; botón A suma gas, B frena)
  float thrCmd = hwInput.ly >= 0.0f
    ? (0.42f + hwInput.ly * 0.58f)
    : (0.42f * (1.0f + hwInput.ly));
  if (hwInput.a) thrCmd = fminf(1.0f, thrCmd + 0.35f);
  if (hwInput.b) thrCmd = fmaxf(0.0f, thrCmd - 0.30f);

  const float rateRoll  = applyExpoCurve(hwInput.rx, 0.35f) * 5.2f;
  const float ratePitch = applyExpoCurve(hwInput.ry, 0.35f) * 4.8f;
  const float rateYaw   = -applyExpoCurve(hwInput.lx, 0.25f) * 3.8f;

  for (int s = 0; s < subSteps; s++) {
    float wx = ratePitch;
    float wy = rateYaw;
    float wz = -rateRoll;

    if (drone.horizonMode) {
      // Auto-nivelación suave hacia la vertical del mundo
      const float upX = 2.0f * (drone.qx * drone.qy - drone.qw * drone.qz);
      const float upZ = 2.0f * (drone.qy * drone.qz + drone.qw * drone.qx);
      wx -= upZ * 3.5f * (1.0f - fabsf(hwInput.ry));
      wz += upX * 3.5f * (1.0f - fabsf(hwInput.rx));
    }

    const float dqW = 0.5f * (-drone.qx * wx - drone.qy * wy - drone.qz * wz) * subDt;
    const float dqX = 0.5f * ( drone.qw * wx + drone.qy * wz - drone.qz * wy) * subDt;
    const float dqY = 0.5f * ( drone.qw * wy - drone.qx * wz + drone.qz * wx) * subDt;
    const float dqZ = 0.5f * ( drone.qw * wz + drone.qx * wy - drone.qy * wx) * subDt;

    drone.qw += dqW; drone.qx += dqX; drone.qy += dqY; drone.qz += dqZ;
    const float qNorm = 1.0f / sqrtf(drone.qw*drone.qw + drone.qx*drone.qx + drone.qy*drone.qy + drone.qz*drone.qz);
    drone.qw *= qNorm; drone.qx *= qNorm; drone.qy *= qNorm; drone.qz *= qNorm;

    // Vector Up del chasis (dirección del empuje de los 4 motores)
    const float ux = 2.0f * (drone.qx * drone.qy - drone.qw * drone.qz);
    const float uy = 1.0f - 2.0f * (drone.qx * drone.qx + drone.qz * drone.qz);
    const float uz = 2.0f * (drone.qy * drone.qz + drone.qw * drone.qx);

    const float thrustAcc = thrCmd * 24.5f;
    const float drag      = 0.42f;
    drone.vx += (ux * thrustAcc - drone.vx * drag) * subDt;
    drone.vy += (uy * thrustAcc - 9.81f - drone.vy * drag) * subDt;
    drone.vz += (uz * thrustAcc - drone.vz * drag) * subDt;

    drone.px += drone.vx * subDt;
    drone.py += drone.vy * subDt;
    drone.pz += drone.vz * subDt;

    const float floorY = getTerrainHeightMeters(drone.px, drone.pz) + 1.2f;
    if (drone.py < floorY) {
      drone.py = floorY;
      if (drone.vy < 0.0f) drone.vy = 0.0f;
    }
  }

  drone.lapTime += dt;
  const Gate3D& g = GATES[drone.activeGate];
  const float dGate = hypotf(drone.px - g.x, drone.pz - g.z);
  if (dGate < g.w * 0.65f) {
    drone.activeGate = (drone.activeGate + 1) & 7;
  }
}

// ============================================================================
// 7. BUCLE PRINCIPAL ARDUINO / ESP-IDF EN ESP32-S3 N16R8
// ============================================================================
static uint32_t lastFrameMicros = 0;
static float    fpsFiltered     = 60.0f;

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.setRotation(1);
  tft.setBrightness(235);
  tft.fillScreen(0x0000);

  if (!halInitConsole()) {
    tft.setTextColor(0xF800, 0x0000);
    tft.setCursor(10, 100);
    tft.println("ERROR: FALLO MALLOC SRAM/PSRAM OCTAL");
    tft.println("Verifica board_build.arduino.memory_type = qio_opi");
    while (true) { delay(500); }
  }

  initVoxelFixedPointLutsAndWorld();
  resetDrone();
  tft.startWrite(); // Mantener CS activo para transferencias DMA continuas
  lastFrameMicros = micros();
}

void loop() {
  const uint32_t nowMicros = micros();
  float dt = (float)(nowMicros - lastFrameMicros) * 1e-6f;
  lastFrameMicros = nowMicros;
  if (dt <= 0.001f || dt > 0.1f) dt = 0.016f;
  fpsFiltered = fpsFiltered * 0.92f + (1.0f / dt) * 0.08f;

  // 1. Escanear Joysticks ADC1, 12 Botones y Touch FT6336U
  halPollInputs();

  // 2. Actualizar física de vuelo a 240 Hz
  updateDronePhysics(dt);

  // 3. Base ortonormal de la cámara con Camera Uptilt de 20°
  const float qx = drone.qx, qy = drone.qy, qz = drone.qz, qw = drone.qw;
  const float rx = 1.0f - 2.0f * (qy*qy + qz*qz);
  const float ry = 2.0f * (qx*qy + qw*qz);
  const float rz = 2.0f * (qx*qz - qw*qy);

  const float ux = 2.0f * (qx*qy - qw*qz);
  const float uy = 1.0f - 2.0f * (qx*qx + qz*qz);
  const float uz = 2.0f * (qy*qz + qw*qx);

  // En nuestro sistema local Forward mira hacia -Z
  const float fx = -2.0f * (qx*qz + qw*qy);
  const float fy = -2.0f * (qy*qz - qw*qx);
  const floatfz  = -(1.0f - 2.0f * (qx*qx + qy*qy));

  constexpr float uptiltRad = 20.0f * ((float)M_PI / 180.0f);
  const float cT = cosf(uptiltRad), sT = sinf(uptiltRad);
  const float cUx = cT * ux - sT * fx;
  const float cUy = cT * uy - sT * fy;
  const float cUz = cT * uz - sT * floatfz;
  const float cFx = sT * ux + cT * fx;
  const float cFy = sT * uy + cT * fy;
  const float cFz = sT * uz + cT * floatfz;

  // 4. Renderizar Terreno Voxel + Cielo con Nubes en vram565 (Punto Fijo Int32)
  renderVoxelTerrainAndSky(
    drone.px, drone.py, drone.pz,
    rx, ry, rz,
    cUx, cUy, cUz,
    cFx, cFy, cFz,
    dt
  );

  // 5. Proyectar las 8 Puertas 3D del Circuito sobre vram565
  for (int i = 0; i < 8; i++) {
    const Gate3D& g = GATES[i];
    const float gFloor = getTerrainHeightMeters(g.x, g.z);
    const float yBot   = gFloor + 1.5f;
    const float yTop   = gFloor + g.y + g.h * 0.5f;
    const float hw     = g.w * 0.5f;

    const float corners[4][3] = {
      { g.x - hw, yBot, g.z },
      { g.x + hw, yBot, g.z },
      { g.x + hw, yTop, g.z },
      { g.x - hw, yTop, g.z }
    };
    float sx[4], sy[4];
    bool vis = true;
    for (int c = 0; c < 4; c++) {
      const float dx = corners[c][0] - drone.px;
      const float dy = corners[c][1] - drone.py;
      const float dz = corners[c][2] - drone.pz;
      const float cx = dx * rx  + dy * ry  + dz * rz;
      const float cy = dx * cUx + dy * cUy + dz * cUz;
      const float cz = dx * cFx + dy * cFy + dz * cFz;
      if (cz < 0.5f || cz > 650.0f) { vis = false; break; }
      sx[c] = 160.0f + (cx * FOCAL_240) / cz;
      sy[c] = 120.0f - (cy * FOCAL_240) / cz;
    }
    if (vis) {
      const uint16_t col = (i == drone.activeGate) ? C565::GREEN : C565::RED;
      fbDrawLine320(sx[0], sy[0], sx[3], sy[3], col);
      fbDrawLine320(sx[1], sy[1], sx[2], sy[2], col);
      fbDrawLine320(sx[3], sy[3], sx[2], sy[2], (i == drone.activeGate) ? C565::GOLD : col);
    }
  }

  // 6. Dibujar OSD Estilo INAV en 1 Sola Pasada Lineal (2-Bit MAX7456 ROM)
  char buf[64];
  const float speedKmh = hypotf(hypotf(drone.vx, drone.vy), drone.vz) * 3.6f;
  const float aglM     = fmaxf(0.0f, drone.py - getTerrainHeightMeters(drone.px, drone.pz));

  snprintf(buf, sizeof(buf), "S3 N16R8 %s", drone.horizonMode ? "HORIZON" : "ACRO");
  drawInavSmallText320(buf, 8, 6, C565::GOLD);

  snprintf(buf, sizeof(buf), "FPS:%d", (int)lroundf(fpsFiltered));
  drawInavSmallText320(buf, 272, 6, C565::GREEN);

  snprintf(buf, sizeof(buf), "SPD:%dKMH", (int)lroundf(speedKmh));
  drawInavSmallText320(buf, 8, 224, C565::WHITE);

  snprintf(buf, sizeof(buf), "ALT:%dM GATE:%d/8", (int)lroundf(aglM), drone.activeGate + 1);
  drawInavSmallText320(buf, 196, 224, C565::CYAN);

  // 7. Panel de Diagnóstico Hardware en Vivo (Al pulsar SELECT)
  if (showHwDiagOverlay) {
    snprintf(buf, sizeof(buf), "ADC1 L:%d,%d R:%d,%d", hwInput.rawLX, hwInput.rawLY, hwInput.rawRX, hwInput.rawRY);
    drawInavSmallText320(buf, 16, 36, C565::CYAN);

    snprintf(buf, sizeof(buf), "BTN U:%d D:%d L:%d R:%d A:%d B:%d X:%d Y:%d",
      hwInput.up, hwInput.down, hwInput.left, hwInput.right,
      hwInput.a, hwInput.b, hwInput.x, hwInput.y);
    drawInavSmallText320(buf, 16, 48, C565::GOLD);

    snprintf(buf, sizeof(buf), "TOUCH:%d (%d,%d) L3:%d R3:%d",
      hwInput.touchDown, hwInput.touchX, hwInput.touchY, hwInput.l3, hwInput.r3);
    drawInavSmallText320(buf, 16, 60, C565::GREEN);

    const uint32_t freeSramKb  = heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024;
    const uint32_t freePsramKb = heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024;
    snprintf(buf, sizeof(buf), "FREE SRAM:%uKB PSRAM:%uKB", (unsigned)freeSramKb, (unsigned)freePsramKb);
    drawInavSmallText320(buf, 16, 72, C565::WHITE);
  }

  // 8. Transferencia SPI2 DMA de vram565 hacia el panel IPS ILI9341V
  tft.pushImageDMA(0, 0, DISP_W, DISP_H, vram565);
}
