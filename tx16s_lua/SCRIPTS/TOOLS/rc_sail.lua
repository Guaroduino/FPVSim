-- TNS|RC Sailboat|TNE
-- ============================================================================
-- RC SAILBOAT LOS SIMULATOR (IOM 1m / DF65 / RG65) PARA RADIOMASTER TX16S
-- ============================================================================
-- Port 1:1 de las fisicas navales v53 de index.html:
--   - Modelo aerodinamico polar de vela individual (computeSingleSailForce):
--     Sustentacion laminar (AoA 5.5..33 deg) + transicion suave a empuje por
--     presion normal en perdida/popa (>33 deg) + arrastre parasito e inducido.
--   - Mayor (Mainsail) y Foque (Jib) independientes con embolsamiento (camber)
--     dinamico a sotavento (mainLeeR/F, jibLeeR/F) y torsion de baluma.
--   - Hidrodinamica de orza (Keel lift kKeel = 7.0 * keelArea, abatimiento
--     leeway y arrastre inducido) + resistencia viscosa y de ola (hullSpeed).
--   - Timon con perdida hidrodinamica (critRudRad) y flujo minimo rudU2.
--   - Escala inercial GAME_INERTIA = 1.6, momento adrizante del bulbo de plomo
--     (9.81 * GM * sin(heel - waveSlopeRoll)), cabeceo/escora por pendiente de
--     ola y rachas visibles sobre el agua (Catspaws / sailGustAt).
--   - 3 Presets (IOM 1M PRO, DF65 AGILE, RG65 CARBON), 5 Circuitos de Regata,
--     2 rivales IA (AI1, AI2), colision deslizante con costa e islas,
--     vectores de fuerza 3D y camara LOS Auto-Zoom / Manual / Follow 3D.
-- ============================================================================

local sin, cos, tan, asin, acos, sqrt, abs, floor, min, max =
      math.sin, math.cos, math.tan, math.asin, math.acos, math.sqrt, math.abs, math.floor, math.min, math.max
local atan = math.atan2 or math.atan
local PI = math.pi
local TWO_PI = PI * 2.0
local DEG2RAD = PI / 180.0
local RAD2DEG = 180.0 / PI

local drawLine = lcd.drawLine
local drawFilledTriangle = lcd.drawFilledTriangle
local drawFilledRectangle = lcd.drawFilledRectangle
local drawRectangle = lcd.drawRectangle
local drawCircle = lcd.drawCircle
local drawFilledCircle = lcd.drawFilledCircle
local drawText = lcd.drawText
local drawNumber = lcd.drawNumber
local RGB = lcd.RGB

-- ============================================================================
-- PALETA RGB565 PRE-COMPUTADA
-- ============================================================================
local COL_SKY_TOP   = RGB(36, 98, 184)
local COL_SKY_MID   = RGB(86, 152, 224)
local COL_SKY_HOR   = RGB(164, 212, 248)
local COL_SEA_DEEP  = RGB(12, 58, 108)
local COL_SEA_MID   = RGB(18, 76, 134)
local COL_SEA_NEAR  = RGB(24, 92, 154)
local COL_PUFF_CORE = RGB(8, 38, 76)
local COL_PUFF_EDGE = RGB(12, 48, 92)
local COL_WAVE_CREST= RGB(52, 132, 196)
local COL_WAVE_FOAM = RGB(160, 216, 246)
local COL_WAKE      = RGB(220, 244, 255)

local COL_SAND      = RGB(214, 192, 136)
local COL_GRASS_D   = RGB(46, 114, 44)
local COL_GRASS_L   = RGB(68, 148, 56)
local COL_PIER      = RGB(132, 92, 54)

local COL_HULL_PORT = RGB(220, 38, 38)
local COL_HULL_STBD = RGB(175, 24, 24)
local COL_DECK      = RGB(244, 244, 248)
local COL_KEEL      = RGB(60, 68, 78)
local COL_BULB      = RGB(250, 204, 21)
local COL_RUDDER    = RGB(245, 158, 11)
local COL_MAST      = RGB(210, 218, 228)
local COL_SAIL_FILL = RGB(242, 246, 252)
local COL_SAIL_SHAD = RGB(212, 224, 238)
local COL_SAIL_LUFF = RGB(250, 204, 21)
local COL_SEAM_RED  = RGB(235, 65, 55)
local COL_SEAM_BLUE = RGB(59, 130, 246)

local COL_VEC_LIFT  = RGB(34, 197, 94)
local COL_VEC_DRAG  = RGB(239, 68, 68)
local COL_VEC_KEEL  = RGB(56, 189, 248)
local COL_VEC_WIND  = RGB(56, 189, 248)

local COL_WHITE     = RGB(255, 255, 255)
local COL_BLACK     = RGB(0, 0, 0)
local COL_YELLOW    = RGB(255, 224, 32)
local COL_CYAN      = RGB(64, 234, 255)
local COL_GREEN     = RGB(48, 235, 86)
local COL_RED       = RGB(255, 64, 64)
local COL_ORANGE    = RGB(255, 155, 32)
local COL_HUD_BG    = RGB(10, 22, 38)
local COL_MENU_SEL  = RGB(28, 64, 108)

-- ============================================================================
-- PRESETS DE VELEROS RC (1:1 CON SAIL_PRESETS EN INDEX.HTML)
-- ============================================================================
local SAIL_PRESETS = {
  {
    name = "IOM 1M PRO",
    mass = 4.0, sailArea = 0.62, keelArea = 0.042, hullSpeed = 2.65,
    gm = 0.34, mastH = 1.65, hullL = 1.00, hullW = 0.23,
    rudderRate = 1.00, turnDrag = 0.55, tackAssist = 0.25,
    desc = "Estable, alta inercia y gran ceñida"
  },
  {
    name = "DF65 AGILE",
    mass = 1.35, sailArea = 0.26, keelArea = 0.022, hullSpeed = 2.25,
    gm = 0.14, mastH = 1.05, hullL = 0.65, hullW = 0.16,
    rudderRate = 1.38, turnDrag = 0.38, tackAssist = 0.38,
    desc = "Ligero, muy agil en boyas y viradas"
  },
  {
    name = "RG65 CARBON",
    mass = 1.05, sailArea = 0.31, keelArea = 0.024, hullSpeed = 2.48,
    gm = 0.12, mastH = 1.15, hullL = 0.65, hullW = 0.15,
    rudderRate = 1.50, turnDrag = 0.32, tackAssist = 0.42,
    desc = "Regata extrema: planea rapido, escora viva"
  }
}

-- ============================================================================
-- 5 CIRCUITOS DE REGATA (1:1 CON SAIL_COURSES EN INDEX.HTML)
-- ============================================================================
local SAIL_COURSES = {
  {
    name = "1. TRIANGULO OLIMPICO",
    desc = "Clasico de club: Ceñida (1) -> Traves (2) -> Popa (3) -> Meta (4)",
    buoys = {
      { x =  12, z =  42, r = 0.65, col = COL_ORANGE, label = "1" },
      { x = -22, z =  24, r = 0.65, col = COL_YELLOW, label = "2" },
      { x =   0, z =   8, r = 0.65, col = COL_ORANGE, label = "3" },
      { x =  18, z =  12, r = 0.65, col = COL_RED,    label = "4" },
      { x =  -5, z =   4, r = 0.45, col = COL_GREEN,  label = "S1" },
      { x =   5, z =   4, r = 0.45, col = COL_GREEN,  label = "S2" }
    },
    islands = {}
  },
  {
    name = "2. BARLOVENTO-SOTAVENTO",
    desc = "Match Race tactico: Ceñida larga al N (1) + Offset (2) + Puerta (3-4)",
    buoys = {
      { x =   2, z =  52, r = 0.65, col = COL_ORANGE, label = "1" },
      { x =  -8, z =  49, r = 0.55, col = COL_YELLOW, label = "2" },
      { x =  -6, z =  10, r = 0.65, col = COL_ORANGE, label = "3" },
      { x =   6, z =  10, r = 0.65, col = COL_RED,    label = "4" },
      { x =  -5, z =   4, r = 0.45, col = COL_GREEN,  label = "S1" },
      { x =   5, z =   4, r = 0.45, col = COL_GREEN,  label = "S2" }
    },
    islands = {}
  },
  {
    name = "3. SLALOM COSTERO",
    desc = "Zig-zag rapido de traves y trasluchadas cerca del muelle",
    buoys = {
      { x = -24, z =  16, r = 0.65, col = COL_ORANGE, label = "1" },
      { x =  24, z =  26, r = 0.65, col = COL_YELLOW, label = "2" },
      { x = -20, z =  38, r = 0.65, col = COL_ORANGE, label = "3" },
      { x =  20, z =  14, r = 0.65, col = COL_RED,    label = "4" },
      { x =  -5, z =   4, r = 0.45, col = COL_GREEN,  label = "S1" },
      { x =   5, z =   4, r = 0.45, col = COL_GREEN,  label = "S2" }
    },
    islands = {}
  },
  {
    name = "4. GRAN BAHIA LARGA XL",
    desc = "Travesia de fondo hasta 95m mar adentro con vientos racheados",
    buoys = {
      { x =  28, z =  88, r = 0.80, col = COL_ORANGE, label = "1" },
      { x = -38, z =  64, r = 0.80, col = COL_YELLOW, label = "2" },
      { x = -18, z =  18, r = 0.70, col = COL_ORANGE, label = "3" },
      { x =  26, z =  22, r = 0.70, col = COL_RED,    label = "4" },
      { x =  -6, z =   5, r = 0.50, col = COL_GREEN,  label = "S1" },
      { x =   6, z =   5, r = 0.50, col = COL_GREEN,  label = "S2" }
    },
    islands = {}
  },
  {
    name = "5. VUELTA A LA ISLA",
    desc = "Rodea los dos islotes centrales evitando encallar en la arena",
    buoys = {
      { x =  24, z =  56, r = 0.70, col = COL_ORANGE, label = "1" },
      { x = -24, z =  58, r = 0.70, col = COL_YELLOW, label = "2" },
      { x = -26, z =  18, r = 0.65, col = COL_ORANGE, label = "3" },
      { x =  22, z =  16, r = 0.65, col = COL_RED,    label = "4" },
      { x =  -5, z =   4, r = 0.45, col = COL_GREEN,  label = "S1" },
      { x =   5, z =   4, r = 0.45, col = COL_GREEN,  label = "S2" }
    },
    islands = {
      { x =   0, z =  36, r = 9.5, h = 2.2 },
      { x = -12, z =  44, r = 5.5, h = 1.5 }
    }
  }
}

-- ============================================================================
-- ESTADO DEL SIMULADOR
-- ============================================================================
local S = {
  -- Posicion y cinemática del barco
  x = 0.0,
  z = 8.0,
  vx = 0.0,
  vz = 0.0,
  heading = PI * 0.35,
  yawRate = 0.0,
  heel = 0.0,
  heelRate = 0.0,
  pitch = 0.0,

  -- Velas y controles
  boomAngle = 0.0,
  jibAngle = 0.0,
  sheetRatio = 0.35,
  rudderAngle = 0.0,
  mainStall = 0.0,
  jibStall = 0.0,
  sailTrimEff = 0.0,
  mainLeeR = 0.0,
  mainLeeF = 0.0,
  jibLeeR = 0.0,
  jibLeeF = 0.0,
  optSheetRatio = 0.35,

  -- Viento y entorno
  windDir = 0.0,          -- 0 = Viento desde el Norte (+Z) soplando hacia el Sur (-Z)
  windBase = 4.5,         -- m/s (~8.7 kt)
  windSpeed = 4.5,
  gustTimer = 0.0,
  appWindSpeed = 4.5,
  appWindAngle = 0.0,
  trueWindAngle = 0.0,
  vmg = 0.0,
  speed = 0.0,
  leewayDeg = 0.0,

  -- Regata
  presetIdx = 1,
  courseIdx = 1,
  nextMark = 1,
  lap = 1,
  raceTime = 0.0,
  bestLap = 0.0,
  markFlash = 0.0,

  -- Vectores 3D en mundo
  vecLiftX = 0.0, vecLiftZ = 0.0,
  vecDragX = 0.0, vecDragZ = 0.0,
  vecKeelX = 0.0, vecKeelZ = 0.0,

  -- Camara: 1 = LOS Auto-Zoom, 2 = LOS Manual Zoom, 3 = Follow 3D
  camMode = 1,
  manualFov = 28.0,
  autoFov = 32.0,
  camYaw = 0.0,
  camPitch = -0.18,
  showVectors = true,
  showGuides = true,
  halfRes = true,

  -- Menu
  menuOpen = false,
  menuCursor = 1,

  -- Telemetria FPS
  lastTick = 0,
  fps = 30
}

-- Estela (ring buffer pre-asignado sin basura GC por frame) y rivales IA
local WAKE_MAX = 12
local wakeHistory = {}
for i = 1, WAKE_MAX do
  wakeHistory[i] = { x = 0.0, z = 0.0, rx = 1.0, rz = 0.0, spd = 0.0 }
end
local wakeHead = 1
local wakeCount = 0
local wakeTimer = 0.0

local AI_BOATS = {
  { x = -3.5, z = 6.5, vx = 0, vz = 0, heading = 0.8, heel = 0, boom = 0.4, nextMark = 1, col = COL_CYAN,   skill = 0.92, tackDir =  1, name = "AI1" },
  { x =  3.5, z = 6.5, vx = 0, vz = 0, heading = -0.8, heel = 0, boom = -0.4, nextMark = 1, col = COL_ORANGE, skill = 0.84, tackDir = -1, name = "AI2" }
}

-- ============================================================================
-- UTILIDADES MATEMATICAS Y RACHAS ESPACIALES (CATSPAWS)
-- ============================================================================
local function wrapAngle(a)
  while a > PI do a = a - TWO_PI end
  while a < -PI do a = a + TWO_PI end
  return a
end

local function clamp(v, lo, hi)
  if v < lo then return lo end
  if v > hi then return hi end
  return v
end

-- Campo espacial de rachas (Catspaws) que viaja con el viento hacia sotavento
local function sailGustAt(wx, wz, t, windDir)
  local wSin = sin(windDir)
  local wCos = cos(windDir)
  local along = wx * wSin + wz * wCos + t * 3.2
  local across = wx * wCos - wz * wSin
  local p1 = sin(along * 0.19 + sin(across * 0.14) * 1.6)
  local p2 = cos(across * 0.23 + along * 0.11 - t * 0.35)
  local raw = (p1 * 0.6 + p2 * 0.4 + 1.0) * 0.5
  local r2 = raw * raw
  return r2 * r2 -- ^3.5 aprox para resaltar manchas de racha definidas
end

-- ============================================================================
-- REINICIO DE BARCO Y REGATA
-- ============================================================================
local function resetSailboat()
  S.x = 0.0
  S.z = 8.0
  S.vx = 0.0
  S.vz = 0.0
  S.heading = PI * 0.38
  S.yawRate = 0.0
  S.heel = 0.0
  S.heelRate = 0.0
  S.pitch = 0.0
  S.boomAngle = 0.35
  S.jibAngle = 0.38
  S.nextMark = 1
  S.lap = 1
  S.raceTime = 0.0
  S.markFlash = 0.0
  S.camYaw = 0.0
  S.camPitch = -0.18
  S.autoFov = 32.0

  wakeHead = 1
  wakeCount = 0
  wakeTimer = 0.0
  for i = 1, #AI_BOATS do
    local ai = AI_BOATS[i]
    ai.x = (i == 1) and -3.8 or 3.8
    ai.z = 6.2
    ai.vx = 0.0
    ai.vz = 0.0
    ai.heading = (i == 1) and 0.75 or -0.75
    ai.heel = 0.0
    ai.boom = 0.35
    ai.nextMark = 1
    ai.tackDir = (i == 1) and 1 or -1
  end
end

-- ============================================================================
-- LECTURA DE CONTROLES EDGETX (STICKS Y BOTONES)
-- ============================================================================
local function readControls()
  -- AIL (ch1) o RUD (ch4) = Timon (permite volar con una o dos manos)
  local rawAil = getValue("ail") or 0
  local rawRud = getValue("rud") or 0
  local steerRaw = (abs(rawAil) > abs(rawRud)) and rawAil or rawRud
  local steerCmd = clamp(steerRaw / 1024.0, -1.0, 1.0)

  -- THR (ch3) = Winche de Escota (abajo -1024 = cazado 0.03, arriba +1024 = largado 1.0)
  local rawThr = getValue("thr") or -300
  local throttleNorm = clamp((rawThr + 1024.0) / 2048.0, 0.0, 1.0)

  -- ELE (ch2) = Ajuste fino de escota o Zoom manual
  local rawEle = getValue("ele") or 0
  local eleCmd = clamp(rawEle / 1024.0, -1.0, 1.0)

  return steerCmd, throttleNorm, eleCmd
end

-- ============================================================================
-- MODELO AERODINAMICO DE UNA VELA (1:1 CON computeSingleSailForce EN INDEX.HTML)
-- ============================================================================
local function computeSingleSailForce(sailAngle, areaScale, luffRad, optRad, stallRad, appVx, appVz, appWindSpd, dirX, dirZ, rightX, rightZ, rhoAir, presetSailArea, cosHeel, speedFactor)
  local cB = cos(sailAngle)
  local sB = sin(sailAngle)
  local chordX = -dirX * cB + rightX * sB
  local chordZ = -dirZ * cB + rightZ * sB
  local windCrossChord = appVx * chordZ - appVz * chordX
  local windDotChord   = appVx * chordX + appVz * chordZ
  local aoa = atan(windCrossChord, max(0.001, windDotChord))
  local absAoa = abs(aoa)
  local signAoa = (aoa >= 0) and 1.0 or -1.0

  local cl = 0.0
  local cd = 0.04
  local stall = 0.0
  local trim = 0.0

  if absAoa < luffRad then
    -- Flameo (Luffing)
    cl = 0.0
    cd = 0.05
    stall = 0.0
    trim = 0.0
  elseif absAoa < stallRad then
    -- Flujo laminar adjunto (Sustentacion de ala)
    local norm = (absAoa - luffRad) / (optRad - luffRad)
    if norm <= 1.0 then
      cl = 1.55 * sin(norm * PI * 0.5)
      trim = norm
    else
      local over = (absAoa - optRad) / (stallRad - optRad)
      cl = 1.55 * (1.0 - 0.28 * over * over)
      trim = max(0.25, 1.0 - over * 0.85)
    end
    cd = 0.04 + (cl * cl) / (PI * 3.8 * 0.8)
    stall = 0.0
  else
    -- Perdida / Empuje por presion en popa (Stall / Running plate drag)
    local stallOver = min(1.0, (absAoa - stallRad) / (45.0 * DEG2RAD))
    cl = 1.12 * cos(absAoa) * (1.0 - 0.35 * stallOver)
    cd = 0.18 + 1.28 * sin(absAoa) * sin(absAoa)
    stall = stallOver
    trim = max(0.15, (1.0 - stallOver) * 0.6)
  end

  local qAero = 0.5 * rhoAir * (presetSailArea * areaScale) * appWindSpd * appWindSpd * cosHeel * speedFactor
  local liftMag = qAero * cl
  local dragMag = qAero * cd

  local wDirX = (appWindSpd > 0.05) and (appVx / appWindSpd) or 0.0
  local wDirZ = (appWindSpd > 0.05) and (appVz / appWindSpd) or -1.0
  local liftDirX = -wDirZ * signAoa
  local liftDirZ =  wDirX * signAoa

  local fx = liftMag * liftDirX + dragMag * wDirX
  local fz = liftMag * liftDirZ + dragMag * wDirZ
  local fNorm = fx * (-chordZ) + fz * chordX

  return fx, fz, liftMag, dragMag, liftDirX, liftDirZ, wDirX, wDirZ, stall, trim, fNorm
end

-- ============================================================================
-- PASO DE FISICA 1:1 (stepSailboatPhysics DE INDEX.HTML)
-- ============================================================================
local function stepPhysics(dt)
  local preset = SAIL_PRESETS[S.presetIdx]
  local course = SAIL_COURSES[S.courseIdx]
  local steerCmd, throttleNorm, eleCmd = readControls()

  S.raceTime = S.raceTime + dt
  S.gustTimer = S.gustTimer + dt
  if S.markFlash > 0 then S.markFlash = max(0.0, S.markFlash - dt) end

  -- Zoom manual si estamos en modo 2
  if S.camMode == 2 and abs(eleCmd) > 0.08 then
    S.manualFov = clamp(S.manualFov + eleCmd * 22.0 * dt, 6.0, 55.0)
  end

  -- 1. Viento real con racha espacial (Catspaw)
  local localPuff = sailGustAt(S.x, S.z, S.gustTimer, S.windDir)
  local shift = 0.09 * sin(S.gustTimer * 0.32)
  local curWindDir = S.windDir + shift
  S.windSpeed = max(1.5, S.windBase * (0.85 + 0.42 * localPuff))

  -- Vector viento real en mundo
  local twVx = -sin(curWindDir) * S.windSpeed
  local twVz = -cos(curWindDir) * S.windSpeed

  -- 2. Viento Aparente (V_app = V_true - V_boat)
  local appVx = twVx - S.vx
  local appVz = twVz - S.vz
  S.appWindSpeed = sqrt(appVx * appVx + appVz * appVz)

  -- Vectores unitarios del casco
  local dirX = sin(S.heading)
  local dirZ = cos(S.heading)
  local rightX =  dirZ
  local rightZ = -dirX

  -- Angulo de viento aparente respecto a proa
  local appFromX = -appVx
  local appFromZ = -appVz
  local localAppZ = appFromX * dirX + appFromZ * dirZ
  local localAppX = appFromX * rightX + appFromZ * rightZ
  S.appWindAngle = atan(localAppX, localAppZ)

  local twFromX = -twVx
  local twFromZ = -twVz
  S.trueWindAngle = atan(twFromX * rightX + twFromZ * rightZ, twFromX * dirX + twFromZ * dirZ)

  -- 3. Controles: Timon y Winche de Escota
  local targetRudder = -steerCmd * (38.0 * DEG2RAD)
  S.rudderAngle = S.rudderAngle + (targetRudder - S.rudderAngle) * min(1.0, dt * 12.0)

  local targetSheet = 0.03 + 0.97 * throttleNorm
  S.sheetRatio = S.sheetRatio + (targetSheet - S.sheetRatio) * min(1.0, dt * 6.0)
  local maxBoomRad  = (10.0 + S.sheetRatio * 75.0) * DEG2RAD
  local maxJibRad   = (12.0 + S.sheetRatio * 72.0) * DEG2RAD

  -- Guia de escota optima segun angulo de viento aparente
  local absAppDeg = abs(S.appWindAngle) * RAD2DEG
  S.optSheetRatio = clamp((absAppDeg - 32.0) / 135.0, 0.05, 0.95)

  -- 4. Dinamica de Botavara (Mayor) y Foque (Jib) empujados por el viento aparente
  local naturalBoom = clamp( -S.appWindAngle * 0.88, -maxBoomRad, maxBoomRad )
  local naturalJib  = clamp( -S.appWindAngle * 0.92, -maxJibRad,  maxJibRad  )
  S.boomAngle = S.boomAngle + (naturalBoom - S.boomAngle) * min(1.0, dt * 9.5)
  S.jibAngle  = S.jibAngle  + (naturalJib  - S.jibAngle)  * min(1.0, dt * 11.0)

  -- 5. Aerodinamica de Mayor (62%) y Foque (38%)
  local rhoAir = 1.225
  local cosHeel = max(0.25, cos(S.heel))
  local speedFactor = 2.5

  local mFx, mFz, mLift, mDrag, mLx, mLz, mWx, mWz, mStall, mTrim, mNorm =
    computeSingleSailForce(S.boomAngle, 0.62, 5.5 * DEG2RAD, 20.0 * DEG2RAD, 33.0 * DEG2RAD,
      appVx, appVz, S.appWindSpeed, dirX, dirZ, rightX, rightZ, rhoAir, preset.sailArea, cosHeel, speedFactor)

  local jFx, jFz, jLift, jDrag, jLx, jLz, jWx, jWz, jStall, jTrim, jNorm =
    computeSingleSailForce(S.jibAngle, 0.38, 5.0 * DEG2RAD, 19.0 * DEG2RAD, 31.0 * DEG2RAD,
      appVx, appVz, S.appWindSpeed, dirX, dirZ, rightX, rightZ, rhoAir, preset.sailArea, cosHeel, speedFactor)

  S.mainStall = mStall
  S.jibStall  = jStall
  S.sailTrimEff = mTrim * 0.62 + jTrim * 0.38

  -- Embolsamiento (camber) suave de cada vela hacia sotavento
  local mSign = (abs(mNorm) > 0.02) and ((mNorm >= 0) and 1.0 or -1.0) or 0.0
  local jSign = (abs(jNorm) > 0.02) and ((jNorm >= 0) and 1.0 or -1.0) or 0.0
  local mCamber = clamp(abs(mNorm) / (1.8 * preset.sailArea), 0.0, 1.0) * mSign
  local jCamber = clamp(abs(jNorm) / (1.4 * preset.sailArea), 0.0, 1.0) * jSign

  local cM = cos(S.boomAngle); local sM = sin(S.boomAngle)
  local cJ = cos(S.jibAngle);  local sJ = sin(S.jibAngle)
  local tgtMainR = cM * mCamber; local tgtMainF = sM * mCamber
  local tgtJibR  = cJ * jCamber; local tgtJibF  = sJ * jCamber
  local camSmooth = min(1.0, dt * 8.5)
  S.mainLeeR = S.mainLeeR + (tgtMainR - S.mainLeeR) * camSmooth
  S.mainLeeF = S.mainLeeF + (tgtMainF - S.mainLeeF) * camSmooth
  S.jibLeeR  = S.jibLeeR  + (tgtJibR  - S.jibLeeR)  * camSmooth
  S.jibLeeF  = S.jibLeeF  + (tgtJibF  - S.jibLeeF)  * camSmooth

  -- Vectores para visualizacion 3D
  local totLift = mLift + jLift
  local totDrag = mDrag + jDrag
  S.vecLiftX = mLx * (totLift * 0.35)
  S.vecLiftZ = mLz * (totLift * 0.35)
  S.vecDragX = mWx * (totDrag * 0.35)
  S.vecDragZ = mWz * (totDrag * 0.35)

  local aeroFx = mFx + jFx
  local aeroFz = mFz + jFz
  local aeroFwd = aeroFx * dirX + aeroFz * dirZ
  local aeroLat = aeroFx * rightX + aeroFz * rightZ
  local mainLat = mFx * rightX + mFz * rightZ
  local jibLat  = jFx * rightX + jFz * rightZ

  -- 6. Hidrodinamica del Casco y la Orza (Keel)
  local uFwd = S.vx * dirX + S.vz * dirZ
  local vLat = S.vx * rightX + S.vz * rightZ
  local absU = abs(uFwd)

  -- Sustentacion lateral de la orza oponiendose al abatimiento
  local kKeel = 7.0 * (preset.keelArea / 0.042)
  local keelLatForce = -kKeel * absU * vLat - 12.0 * vLat * abs(vLat)
  local keelInducedDrag = 0.14 * abs(keelLatForce * vLat) / max(0.25, absU)

  S.vecKeelX = rightX * (keelLatForce * 0.25)
  S.vecKeelZ = rightZ * (keelLatForce * 0.25)

  -- Resistencia de casco (viscosa + resistencia de ola Froude)
  local hullRatio = absU / preset.hullSpeed
  local waveMakingDrag = (hullRatio > 0.72) and (3.2 * ((hullRatio - 0.72) ^ 3)) or 0.0
  local rudderDrag = 0.38 * (S.rudderAngle * S.rudderAngle) * (uFwd * uFwd)
  local turnInducedDrag = preset.turnDrag * (S.yawRate * S.yawRate) * absU
  local hydroFwdDrag = -(0.30 * uFwd + 0.42 * uFwd * absU + waveMakingDrag + keelInducedDrag + rudderDrag + turnInducedDrag)

  -- 7. Integracion Lineal (con GAME_INERTIA = 1.6 para respuesta RC natural)
  local GAME_INERTIA = 1.6
  local netFwd = aeroFwd + hydroFwdDrag
  local netLat = aeroLat + keelLatForce

  local du = (netFwd / preset.mass) * GAME_INERTIA * dt
  local dv = (netLat / (preset.mass * 2.1)) * GAME_INERTIA * dt
  local newU = uFwd + du
  local newV = (vLat + dv) * 0.90

  S.vx = dirX * newU + rightX * newV
  S.vz = dirZ * newU + rightZ * newV
  S.x = S.x + S.vx * dt
  S.z = S.z + S.vz * dt
  S.speed = sqrt(S.vx * S.vx + S.vz * S.vz)
  S.leewayDeg = (absU > 0.12) and (atan(newV, newU) * RAD2DEG) or 0.0

  -- VMG hacia el viento real
  local upwindDirX = sin(curWindDir)
  local upwindDirZ = cos(curWindDir)
  S.vmg = S.vx * upwindDirX + S.vz * upwindDirZ

  -- 8. Dinamica de Guiñada (Yaw) con perdida de timon y equilibrio velico
  local rudAngle = S.rudderAngle
  local absRud = abs(rudAngle)
  local critRudRad = 26.0 * DEG2RAD
  local rudEff = rudAngle
  if absRud > critRudRad then
    local over = (absRud - critRudRad) / (15.0 * DEG2RAD)
    local signR = (rudAngle >= 0) and 1.0 or -1.0
    rudEff = signR * (critRudRad - 0.14 * over)
  end

  local rudU2 = uFwd * uFwd + 0.45
  local flowSign = (uFwd >= -0.05) and 1.0 or -0.65
  local fRudLat = 2.65 * preset.rudderRate * rudU2 * rudEff * flowSign
  local mRudder = -1.05 * fRudLat

  local mSails = (-0.11 * mainLat + 0.18 * jibLat)
  local mHeel = -0.22 * sin(S.heel) * aeroFwd
  local mDamp = -1.85 * S.yawRate * (1.0 + 0.9 * absU)
  local izz = preset.mass * 0.34

  local yawAcc = ((mRudder + mSails + mHeel + mDamp) / izz) * GAME_INERTIA
  S.yawRate = clamp(S.yawRate + yawAcc * dt, -2.6, 2.6)
  S.heading = wrapAngle(S.heading + S.yawRate * dt)

  -- 9. Escora (Heel) y Cabeceo (Pitch) acoplados al oleaje 3D
  local wavePhase = S.x * 1.1 + S.z * 1.6 - S.gustTimer * 3.8
  local waveSlopePitch = 0.045 * cos(wavePhase)
  local waveSlopeRoll  = 0.040 * sin(wavePhase * 0.85 + 1.1)

  local heelingMoment = aeroLat * (preset.mastH * 0.38) * cos(S.heel)
  local rightingMoment = -9.81 * preset.mass * preset.gm * sin(S.heel - waveSlopeRoll)
  local heelDamping = -2.2 * S.heelRate * preset.mass * 0.25
  local ixx = preset.mass * 0.18
  S.heelRate = S.heelRate + ((heelingMoment + rightingMoment + heelDamping) / ixx) * dt
  S.heel = clamp(S.heel + S.heelRate * dt, -68.0 * DEG2RAD, 68.0 * DEG2RAD)

  local targetPitch = clamp(aeroFwd * 0.015 - uFwd * 0.02 + waveSlopePitch, -0.18, 0.18)
  S.pitch = S.pitch + (targetPitch - S.pitch) * min(1.0, dt * 6.0)

  -- 10. Colisiones deslizantes con orilla Sur (z < 1.2) e islas
  if S.z < 1.2 then
    S.z = 1.2
    if S.vz < 0 then S.vz = -S.vz * 0.25 end
    S.vx = S.vx * 0.75
  end
  local islands = course.islands
  for i = 1, #islands do
    local isl = islands[i]
    local dx = S.x - isl.x
    local dz = S.z - isl.z
    local d = sqrt(dx * dx + dz * dz)
    local minR = isl.r + 0.65
    if d < minR and d > 0.001 then
      local nx = dx / d
      local nz = dz / d
      S.x = isl.x + nx * minR
      S.z = isl.z + nz * minR
      local vn = S.vx * nx + S.vz * nz
      if vn < 0 then
        S.vx = (S.vx - 1.25 * vn * nx) * 0.75
        S.vz = (S.vz - 1.25 * vn * nz) * 0.75
      end
    end
  end

  -- 11. Paso por Boyas de Regata (1..4)
  local buoys = course.buoys
  local targetBuoy = buoys[S.nextMark]
  if targetBuoy then
    local bdx = S.x - targetBuoy.x
    local bdz = S.z - targetBuoy.z
    if (bdx * bdx + bdz * bdz) < 12.25 then -- radio de validacion 3.5m
      S.markFlash = 1.2
      S.nextMark = S.nextMark + 1
      if S.nextMark > 4 then
        S.nextMark = 1
        if S.bestLap == 0.0 or S.raceTime < S.bestLap then
          S.bestLap = S.raceTime
        end
        S.raceTime = 0.0
        S.lap = S.lap + 1
      end
    end
  end

  -- 12. Historial de Estela (Ring buffer cero-allocations)
  wakeTimer = wakeTimer + dt
  if wakeTimer >= 0.15 then
    wakeTimer = 0.0
    wakeHead = (wakeHead % WAKE_MAX) + 1
    local slot = wakeHistory[wakeHead]
    slot.x = S.x - dirX * 0.48
    slot.z = S.z - dirZ * 0.48
    slot.rx = rightX
    slot.rz = rightZ
    slot.spd = S.speed
    if wakeCount < WAKE_MAX then wakeCount = wakeCount + 1 end
  end

  -- 13. Rivales IA (AI1, AI2) navegando el circuito con ceñida a 44 deg
  for i = 1, #AI_BOATS do
    local ai = AI_BOATS[i]
    local mb = buoys[ai.nextMark] or buoys[1]
    local dx = mb.x - ai.x
    local dz = mb.z - ai.z
    local dist = sqrt(dx * dx + dz * dz)
    if dist < 3.4 then
      ai.nextMark = (ai.nextMark % 4) + 1
    end
    local desiredHdg = atan(dx, dz)
    local relToWind = wrapAngle(desiredHdg - curWindDir)
    if abs(relToWind) < 43.0 * DEG2RAD then
      if dist < 7.0 then ai.tackDir = (relToWind >= 0) and 1 or -1 end
      desiredHdg = curWindDir + ai.tackDir * (44.0 * DEG2RAD)
    end
    local hdgErr = wrapAngle(desiredHdg - ai.heading)
    ai.heading = wrapAngle(ai.heading + clamp(hdgErr * 1.9, -1.6, 1.6) * dt)
    local aiTwa = wrapAngle(curWindDir - ai.heading)
    local absTwa = abs(aiTwa)
    local polar = (absTwa < 36.0 * DEG2RAD) and 0.25 or sin(min(PI * 0.85, absTwa * 1.15))
    local aiSpd = S.windSpeed * 0.42 * polar * ai.skill
    ai.vx = sin(ai.heading) * aiSpd
    ai.vz = cos(ai.heading) * aiSpd
    ai.x = ai.x + ai.vx * dt
    ai.z = ai.z + ai.vz * dt
    ai.heel = clamp(-sin(aiTwa) * 0.42, -0.55, 0.55)
    ai.boom = clamp(-aiTwa * 0.65, -1.2, 1.2)
  end
end

-- ============================================================================
-- MOTOR DE PROYECCION 3D Y RASTERIZACION DMA2D
-- ============================================================================
local camX, camY, camZ = 0.0, 1.75, 0.0
local r00, r01, r02 = 1, 0, 0
local r10, r11, r12 = 0, 1, 0
local r20, r21, r22 = 0, 0, 1
local focalPx = 280.0
local scrW, scrH = 480, 272
local halfW, halfH = 240, 136
local scalePix = 1

local function setupCamera(dt)
  scrW = S.halfRes and 240 or 480
  scrH = S.halfRes and 136 or 272
  scalePix = S.halfRes and 2 or 1
  halfW = scrW * 0.5
  halfH = scrH * 0.5

  if S.camMode == 3 then
    -- MODO 3: FOLLOW 3D (Persecucion detras del barco)
    local dirX = sin(S.heading)
    local dirZ = cos(S.heading)
    camX = S.x - dirX * 4.2
    camY = 1.65
    camZ = S.z - dirZ * 4.2
    local tgtYaw = S.heading
    local tgtPitch = -0.22
    S.camYaw = wrapAngle(S.camYaw + wrapAngle(tgtYaw - S.camYaw) * min(1.0, dt * 8.0))
    S.camPitch = S.camPitch + (tgtPitch - S.camPitch) * min(1.0, dt * 8.0)
    S.autoFov = 46.0
  else
    -- MODO 1 y 2: LOS DESDE EL MUELLE (x=0, y=1.75, z=0)
    camX = 0.0
    camY = 1.75
    camZ = 0.0
    local dx = S.x - camX
    local dy = 0.45 - camY
    local dz = S.z - camZ
    local distXZ = sqrt(dx * dx + dz * dz)
    local dist3D = sqrt(dx * dx + dy * dy + dz * dz)

    local tgtYaw = atan(dx, dz)
    local tgtPitch = atan(dy, max(0.5, distXZ))
    local trackRate = min(1.0, dt * 10.0)
    S.camYaw = wrapAngle(S.camYaw + wrapAngle(tgtYaw - S.camYaw) * trackRate)
    S.camPitch = S.camPitch + (tgtPitch - S.camPitch) * trackRate

    if S.camMode == 1 then
      -- Auto-Zoom suave manteniendo el velero grande y legible
      local idealFov = clamp(240.0 / max(4.0, dist3D), 7.5, 42.0)
      S.autoFov = S.autoFov + (idealFov - S.autoFov) * min(1.0, dt * 5.0)
    else
      S.autoFov = S.manualFov
    end
  end

  local fovRad = S.autoFov * DEG2RAD
  focalPx = halfH / tan(fovRad * 0.5)

  local cy = cos(-S.camYaw);   local sy = sin(-S.camYaw)
  local cp = cos(-S.camPitch); local sp = sin(-S.camPitch)

  r00 = cy;        r01 = 0.0; r02 = sy
  r10 = sy * sp;   r11 = cp;  r12 = -cy * sp
  r20 = -sy * cp;  r21 = sp;  r22 = cy * cp
end

-- Proyecta un punto 3D del mundo a coordenadas de pantalla 480x272
local function projectPoint(wx, wy, wz)
  local dx = wx - camX
  local dy = wy - camY
  local dz = wz - camZ
  local cz = r20 * dx + r21 * dy + r22 * dz
  if cz < 0.35 then return nil, nil, cz end
  local cx = r00 * dx + r01 * dy + r02 * dz
  local cy = r10 * dx + r11 * dy + r12 * dz
  local invZ = focalPx / cz
  local sx = floor((halfW + cx * invZ) * scalePix)
  local sy = floor((halfH - cy * invZ) * scalePix)
  return sx, sy, cz
end

-- ============================================================================
-- RENDERIZADO DEL ENTORNO MARINO, RACHAS, OLAS, ISLAS Y BOYAS
-- ============================================================================
local function drawEnvironment()
  -- Horizonte proyectado segun camPitch
  local horizonY = floor((halfH + tan(S.camPitch) * focalPx) * scalePix)
  horizonY = clamp(horizonY, 0, 272)

  -- Cielo degradado en 3 bandas
  if horizonY > 0 then
    local h1 = floor(horizonY * 0.45)
    local h2 = floor(horizonY * 0.80)
    if h1 > 0 then drawFilledRectangle(0, 0, 480, h1, COL_SKY_TOP) end
    if h2 > h1 then drawFilledRectangle(0, h1, 480, h2 - h1, COL_SKY_MID) end
    drawFilledRectangle(0, h2, 480, horizonY - h2, COL_SKY_HOR)
  end

  -- Mar en 3 bandas de profundidad
  if horizonY < 272 then
    local seaH = 272 - horizonY
    local s1 = horizonY + floor(seaH * 0.28)
    local s2 = horizonY + floor(seaH * 0.65)
    drawFilledRectangle(0, horizonY, 480, s1 - horizonY, COL_SEA_DEEP)
    drawFilledRectangle(0, s1, 480, s2 - s1, COL_SEA_MID)
    drawFilledRectangle(0, s2, 480, 272 - s2, COL_SEA_NEAR)
  end

  -- Orilla de arena y cesped al Sur (z <= 1.2)
  local sx1, sy1 = projectPoint(-45, 0.05, 1.2)
  local sx2, sy2 = projectPoint( 45, 0.05, 1.2)
  local sx3, sy3 = projectPoint( 45, 0.25, -4.0)
  local sx4, sy4 = projectPoint(-45, 0.25, -4.0)
  if sx1 and sx2 and sx3 and sx4 then
    drawFilledTriangle(sx1, sy1, sx2, sy2, sx3, sy3, COL_SAND)
    drawFilledTriangle(sx1, sy1, sx3, sy3, sx4, sy4, COL_SAND)
  end

  -- Malla estatica en el mundo de olas y manchas de racha (Catspaws)
  -- Optimizada a 7x7 puntos con trig precalculado fuera del bucle para 30 FPS clavados
  local step = (S.autoFov < 16.0) and 5.0 or 8.0
  local span = step * 3.0
  local centerX = floor(S.x / step) * step
  local centerZ = floor(S.z / step) * step
  local wSin = sin(S.windDir)
  local wCos = cos(S.windDir)
  local tWind = S.gustTimer * 3.2

  for gz = centerZ - span, centerZ + span, step do
    if gz > 2.0 then
      for gx = centerX - span, centerX + span, step do
        local px, py, pz = projectPoint(gx, 0.0, gz)
        if px and px >= 0 and px < 480 and py > horizonY and py < 270 then
          local along = gx * wSin + gz * wCos + tWind
          local across = gx * wCos - gz * wSin
          local raw = (sin(along * 0.19) * 0.6 + cos(across * 0.23) * 0.4 + 1.0) * 0.5
          local puff = raw * raw * raw
          local wLen = clamp(floor(80.0 / max(4.0, pz)), 3, 16)
          if puff > 0.42 then
            -- Mancha oscura de racha (Catspaw) con rizos de viento
            drawFilledRectangle(px - wLen, py - 1, wLen * 2, 3, (puff > 0.62) and COL_PUFF_CORE or COL_PUFF_EDGE)
            drawLine(px - wLen, py, px + wLen, py, SOLID, COL_WAVE_CREST)
          else
            -- Cresta de ola normal anclada al mundo
            drawLine(px - wLen, py, px + wLen, py, SOLID, COL_WAVE_CREST)
          end
        end
      end
    end
  end

  -- Islas del circuito (si existen, ej. Circuito 5)
  local course = SAIL_COURSES[S.courseIdx]
  for i = 1, #course.islands do
    local isl = course.islands[i]
    local ix, iy, iz = projectPoint(isl.x, 0.0, isl.z)
    local tx, ty = projectPoint(isl.x, isl.h, isl.z)
    local rx, ry = projectPoint(isl.x + isl.r, 0.0, isl.z)
    if ix and tx and rx then
      local radPx = max(6, abs(rx - ix))
      drawFilledTriangle(ix - radPx, iy, ix + radPx, iy, tx, ty - 2, COL_SAND)
      drawFilledTriangle(ix - floor(radPx * 0.78), iy - 1, ix + floor(radPx * 0.78), iy - 1, tx, ty, COL_GRASS_D)
      drawFilledTriangle(ix, iy - 1, ix + floor(radPx * 0.70), iy - 1, tx, ty, COL_GRASS_L)
    end
  end

  -- Estela en V (Wake) detras del barco (ring buffer cero-allocations)
  for age = 0, wakeCount - 2 do
    local idx1 = ((wakeHead - age - 1) % WAKE_MAX) + 1
    local idx2 = ((wakeHead - age - 2) % WAKE_MAX) + 1
    local w1 = wakeHistory[idx1]
    local w2 = wakeHistory[idx2]
    if w1.spd > 0.25 then
      local spread1 = 0.12 + age * 0.09
      local spread2 = 0.12 + (age + 1) * 0.09
      local lx1, ly1 = projectPoint(w1.x - w1.rx * spread1, 0.02, w1.z - w1.rz * spread1)
      local lx2, ly2 = projectPoint(w2.x - w2.rx * spread2, 0.02, w2.z - w2.rz * spread2)
      local rx1, ry1 = projectPoint(w1.x + w1.rx * spread1, 0.02, w1.z + w1.rz * spread1)
      local rx2, ry2 = projectPoint(w2.x + w2.rx * spread2, 0.02, w2.z + w2.rz * spread2)
      local wCol = (age < 4) and COL_WAKE or COL_WAVE_FOAM
      if lx1 and lx2 then drawLine(lx1, ly1, lx2, ly2, SOLID, wCol) end
      if rx1 and rx2 then drawLine(rx1, ry1, rx2, ry2, SOLID, wCol) end
    end
  end

  -- Boyas del Circuito + Anillo de la siguiente boya objetivo
  local buoys = course.buoys
  for i = 1, #buoys do
    local b = buoys[i]
    local bx, by, bz = projectPoint(b.x, 0.0, b.z)
    local tx, ty = projectPoint(b.x, b.r * 1.15, b.z)
    if bx and tx and bx >= -20 and bx <= 500 then
      local radPx = clamp(floor((b.r * focalPx / bz) * scalePix * 0.65), 3, 24)
      drawFilledTriangle(bx - radPx, by, bx + radPx, by, tx, ty, b.col)
      drawLine(bx - radPx, by, bx + radPx, by, SOLID, COL_WHITE)
      if i == S.nextMark then
        -- Indicador visual sobre la boya objetivo activa
        drawCircle(bx, by, radPx + 5, COL_YELLOW)
        drawText(bx - 4, ty - 16, b.label, SMLSIZE + COL_YELLOW)
      else
        drawText(bx - 3, ty - 14, b.label, SMLSIZE + COL_WHITE)
      end
    end
  end
end

-- ============================================================================
-- RENDERIZADO 3D DEL VELERO RC (CASCO, ORZA, BULBO, TIMON, MAYOR Y FOQUE)
-- ============================================================================
local function drawBoat3D(bx, bz, heading, heel, pitch, boomAngle, jibAngle, rudderAngle, mainLeeR, mainLeeF, jibLeeR, jibLeeF, hullColPort, hullColStbd, isPlayer)
  local preset = SAIL_PRESETS[S.presetIdx]
  local scaleL = isPlayer and (preset.hullL / 1.00) or 1.0
  local scaleH = isPlayer and (preset.mastH / 1.65) or 1.0

  local ch = cos(heading); local sh = sin(heading)
  local cr = cos(heel);    local sr = sin(heel)
  local cp = cos(pitch);   local sp = sin(pitch)

  -- Transforma coordenadas locales del barco (lx=babor/estribor, ly=arriba, lz=proa) a pantalla
  local function boatPt(lx, ly, lz)
    lx = lx * scaleL
    ly = ly * scaleH
    lz = lz * scaleL
    -- Roll (escora)
    local x1 = lx * cr - ly * sr
    local y1 = lx * sr + ly * cr
    local z1 = lz
    -- Pitch (cabeceo)
    local x2 = x1
    local y2 = y1 * cp - z1 * sp
    local z2 = y1 * sp + z1 * cp
    -- Yaw (rumbo) + traslacion
    local wx = bx + x2 * ch + z2 * sh
    local wy = 0.04 + y2
    local wz = bz - x2 * sh + z2 * ch
    return projectPoint(wx, wy, wz)
  end

  -- 1. Orza (Keel) y Bulbo de Plomo sumergidos
  local kTopX, kTopY = boatPt(0.0, -0.02, 0.0)
  local kBotX, kBotY = boatPt(0.0, -0.42, -0.03)
  local bFrontX, bFrontY = boatPt(0.0, -0.43, 0.14)
  local bRearX, bRearY   = boatPt(0.0, -0.43, -0.16)
  if kTopX and kBotX then
    drawLine(kTopX, kTopY, kBotX, kBotY, SOLID, COL_KEEL)
    if bFrontX and bRearX then
      drawLine(bFrontX, bFrontY, bRearX, bRearY, SOLID, COL_BULB)
      drawFilledCircle(kBotX, kBotY, 3, COL_BULB)
    end
  end

  -- 2. Pala del Timon movil en popa
  if isPlayer then
    local rTopX, rTopY = boatPt(0.0, -0.02, -0.52)
    local rTipX, rTipY = boatPt(sin(rudderAngle) * 0.14, -0.24, -0.52 - cos(rudderAngle) * 0.12)
    if rTopX and rTipX then
      drawLine(rTopX, rTopY, rTipX, rTipY, SOLID, COL_RUDDER)
    end
  end

  -- 3. Casco (Hull) + Cubierta (Deck)
  local bowX, bowY   = boatPt( 0.00, 0.09,  0.58)
  local portX, portY = boatPt(-0.14, 0.06,  0.02)
  local stbdX, stbdY = boatPt( 0.14, 0.06,  0.02)
  local pStnX, pStnY = boatPt(-0.10, 0.06, -0.52)
  local sStnX, sStnY = boatPt( 0.10, 0.06, -0.52)
  local keelX, keelY = boatPt( 0.00, -0.04, 0.00)

  if bowX and portX and stbdX and pStnX and sStnX and keelX then
    -- Costados del casco
    drawFilledTriangle(bowX, bowY, portX, portY, keelX, keelY, hullColPort)
    drawFilledTriangle(portX, portY, pStnX, pStnY, keelX, keelY, hullColPort)
    drawFilledTriangle(bowX, bowY, keelX, keelY, stbdX, stbdY, hullColStbd)
    drawFilledTriangle(stbdX, stbdY, keelX, keelY, sStnX, sStnY, hullColStbd)
    -- Cubierta blanca
    drawFilledTriangle(bowX, bowY, portX, portY, stbdX, stbdY, COL_DECK)
    drawFilledTriangle(portX, portY, pStnX, pStnY, sStnX, sStnY, COL_DECK)
    drawFilledTriangle(portX, portY, sStnX, sStnY, stbdX, stbdY, COL_DECK)
  end

  -- 4. Mastil, Vela Mayor (Mainsail) y Foque (Jib) con embolsamiento (camber)
  local mBaseX, mBaseY = boatPt(0.0, 0.08, 0.08)
  local mMidX,  mMidY  = boatPt(0.0, 0.86, 0.07)
  local mTopX,  mTopY  = boatPt(0.0, 1.62, 0.05)

  local sB = sin(boomAngle); local cB = cos(boomAngle)
  local clewX, clewY = boatPt(sB * 0.56, 0.13, 0.08 - cB * 0.56)
  local leechMidX, leechMidY = boatPt(
    sB * 0.38 + mainLeeR * 0.12,
    0.88,
    0.07 - cB * 0.38 + mainLeeF * 0.12
  )

  -- Foque (Jib) en proa
  local jTackX, jTackY = boatPt(0.0, 0.09, 0.55)
  local jHeadX, jHeadY = boatPt(0.0, 1.24, 0.06)
  local sJ = sin(jibAngle); local cJ = cos(jibAngle)
  local jClewX, jClewY = boatPt(sJ * 0.38 + jibLeeR * 0.06, 0.15, 0.55 - cJ * 0.38 + jibLeeF * 0.06)

  -- Dibujar Foque
  if jTackX and jHeadX and jClewX then
    local jibCol = (isPlayer and S.jibStall > 0.35) and COL_SAIL_LUFF or COL_SAIL_SHAD
    drawFilledTriangle(jTackX, jTackY, jHeadX, jHeadY, jClewX, jClewY, jibCol)
    drawLine(jTackX, jTackY, jHeadX, jHeadY, SOLID, COL_WHITE)
    if isPlayer then
      -- Franja de camber azul en el foque
      local jMid1X = floor((jTackX + jHeadX) * 0.5)
      local jMid1Y = floor((jTackY + jHeadY) * 0.5)
      drawLine(jMid1X, jMid1Y, jClewX, jClewY, SOLID, COL_SEAM_BLUE)
    end
  end

  -- Dibujar Vela Mayor en 2 paneles para mostrar la curvatura 3D de la baluma
  if mBaseX and mMidX and mTopX and clewX and leechMidX then
    local mainCol = (isPlayer and S.mainStall > 0.35) and COL_SAIL_LUFF or COL_SAIL_FILL
    drawFilledTriangle(mBaseX, mBaseY, mMidX, mMidY, clewX, clewY, mainCol)
    drawFilledTriangle(mMidX, mMidY, leechMidX, leechMidY, clewX, clewY, COL_SAIL_SHAD)
    drawFilledTriangle(mMidX, mMidY, mTopX, mTopY, leechMidX, leechMidY, mainCol)

    -- Botavara y Mastil
    drawLine(mBaseX, mBaseY, clewX, clewY, SOLID, COL_MAST)
    drawLine(mBaseX, mBaseY, mTopX, mTopY, SOLID, COL_MAST)

    -- Franja roja de lectura de camber (draft stripe)
    if isPlayer then
      drawLine(mMidX, mMidY, leechMidX, leechMidY, SOLID, COL_SEAM_RED)
    end
  end

  -- 5. Vectores de Fuerza y Guias sobre el agua (solo para el jugador)
  if isPlayer and S.showVectors then
    local cX, cY = projectPoint(bx, 0.25, bz)
    if cX then
      -- Sustentacion velica (Verde)
      local lx, ly = projectPoint(bx + S.vecLiftX, 0.25, bz + S.vecLiftZ)
      if lx then drawLine(cX, cY, lx, ly, SOLID, COL_VEC_LIFT) end
      -- Arrastre velico (Rojo)
      local dx, dy = projectPoint(bx + S.vecDragX, 0.25, bz + S.vecDragZ)
      if dx then drawLine(cX, cY, dx, dy, SOLID, COL_VEC_DRAG) end
      -- Fuerza lateral de orza (Cian)
      local kx, ky = projectPoint(bx + S.vecKeelX, 0.10, bz + S.vecKeelZ)
      if kx then drawLine(cX, cY, kx, ky, SOLID, COL_VEC_KEEL) end
    end
  end

  -- Guia a la siguiente boya sobre el agua
  if isPlayer and S.showGuides then
    local b = SAIL_COURSES[S.courseIdx].buoys[S.nextMark]
    if b then
      local dx = b.x - bx
      local dz = b.z - bz
      local d = sqrt(dx * dx + dz * dz)
      if d > 1.5 then
        local gx1, gy1 = projectPoint(bx + (dx / d) * 1.1, 0.05, bz + (dz / d) * 1.1)
        local gx2, gy2 = projectPoint(bx + (dx / d) * min(d, 4.5), 0.05, bz + (dz / d) * min(d, 4.5))
        if gx1 and gx2 then
          drawLine(gx1, gy1, gx2, gy2, DOTTED, COL_YELLOW)
        end
      end
    end
  end
end

-- ============================================================================
-- HUD NAUTICO (ROSA DE VIENTOS, INDICADOR DE ESCOTA Y TELEMETRIA)
-- ============================================================================
local function drawHud()
  local preset = SAIL_PRESETS[S.presetIdx]
  local course = SAIL_COURSES[S.courseIdx]

  -- Barra superior compacta
  drawFilledRectangle(0, 0, 480, 20, COL_HUD_BG)
  local knots = S.speed * 1.94384
  local windKt = S.windSpeed * 1.94384
  local vmgKt = S.vmg * 1.94384

  drawText(6, 2, preset.name, SMLSIZE + COL_CYAN)
  drawText(112, 2, string.format("SPD:%.1fkt", knots), SMLSIZE + COL_WHITE)
  drawText(192, 2, string.format("VMG:%+.1f", vmgKt), SMLSIZE + (vmgKt >= 0 and COL_GREEN or COL_ORANGE))
  drawText(268, 2, string.format("TWS:%.1fkt", windKt), SMLSIZE + COL_YELLOW)
  drawText(350, 2, string.format("BOYA:%d/4 L:%d", S.nextMark, S.lap), SMLSIZE + COL_WHITE)
  drawText(438, 2, string.format("%dFPS", S.fps), SMLSIZE + (S.fps >= 25 and COL_GREEN or COL_ORANGE))

  -- 1. Rosa de Vientos y Rumbo del Barco (Arriba Izquierda)
  local cx, cy, rad = 44, 64, 30
  drawFilledCircle(cx, cy, rad + 2, COL_HUD_BG)
  drawCircle(cx, cy, rad, COL_WHITE)

  -- Sector No-Go Zone (+/- 40 deg respecto al viento real)
  local wScreen = S.windDir - S.camYaw
  local ng1x = cx + floor(sin(wScreen - 40 * DEG2RAD) * rad)
  local ng1y = cy - floor(cos(wScreen - 40 * DEG2RAD) * rad)
  local ng2x = cx + floor(sin(wScreen + 40 * DEG2RAD) * rad)
  local ng2y = cy - floor(cos(wScreen + 40 * DEG2RAD) * rad)
  drawLine(cx, cy, ng1x, ng1y, DOTTED, COL_RED)
  drawLine(cx, cy, ng2x, ng2y, DOTTED, COL_RED)

  -- Flecha de Viento Real (Azul Cian entrando hacia el centro)
  local wx1 = cx + floor(sin(wScreen) * (rad + 5))
  local wy1 = cy - floor(cos(wScreen) * (rad + 5))
  local wx2 = cx + floor(sin(wScreen) * (rad - 12))
  local wy2 = cy - floor(cos(wScreen) * (rad - 12))
  drawLine(wx1, wy1, wx2, wy2, SOLID, COL_CYAN)
  drawText(cx - 8, cy + rad + 4, "WIND", SMLSIZE + COL_CYAN)

  -- Silueta del barco y su botavara dentro de la rosa
  local bScreen = S.heading - S.camYaw
  local bxProa = cx + floor(sin(bScreen) * 16)
  local byProa = cy - floor(cos(bScreen) * 16)
  local bxPopa = cx - floor(sin(bScreen) * 12)
  local byPopa = cy + floor(cos(bScreen) * 12)
  drawLine(bxPopa, byPopa, bxProa, byProa, SOLID, COL_WHITE)
  local boomScr = bScreen + PI + S.boomAngle
  local bmX = cx + floor(sin(boomScr) * 14)
  local bmY = cy - floor(cos(boomScr) * 14)
  drawLine(cx, cy, bmX, bmY, SOLID, COL_YELLOW)

  -- 2. Barra de Winche de Escota (Sheet Trim) vs Marca Optima (Abajo Izquierda)
  local barX, barY, barW, barH = 12, 246, 140, 12
  drawFilledRectangle(barX, barY, barW, barH, COL_HUD_BG)
  drawRectangle(barX, barY, barW, barH, COL_WHITE)
  local fillW = floor(S.sheetRatio * (barW - 2))
  local trimCol = (S.sailTrimEff > 0.70) and COL_GREEN or ((S.mainStall > 0.35) and COL_RED or COL_YELLOW)
  drawFilledRectangle(barX + 1, barY + 1, fillW, barH - 2, trimCol)

  -- Marca vertical blanca de escota optima para el rumbo actual
  local optX = barX + 1 + floor(S.optSheetRatio * (barW - 2))
  drawLine(optX, barY - 3, optX, barY + barH + 3, SOLID, COL_WHITE)
  drawText(barX, barY - 16, string.format("ESCOTA %d%% (EFF %d%%)", floor(S.sheetRatio * 100), floor(S.sailTrimEff * 100)), SMLSIZE + COL_WHITE)

  -- 3. Estado del Reglaje / Rumbo (Abajo Centro)
  local absTwaDeg = abs(S.trueWindAngle) * RAD2DEG
  local pointName = "TRAVES"
  local pointCol = COL_GREEN
  if absTwaDeg < 38 then
    pointName = "PROA AL VIENTO (NO-GO!)"
    pointCol = COL_RED
  elseif absTwaDeg < 68 then
    pointName = "CENIDA (CLOSE HAULED)"
    pointCol = COL_CYAN
  elseif absTwaDeg < 115 then
    pointName = "TRAVES (BEAM REACH)"
    pointCol = COL_GREEN
  elseif absTwaDeg < 150 then
    pointName = "LARGO (BROAD REACH)"
    pointCol = COL_YELLOW
  else
    pointName = "POPA CERRADA (RUNNING)"
    pointCol = COL_ORANGE
  end
  drawText(172, 246, pointName, SMLSIZE + pointCol)

  -- 4. Camara y Escora (Abajo Derecha)
  local camNames = { "CAM:AUTO", "CAM:MAN", "CAM:3D" }
  drawText(355, 232, string.format("HEEL:%+d deg", floor(S.heel * RAD2DEG)), SMLSIZE + COL_WHITE)
  drawText(355, 248, string.format("%s [%s]", camNames[S.camMode], S.halfRes and "240p" or "480p"), SMLSIZE + COL_CYAN)

  if S.markFlash > 0 then
    drawFilledRectangle(165, 36, 150, 22, COL_HUD_BG)
    drawRectangle(165, 36, 150, 22, COL_YELLOW)
    drawText(176, 40, "BOYA SUPERADA!", SMLSIZE + COL_YELLOW)
  end
end

-- ============================================================================
-- MENU DE CONFIGURACION (SYS / TELE / TOUCH)
-- ============================================================================
local function drawMenu()
  drawFilledRectangle(60, 28, 360, 216, COL_HUD_BG)
  drawRectangle(60, 28, 360, 216, COL_CYAN)
  drawText(76, 36, "RC SAILBOAT - CONFIGURACION (TX16S)", SMLSIZE + COL_YELLOW)

  local camNames = { "1. LOS AUTO-ZOOM (Muelle)", "2. LOS ZOOM MANUAL (Stick ELE)", "3. FOLLOW 3D (Persecucion)" }
  local items = {
    string.format("Velero: %s", SAIL_PRESETS[S.presetIdx].name),
    string.format("Circuito: %s", SAIL_COURSES[S.courseIdx].name),
    string.format("Viento Base: %.1f kt (%.1f m/s)", S.windBase * 1.94384, S.windBase),
    string.format("Camara: %s", camNames[S.camMode]),
    string.format("Vectores Aero/Hidro: %s", S.showVectors and "SI" or "NO"),
    string.format("Resolucion Render: %s", S.halfRes and "240x136 (2x Rapido)" or "480x272 (Nativo)"),
    "Reiniciar Regata Ahora"
  }

  for i = 1, #items do
    local y = 62 + (i - 1) * 22
    if i == S.menuCursor then
      drawFilledRectangle(68, y - 1, 344, 18, COL_MENU_SEL)
      drawText(74, y, "> " .. items[i], SMLSIZE + COL_YELLOW)
    else
      drawText(74, y, "  " .. items[i], SMLSIZE + COL_WHITE)
    end
  end
  drawText(74, 222, "ROTARY: Mover | ENT: Cambiar | RTN/SYS: Volver", SMLSIZE + COL_CYAN)
end

local function handleEvents(event, touchState)
  if S.menuOpen then
    if event == EVT_VIRTUAL_NEXT or event == EVT_ROT_RIGHT then
      S.menuCursor = (S.menuCursor % 7) + 1
    elseif event == EVT_VIRTUAL_PREV or event == EVT_ROT_LEFT then
      S.menuCursor = ((S.menuCursor - 2) % 7) + 1
    elseif event == EVT_VIRTUAL_ENTER then
      if S.menuCursor == 1 then
        S.presetIdx = (S.presetIdx % #SAIL_PRESETS) + 1
      elseif S.menuCursor == 2 then
        S.courseIdx = (S.courseIdx % #SAIL_COURSES) + 1
        resetSailboat()
      elseif S.menuCursor == 3 then
        S.windBase = S.windBase + 1.0
        if S.windBase > 8.5 then S.windBase = 2.5 end
      elseif S.menuCursor == 4 then
        S.camMode = (S.camMode % 3) + 1
      elseif S.menuCursor == 5 then
        S.showVectors = not S.showVectors
      elseif S.menuCursor == 6 then
        S.halfRes = not S.halfRes
      elseif S.menuCursor == 7 then
        resetSailboat()
        S.menuOpen = false
      end
    elseif event == EVT_VIRTUAL_EXIT or event == EVT_SYS_FIRST or event == EVT_TELEM_FIRST then
      S.menuOpen = false
    end
    return
  end

  -- En navegacion normal:
  if event == EVT_SYS_FIRST or event == EVT_TELEM_FIRST or event == EVT_MODEL_FIRST then
    S.menuOpen = true
  elseif event == EVT_VIRTUAL_ENTER then
    -- Ciclar modo de camara rapidamente
    S.camMode = (S.camMode % 3) + 1
  elseif event == EVT_VIRTUAL_NEXT or event == EVT_ROT_RIGHT then
    -- Cambiar circuito o zoom manual con la rueda
    if S.camMode == 2 then
      S.manualFov = clamp(S.manualFov - 2.5, 6.0, 55.0)
    else
      S.courseIdx = (S.courseIdx % #SAIL_COURSES) + 1
      resetSailboat()
    end
  elseif event == EVT_VIRTUAL_PREV or event == EVT_ROT_LEFT then
    if S.camMode == 2 then
      S.manualFov = clamp(S.manualFov + 2.5, 6.0, 55.0)
    else
      S.presetIdx = (S.presetIdx % #SAIL_PRESETS) + 1
    end
  elseif touchState and touchState.x then
    if touchState.y < 36 then
      S.menuOpen = not S.menuOpen
    elseif touchState.x > 340 and touchState.y > 220 then
      S.camMode = (S.camMode % 3) + 1
    end
  end
end

-- ============================================================================
-- ENTRYPOINTS EDGETX TOOL SCRIPT
-- ============================================================================
local function init()
  S.lastTick = getTime()
  resetSailboat()
end

local function drawBoatById(id)
  if id == 0 then
    drawBoat3D(
      S.x, S.z, S.heading, S.heel, S.pitch,
      S.boomAngle, S.jibAngle, S.rudderAngle,
      S.mainLeeR, S.mainLeeF, S.jibLeeR, S.jibLeeF,
      COL_HULL_PORT, COL_HULL_STBD, true
    )
  else
    local ai = AI_BOATS[id]
    drawBoat3D(
      ai.x, ai.z, ai.heading, ai.heel, 0.0,
      ai.boom, ai.boom * 1.05, 0.0,
      0.0, 0.0, 0.0, 0.0,
      ai.col, COL_KEEL, false
    )
  end
end

local function run(event, touchState)
  local now = getTime() -- multiplos de 10ms
  local dt = (now - S.lastTick) * 0.01
  S.lastTick = now
  if dt <= 0.0 then dt = 0.033 end
  if dt > 0.06 then dt = 0.06 end
  S.fps = floor(0.85 * S.fps + 0.15 * (1.0 / max(0.01, dt)))

  handleEvents(event, touchState)

  if not S.menuOpen then
    stepPhysics(dt)
  end

  setupCamera(dt)
  drawEnvironment()

  -- Ordenar 3 barcos por distancia a la camara sin crear tablas ni closures (cero basura GC)
  local i1, i2, i3 = 0, 1, 2
  local d1 = (S.x - camX) * (S.x - camX) + (S.z - camZ) * (S.z - camZ)
  local d2 = (AI_BOATS[1].x - camX) * (AI_BOATS[1].x - camX) + (AI_BOATS[1].z - camZ) * (AI_BOATS[1].z - camZ)
  local d3 = (AI_BOATS[2].x - camX) * (AI_BOATS[2].x - camX) + (AI_BOATS[2].z - camZ) * (AI_BOATS[2].z - camZ)
  if d1 < d2 then d1, d2 = d2, d1; i1, i2 = i2, i1 end
  if d2 < d3 then d2, d3 = d3, d2; i2, i3 = i3, i2 end
  if d1 < d2 then d1, d2 = d2, d1; i1, i2 = i2, i1 end

  drawBoatById(i1)
  drawBoatById(i2)
  drawBoatById(i3)

  drawHud()
  if S.menuOpen then
    drawMenu()
  end

  return 0
end

return { init = init, run = run }
