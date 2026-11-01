-- TNS|FPV Fixed-Wing|TNE
-- =============================================================================
-- FPV FIXED-WING 6-DOF SIMULATOR PARA RADIOMASTER TX16S (EdgeTX ColorLCD)
-- Port 1:1 de la fisica por momentos, PID INAV, bateria LiPo 3S, choques,
-- aterrizaje de panza y 6 aviones (DART 250G, Z-84, AR WING, BIXLER, TALON, EDGE 540).
-- Optimizado para 30 FPS usando el acelerador hardware DMA2D (drawFilledTriangle)
-- con soporte de Media Resolucion (240x136) y Pantalla Completa (480x272).
-- =============================================================================

local sin, cos, sqrt, atan2 = math.sin, math.cos, math.sqrt, (math.atan2 or math.atan)
local floor, min, max, abs  = math.floor, math.min, math.max, math.abs
local sformat               = string.format

local drawLine, drawTri, drawRect, drawText
local getTime, getValue, playTone, getFieldInfo

local DEG2RAD = 0.01745329252
local RAD2DEG = 57.2957795131

-- Resolucion activa: halfRes = true renderiza en un viewport 240x136 (o escalado 2x a 480x272)
-- Con scale2x = true ocupa toda la pantalla 480x272 con coordenadas logicas 240x136 (DMA2D ultra rapido)
local halfRes = true
local VW, VH  = 240, 136
local SCALE   = 2
local OX, OY  = 0, 0
local FOCAL   = 128.0

local function setResolutionMode(useHalf)
  halfRes = useHalf
  if halfRes then
    VW, VH = 240, 136
    SCALE  = 2
    OX, OY = 0, 0
    FOCAL  = 125.0
  else
    VW, VH = 480, 272
    SCALE  = 1
    OX, OY = 0, 0
    FOCAL  = 250.0
  end
end

-- Paleta RGB565 precalculada en init() para cero llamadas a lcd.RGB() por frame
local COL_SKY_TOP, COL_SKY_HOR, COL_GND_DARK, COL_GND_LITE, COL_GND_GRID
local COL_RWY_ASPH, COL_RWY_LINE, COL_WHITE, COL_BLACK, COL_GOLD, COL_GREEN
local COL_RED, COL_CYAN, COL_ORANGE, COL_MAG, COL_GREY, COL_HUD_BG
local COL_WALL_L, COL_WALL_D, COL_ROOF, COL_TREE_G, COL_TREE_D, COL_TRUNK

-- 6 Presets de Ala Fija y Aviones con Cola (1:1 con index.html)
local WING_PRESETS = {
  { name = "DART 250G",   hasRudder = 0, fwdSweep = 1, pitchRate = 215, rollRate = 300, yawRate =   0, rudderRoll = 0.00, expo = 0.38, wingLoad = 1.00, maxThrustAcc = 18.0, cgOffset =  0.00, tipStall = 0.72 },
  { name = "Z-84 WING",   hasRudder = 0, fwdSweep = 0, pitchRate = 210, rollRate = 290, yawRate =   0, rudderRoll = 0.00, expo = 0.38, wingLoad = 1.00, maxThrustAcc = 18.0, cgOffset =  0.00, tipStall = 1.00 },
  { name = "AR WING LR",  hasRudder = 0, fwdSweep = 0, pitchRate = 175, rollRate = 240, yawRate =   0, rudderRoll = 0.00, expo = 0.42, wingLoad = 1.25, maxThrustAcc = 20.5, cgOffset =  0.01, tipStall = 1.20 },
  { name = "BIXLER TAIL", hasRudder = 1, fwdSweep = 0, pitchRate = 145, rollRate = 195, yawRate =  95, rudderRoll = 0.48, expo = 0.48, wingLoad = 0.85, maxThrustAcc = 15.5, cgOffset =  0.02, tipStall = 0.50 },
  { name = "TALON VTAIL", hasRudder = 1, fwdSweep = 0, pitchRate = 185, rollRate = 250, yawRate = 115, rudderRoll = 0.32, expo = 0.40, wingLoad = 1.15, maxThrustAcc = 19.5, cgOffset =  0.01, tipStall = 0.90 },
  { name = "EDGE 540 3D", hasRudder = 1, fwdSweep = 0, pitchRate = 320, rollRate = 430, yawRate = 190, rudderRoll = 0.18, expo = 0.45, wingLoad = 0.82, maxThrustAcc = 25.0, cgOffset = -0.02, tipStall = 1.30 }
}

local WIND_PRESETS = {
  { name = "CALMA 0KT",     baseKts =  0.0, gustPct = 0.00, turbPct = 0.00, shiftDeg =  0, thermalPct = 0.00 },
  { name = "TRANQUILO 6KT", baseKts =  6.0, gustPct = 0.10, turbPct = 0.10, shiftDeg =  4, thermalPct = 0.20 },
  { name = "BRISA 12KT",    baseKts = 12.0, gustPct = 0.25, turbPct = 0.20, shiftDeg = 10, thermalPct = 0.40 },
  { name = "TURBULENTO 16K",baseKts = 16.0, gustPct = 0.65, turbPct = 0.75, shiftDeg = 22, thermalPct = 0.80 }
}

local presetIdx  = 1
local windIdx    = 2
local circuitIdx = 1

-- Bateria LiPo 3S 2200mAh compartida (1:1 con CRAFT_BATT de index.html)
local LIPO_SOC = { 0.00, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00 }
local LIPO_OCV = { 3.00, 3.30, 3.45, 3.60, 3.72, 3.82, 3.98, 4.20 }
local batt = { cells = 3, capMah = 2200, usedMah = 0, rIntCell = 0.010, amps = 0, vRest = 12.6, vLoad = 12.6, lowWarnT = -99 }

local function lipoCellOcv(soc)
  if soc <= 0 then return LIPO_OCV[1] end
  for i = 2, 8 do
    if soc <= LIPO_SOC[i] then
      local f = (soc - LIPO_SOC[i - 1]) / (LIPO_SOC[i] - LIPO_SOC[i - 1])
      return LIPO_OCV[i - 1] + (LIPO_OCV[i] - LIPO_OCV[i - 1]) * f
    end
  end
  return LIPO_OCV[8]
end

local function battReset(cells, capMah, rInt)
  batt.cells = cells; batt.capMah = capMah; batt.rIntCell = rInt
  batt.usedMah = 0; batt.amps = 0; batt.vRest = cells * 4.2; batt.vLoad = batt.vRest; batt.lowWarnT = -99
end

local function battStep(amps, dt, nowT)
  batt.amps = amps
  batt.usedMah = batt.usedMah + amps * dt / 3.6
  local soc = max(0.0, 1.0 - batt.usedMah / batt.capMah)
  batt.vRest = lipoCellOcv(soc) * batt.cells
  batt.vLoad = max(batt.cells * 2.8, batt.vRest - amps * batt.rIntCell * batt.cells)
  if (batt.vRest / batt.cells) < 3.50 and (nowT - batt.lowWarnT) > 6.0 then
    batt.lowWarnT = nowT
    if playTone then playTone(480, 120, 0) end
  end
  return batt.vLoad / (batt.cells * 4.2)
end

-- Coordenadas Chorobobo HD (2178x1452m, Spawn UV = 0.585, 0.365)
local WORLD_W_M = 2178.0
local WORLD_H_M = 1452.0
local SPAWN_U   = 0.585
local SPAWN_V   = 0.365

local function uvToWorldX(u) return (u - SPAWN_U) * WORLD_W_M end
local function uvToWorldZ(v) return (SPAWN_V - v) * WORLD_H_M end

-- 6 Edificios de Chorobobo en coordenadas de mundo precalculadas
local BUILDINGS = {
  { u = 0.6214, v = 0.3807, w = 11, d =  4, h = 3.3, yaw = -0.17, open = true  }, -- Toldo Pits A.I.A
  { u = 0.5485, v = 0.3518, w =  4, d =  4, h = 9.7, yaw =  1.47, open = false }, -- Torre de Agua
  { u = 0.5450, v = 0.4815, w = 10, d = 38, h = 6.7, yaw =  1.57, open = false }, -- Galpon 1
  { u = 0.5468, v = 0.4679, w = 10, d = 37, h = 6.7, yaw =  1.61, open = false }, -- Galpon 2
  { u = 0.5480, v = 0.4500, w = 17, d = 11, h = 6.7, yaw =  1.57, open = false }, -- Galpon 3
  { u = 0.5431, v = 0.4542, w = 13, d =  9, h = 6.7, yaw =  1.54, open = false }  -- Galpon 4
}

-- 24 Arboles representativos alrededor de la pista A.I.A Chorobobo
local TREES = {
  { x =  45, z =  18, h = 7.5, w = 6.0 }, { x =  12, z =  28, h = 8.0, w = 6.5 },
  { x = -28, z =  36, h = 7.2, w = 5.8 }, { x = -65, z =  42, h = 8.5, w = 7.0 },
  { x = -110,z =  25, h = 7.8, w = 6.2 }, { x = -145,z = -15, h = 8.2, w = 6.5 },
  { x = -95, z = -65, h = 7.0, w = 5.6 }, { x = -35, z = -55, h = 7.4, w = 6.0 },
  { x =  25, z = -48, h = 6.8, w = 5.4 }, { x =  88, z = -35, h = 8.0, w = 6.4 },
  { x = 135, z =  10, h = 7.6, w = 6.0 }, { x = 165, z = -45, h = 8.4, w = 6.8 },
  { x =  72, z = -115,h = 7.5, w = 6.0 }, { x =   5, z = -140,h = 8.2, w = 6.6 },
  { x = -62, z = -155,h = 7.8, w = 6.2 }, { x = -130,z = -135,h = 8.0, w = 6.4 },
  { x = -185,z = -80, h = 8.5, w = 6.8 }, { x = -220,z =  10, h = 7.6, w = 6.0 },
  { x = -165,z =  75, h = 8.2, w = 6.5 }, { x = -85, z =  92, h = 7.4, w = 5.8 },
  { x =  15, z =  85, h = 7.8, w = 6.2 }, { x =  95, z =  68, h = 8.0, w = 6.4 },
  { x = 175, z = -95, h = 7.6, w = 6.0 }, { x = 120, z = -165,h = 8.2, w = 6.6 }
}

-- 3 Circuitos de Chorobobo + Vuelo Libre
local CIRCUIT_DEFS = {
  {
    name = "C1 ROJO PISTA", short = "C1",
    uv = {
      {0.5778, 0.3712, 1.00, "arch"},      {0.5261, 0.3502, 0.90, "flag_L"},    {0.4715, 0.3281, 0.85, "flag_R"},
      {0.4246, 0.3087, 1.05, "split_lr"},  {0.3886, 0.3002, 1.35, "dive_cube"}, {0.4114, 0.3528, 1.15, "diamond"},
      {0.4414, 0.4075, 0.85, "arch"},      {0.4668, 0.4595, 0.95, "flag_L"},    {0.5028, 0.4884, 1.25, "over_under"},
      {0.5782, 0.4928, 1.15, "split_lr"},  {0.6447, 0.4918, 0.95, "flag_R"},    {0.6760, 0.4765, 0.85, "flag_L"},
      {0.6727, 0.4039, 1.15, "diamond"},   {0.6147, 0.3851, 1.00, "over_under"}
    }
  },
  {
    name = "C2 AZUL OCHO", short = "C2",
    uv = {
      {0.5538, 0.3652, 0.85, "arch"},      {0.5183, 0.3487, 0.80, "split_lr"},  {0.4489, 0.3204, 1.10, "flag_L"},
      {0.4041, 0.3348, 1.45, "dive_cube"}, {0.4380, 0.3995, 1.20, "diamond"},   {0.4827, 0.4950, 0.85, "flag_R"},
      {0.5283, 0.4914, 0.78, "over_under"},{0.5455, 0.4276, 0.90, "flag_L"},    {0.5756, 0.3437, 1.30, "diamond"},
      {0.6437, 0.3705, 1.55, "dive_cube"}, {0.6782, 0.4779, 1.20, "split_lr"},  {0.5880, 0.4880, 0.95, "flag_R"},
      {0.5380, 0.4550, 1.35, "diamond"},   {0.5075, 0.4248, 1.75, "dive_cube"}, {0.5159, 0.3875, 1.10, "arch"},
      {0.5485, 0.3998, 1.28, "diamond"}
    }
  },
  {
    name = "C3 SLALOM PARK", short = "C3",
    uv = {
      {0.552, 0.376, 0.80, "arch"},       {0.525, 0.365, 0.78, "flag_L"},     {0.495, 0.388, 1.20, "flag_R"},
      {0.468, 0.368, 0.78, "flag_L"},     {0.442, 0.392, 1.50, "dive_cube"},  {0.455, 0.428, 0.85, "split_lr"},
      {0.485, 0.412, 0.78, "flag_R"},     {0.512, 0.445, 1.65, "dive_cube"},  {0.545, 0.432, 0.85, "over_under"},
      {0.575, 0.458, 1.30, "diamond"},    {0.605, 0.435, 0.85, "flag_L"},     {0.618, 0.411, 0.78, "flag_R"},
      {0.612, 0.378, 1.45, "dive_cube"},  {0.588, 0.352, 0.95, "split_lr"},   {0.568, 0.365, 0.82, "over_under"}
    }
  },
  {
    name = "VUELO LIBRE", short = "FREE",
    uv = {}
  }
}

local CIRCUITS = {}
local activeGates = {}

local function initWorldGeometry()
  for i = 1, #BUILDINGS do
    local b = BUILDINGS[i]
    b.x = uvToWorldX(b.u)
    b.z = uvToWorldZ(b.v)
    b.cosY = cos(b.yaw)
    b.sinY = sin(b.yaw)
  end

  for c = 1, #CIRCUIT_DEFS do
    local def = CIRCUIT_DEFS[c]
    local uvL = def.uv
    local n = #uvL
    local gates = {}
    if n > 0 then
      local pts = {}
      for i = 1, n do
        local item = uvL[i]
        pts[i] = {
          x = uvToWorldX(item[1]),
          z = uvToWorldZ(item[2]),
          y = max(4.5, 9.0 * (item[3] or 1.0)),
          gType = item[4] or "arch"
        }
      end
      for i = 1, n do
        local cur = pts[i]
        local nxt = pts[(i % n) + 1]
        local prv = pts[((i - 2 + n) % n) + 1]
        local tx = nxt.x - prv.x
        local tz = nxt.z - prv.z
        local tLen = sqrt(tx * tx + tz * tz)
        if tLen < 1e-4 then tx, tz = -0.95, 0.31 else tx, tz = tx / tLen, tz / tLen end
        gates[i] = {
          x = cur.x, y = cur.y, z = cur.z,
          w = 12.0, h = 9.0,
          gType = cur.gType,
          tx = tx, tz = tz,
          nx = tz, nz = -tx
        }
      end
    end
    CIRCUITS[c] = { name = def.name, short = def.short, gates = gates }
  end
  activeGates = CIRCUITS[circuitIdx].gates
end

-- Entorno atmosferico (1:1 con aeroEnv de index.html)
local aeroEnv = {
  time = 0, windX = 2.5, windY = 0, windZ = 1.2,
  turbPitch = 0, turbRoll = 0, turbYaw = 0,
  vibPitch = 0, vibRoll = 0, vibYaw = 0,
  joltX = 0, joltY = 0, joltZ = 0,
  airSpd = 15, stallWarn = false,
  rssiPct = 99, rssiDbm = -50, failsafeTimer = 0
}

local function updateAeroEnvironment(px, py, pz, dt)
  aeroEnv.time = aeroEnv.time + dt
  local t = aeroEnv.time
  local wCfg = WIND_PRESETS[windIdx]

  local normX = abs(px) / (WORLD_W_M * 0.47)
  local normZ = abs(pz) / (WORLD_H_M * 0.47)
  local edgeRatio = max(normX, normZ)
  if edgeRatio <= 0.20 then
    aeroEnv.rssiPct = floor(99 - edgeRatio * 20)
  elseif edgeRatio <= 0.72 then
    aeroEnv.rssiPct = floor(95 - ((edgeRatio - 0.20) / 0.52) * 55)
  else
    aeroEnv.rssiPct = max(0, floor(40 * (1.0 - min(1.0, (edgeRatio - 0.72) / 0.26))))
  end
  aeroEnv.rssiDbm = floor(-48 - (99 - aeroEnv.rssiPct) * 0.72)

  if aeroEnv.rssiPct <= 3 or edgeRatio >= 0.98 then
    aeroEnv.failsafeTimer = 2.0
  elseif aeroEnv.failsafeTimer > 0 then
    aeroEnv.failsafeTimer = max(0, aeroEnv.failsafeTimer - dt)
  end

  local baseMs = wCfg.baseKts * 0.51444
  local shiftRad = (wCfg.shiftDeg * DEG2RAD) * (0.52 * sin(t * 0.42 + px * 0.008) + 0.31 * cos(t * 1.05 - pz * 0.010))
  local windDir = 1.12 + shiftRad
  local gustWave = 1.0 + wCfg.gustPct * (0.48 * sin(t * 1.35 + px * 0.018) + 0.26 * cos(t * 0.78))
  local effSpd = max(0.0, baseMs * gustWave)

  aeroEnv.windX = cos(windDir) * effSpd
  aeroEnv.windZ = sin(windDir) * effSpd
  aeroEnv.windY = sin(px * 0.028 + pz * 0.030 + t * 1.15) * wCfg.thermalPct * 0.8

  local spdFactor = 0.70 + 0.30 * min(2.2, (aeroEnv.airSpd or 10.0) / 14.0)
  local turbAmp = 1.15 * wCfg.turbPct * spdFactor
  aeroEnv.turbRoll  = (0.48 * sin(t * 2.35 + px * 0.035) + 0.34 * cos(t * 5.2 + pz * 0.055)) * turbAmp
  aeroEnv.turbPitch = (0.40 * cos(t * 2.05 + pz * 0.032) + 0.28 * sin(t * 4.8 - px * 0.050)) * turbAmp
  aeroEnv.turbYaw   = (0.36 * sin(t * 1.85 - px * 0.036) + 0.25 * cos(t * 4.4 + pz * 0.050)) * turbAmp

  local vibAmp = wCfg.turbPct * 0.85 * spdFactor
  aeroEnv.vibRoll  = sin(t * 19.5) * vibAmp
  aeroEnv.vibPitch = cos(t * 17.2) * vibAmp
  aeroEnv.vibYaw   = sin(t * 15.8) * vibAmp * 0.6

  aeroEnv.joltX = sin(t * 7.8) * 1.4 * wCfg.turbPct
  aeroEnv.joltY = cos(t * 8.6) * 1.8 * wCfg.turbPct
  aeroEnv.joltZ = cos(t * 7.2) * 1.4 * wCfg.turbPct
end

-- Integrador exacto en SO(3) (Rodrigues + Gram-Schmidt)
local function stepRotationSO3(craft, dt)
  local p = craft.pitchRate * dt
  local r = craft.rollRate * dt
  local y = craft.yawRate * dt
  local thSq = p * p + r * r + y * y
  if thSq > 1e-12 then
    local th = sqrt(thSq)
    local c1 = sin(th) / th
    local c2 = (1.0 - cos(th)) / thSq
    local rx, ry, rz = craft.rx, craft.ry, craft.rz
    local ux, uy, uz = craft.ux, craft.uy, craft.uz
    local fx, fy, fz = craft.fx, craft.fy, craft.fz

    local v1Rx, v1Ry, v1Rz = -r * ux - y * fx, -r * uy - y * fy, -r * uz - y * fz
    local v1Ux, v1Uy, v1Uz =  r * rx + p * fx,  r * ry + p * fy,  r * rz + p * fz
    local v1Fx, v1Fy, v1Fz =  y * rx - p * ux,  y * ry - p * uy,  y * rz - p * uz

    local v2Rx, v2Ry, v2Rz = -r * v1Ux - y * v1Fx, -r * v1Uy - y * v1Fy, -r * v1Uz - y * v1Fz
    local v2Ux, v2Uy, v2Uz =  r * v1Rx + p * v1Fx,  r * v1Ry + p * v1Fy,  r * v1Rz + p * v1Fz
    local v2Fx, v2Fy, v2Fz =  y * v1Rx - p * v1Ux,  y * v1Ry - p * v1Uy,  y * v1Rz - p * v1Uz

    fx = fx + c1 * v1Fx + c2 * v2Fx
    fy = fy + c1 * v1Fy + c2 * v2Fy
    fz = fz + c1 * v1Fz + c2 * v2Fz
    ux = ux + c1 * v1Ux + c2 * v2Ux
    uy = uy + c1 * v1Uy + c2 * v2Uy
    uz = uz + c1 * v1Uz + c2 * v2Uz

    local fL = sqrt(fx * fx + fy * fy + fz * fz)
    if fL > 1e-6 then fx, fy, fz = fx / fL, fy / fL, fz / fL end
    local dot = ux * fx + uy * fy + uz * fz
    ux, uy, uz = ux - dot * fx, uy - dot * fy, uz - dot * fz
    local uL = sqrt(ux * ux + uy * uy + uz * uz)
    if uL > 1e-6 then ux, uy, uz = ux / uL, uy / uL, uz / uL end

    craft.fx, craft.fy, craft.fz = fx, fy, fz
    craft.ux, craft.uy, craft.uz = ux, uy, uz
    craft.rx = uy * fz - uz * fy
    craft.ry = uz * fx - ux * fz
    craft.rz = ux * fy - uy * fx
  end
end

local function applySuperExpo(rc, rate, expo)
  local absRc = abs(rc)
  return rc * (absRc * absRc * expo + (1.0 - expo)) * rate
end

local function evalWingLiftCoeff(alphaEff, stallAngle, pullUpDeflect)
  local absA = abs(alphaEff)
  local flatPlate = 1.95 * sin(alphaEff) * cos(alphaEff)
  if absA <= stallAngle then
    return 4.50 * alphaEff + 0.24 * pullUpDeflect
  end
  local blend = min(1.0, (absA - stallAngle) / 0.22)
  local signA = alphaEff >= 0 and 1.0 or -1.0
  local preStall = signA * (4.50 * stallAngle) + 0.14 * pullUpDeflect
  return preStall * (1.0 - blend) + flatPlate * blend
end

-- Estado del Ala Fija (1:1 con fpvWingSim de index.html)
local wing = {
  px = 0, py = 12, pz = -20,
  vx = 0, vy = 0, vz = 15, spd = 15,
  pitchRate = 0, rollRate = 0, yawRate = 0,
  rx = 1, ry = 0, rz = 0,
  ux = 0, uy = 1, uz = 0,
  fx = 0, fy = 0, fz = 1,
  mode = "ACRO",
  launchTimer = 1.05,
  chaseCam = false,
  activeGate = 1,
  thrCmd = 0.60, motorRpm = 0.58,
  elevL = 0, elevR = 0, lastAlpha = 0.02,
  iQ = 0, onGround = false, crashT = 0,
  flightTime = 0, totalDistKm = 0
}

local followCam = {
  init = false,
  fx = 0, fy = 0, fz = 1,
  ux = 0, uy = 1, uz = 0,
  rx = 1, ry = 0, rz = 0
}

local toastMsg   = ""
local toastTimer = 0
local function showToast(msg)
  toastMsg   = msg
  toastTimer = 1.5
end

local function resetWing()
  -- Salida oficial Chorobobo (u=0.5967, v=0.3791, hdg=17 deg, alt=12m)
  wing.px = uvToWorldX(0.5967)
  wing.pz = uvToWorldZ(0.3791)
  wing.py = 12.25
  local rad = 17.0 * DEG2RAD
  local dirTx = cos(rad)
  local dirTz = -sin(rad)
  local dirNx = dirTz
  local dirNz = -dirTx

  local throwPitch = 4.0 * DEG2RAD
  local cosP, sinP = cos(throwPitch), sin(throwPitch)
  wing.fx =  dirTx * cosP
  wing.fy =  sinP
  wing.fz =  dirTz * cosP
  wing.ux = -dirTx * sinP
  wing.uy =  cosP
  wing.uz = -dirTz * sinP
  wing.rx =  dirNx
  wing.ry =  0
  wing.rz =  dirNz

  local throwSpd = 15.5
  wing.vx = wing.fx * throwSpd
  wing.vy = wing.fy * throwSpd + 0.55
  wing.vz = wing.fz * throwSpd
  wing.spd = sqrt(wing.vx * wing.vx + wing.vy * wing.vy + wing.vz * wing.vz)

  wing.pitchRate = 0; wing.rollRate = 0.35; wing.yawRate = 0
  wing.launchTimer = 1.05
  wing.thrCmd = 0.60; wing.motorRpm = 0.58
  wing.elevL = 0; wing.elevR = 0; wing.lastAlpha = 0.02
  wing.iQ = 0; wing.onGround = false; wing.crashT = 0
  wing.activeGate = 1
  wing.flightTime = 0; wing.totalDistKm = 0
  followCam.init = false
  aeroEnv.stallWarn = false
  battReset(3, 2200, 0.010)
end

-- Colision contra edificios solidos y arboles
local function checkWorldCollision(prevX, prevY, prevZ)
  local px, py, pz = wing.px, wing.py, wing.pz
  for i = 1, #BUILDINGS do
    local b = BUILDINGS[i]
    if not b.open and py <= b.h + 0.4 then
      local dx, dz = px - b.x, pz - b.z
      local lx =  dx * b.cosY + dz * b.sinY
      local lz = -dx * b.sinY + dz * b.cosY
      if abs(lx) <= (b.w * 0.5 + 0.35) and abs(lz) <= (b.d * 0.5 + 0.35) then
        wing.px, wing.py, wing.pz = prevX, prevY, prevZ
        wing.vx, wing.vy, wing.vz = -wing.vx * 0.35, max(0, wing.vy), -wing.vz * 0.35
        return true
      end
    end
  end
  for i = 1, #TREES do
    local tr = TREES[i]
    if py <= tr.h + 0.25 then
      local dx, dz = px - tr.x, pz - tr.z
      local r = tr.w * 0.30 + 0.35
      if (dx * dx + dz * dz) <= (r * r) then
        wing.px, wing.py, wing.pz = prevX, prevY, prevZ
        wing.vx, wing.vy, wing.vz = -wing.vx * 0.35, max(0, wing.vy), -wing.vz * 0.35
        return true
      end
    end
  end
  return false
end

-- Paso por Gate 3D
local function checkGatePass(g, prevX, prevY, prevZ, px, py, pz)
  if not g then return false end
  local d0 = (prevX - g.x) * g.tx + (prevZ - g.z) * g.tz
  local d1 = (px - g.x) * g.tx + (pz - g.z) * g.tz
  if not (((d0 <= 0 and d1 >= 0) or (d0 >= 0 and d1 <= 0)) and abs(d1 - d0) < 18.0) then
    return false
  end
  local denom = d1 - d0
  local t = abs(denom) > 1e-5 and max(0.0, min(1.0, -d0 / denom)) or 1.0
  local cx = prevX + (px - prevX) * t
  local cy = prevY + (py - prevY) * t
  local cz = prevZ + (pz - prevZ) * t
  local lat = abs((cx - g.x) * g.nx + (cz - g.z) * g.nz)
  local hw, hh = g.w * 0.5 + 0.55, g.h * 0.5 + 0.55
  return lat <= hw and cy >= max(0.5, g.y - hh) and cy <= (g.y + hh + 1.5)
end

-- Lectura de Gimbals de la TX16S
local chThr, chAil, chEle, chRud = 3, 1, 2, 4
local function readSticks()
  local rawAil = getValue(chAil) or 0
  local rawEle = getValue(chEle) or 0
  local rawThr = getValue(chThr) or 0
  local rawRud = getValue(chRud) or 0
  local rx = max(-1.0, min(1.0, rawAil / 1024.0))
  local ry = max(-1.0, min(1.0, rawEle / 1024.0))
  local lx = max(-1.0, min(1.0, rawRud / 1024.0))
  local thr = max(0.0, min(1.0, (rawThr + 1024.0) / 2048.0))
  return lx, thr, rx, ry
end

local function stepWingPhysics(dt, lx, thrStick, rx, ry)
  wing.flightTime = wing.flightTime + dt
  local cfg = WING_PRESETS[presetIdx]
  local groundH = 0.25

  updateAeroEnvironment(wing.px, wing.py, wing.pz, dt)

  if wing.launchTimer > 0 then
    wing.launchTimer = max(0, wing.launchTimer - dt)
    if (rx * rx + ry * ry) > 0.05 then wing.launchTimer = 0 end
  end

  local thr = thrStick
  if wing.launchTimer > 0 or aeroEnv.failsafeTimer > 0 then
    thr = max(thr, 0.58)
  end
  if wing.crashT > 0 then thr = 0.0 end
  wing.thrCmd = thr
  wing.motorRpm = wing.motorRpm + (thr - wing.motorRpm) * min(1.0, 14.0 * dt)

  local vAirX = wing.vx - aeroEnv.windX
  local vAirY = wing.vy - aeroEnv.windY
  local vAirZ = wing.vz - aeroEnv.windZ
  local Va = max(0.5, sqrt(vAirX * vAirX + vAirY * vAirY + vAirZ * vAirZ))
  aeroEnv.airSpd = Va

  local uBody = vAirX * wing.fx + vAirY * wing.fy + vAirZ * wing.fz
  local vBody = vAirX * wing.rx + vAirY * wing.ry + vAirZ * wing.rz
  local wBody = vAirX * wing.ux + vAirY * wing.uy + vAirZ * wing.uz

  local alpha = atan2(-wBody, uBody)
  local beta  = atan2( vBody, max(0.5, sqrt(uBody * uBody + wBody * wBody)))
  wing.lastAlpha = alpha

  local hasRudder = (cfg.hasRudder == 1 and cfg.yawRate > 0)
  local rcPitch   = applySuperExpo(-ry, 1.0, cfg.expo)
  local rcRoll    = applySuperExpo( rx, 1.0, cfg.expo)
  local rcYaw     = hasRudder and applySuperExpo(lx, 1.0, cfg.expo * 0.75) or 0.0

  local VA0 = 19.4
  local Lp  = 0.86
  local Ld  = Lp * (cfg.rollRate * DEG2RAD) / VA0
  local Lb  = 1.40 * Lp / VA0 * (hasRudder and 1.3 or 1.0)
  local Ma  = 0.17 * (1.0 + cfg.cgOffset * 6.0)
  local Md  = Ma * 0.38
  local Mq  = 0.10
  local Nr  = 0.515
  local Nb  = Nr * (hasRudder and 4.2 or 3.4) / VA0
  local Ndr = hasRudder and (Nr * (cfg.yawRate * DEG2RAD) / VA0) or 0.0
  local Ldr = hasRudder and (cfg.rudderRoll * 1.8 * Lp / VA0) or 0.0

  local Qc = Va * Va + (hasRudder and 60.0 or 25.0) * wing.motorRpm * wing.motorRpm
  local alphaTrim = (0.053 * cfg.wingLoad - 0.052) - cfg.cgOffset * 0.30

  local halfSpan   = 0.45
  local dAlphaRoll = max(-0.5, min(0.5, (wing.rollRate * halfSpan) / max(5.0, Va)))
  local alpha0     = -0.052
  local alphaEff   = alpha - alpha0
  local invWingLoad = 1.0 / max(0.55, cfg.wingLoad)
  local qDyn       = 0.110 * Va * Va * invWingLoad
  local stallAngle = 0.285 / sqrt(max(0.6, cfg.wingLoad))
  local stallSpeed = 8.33 * sqrt(max(0.6, cfg.wingLoad))
  local isStalled  = (abs(alphaEff) > stallAngle) or (Va < stallSpeed)
  aeroEnv.stallWarn = isStalled

  local noseDownAngle  = -wing.fy
  local bankRightAngle = atan2(-wing.ry, max(0.15, wing.uy))
  local uyPos          = max(0.0, wing.uy)
  local turnOmegaWorld = (9.81 * (-wing.ry) * uyPos) / max(8.3, Va)
  local coordTurnPitch = turnOmegaWorld * wing.ry * 0.82

  local dE, dA, dR = 0, 0, 0
  local wantP, wantQ, wantR = nil, 0, 0

  if aeroEnv.failsafeTimer > 0 then
    local toHomeX, toHomeZ = -wing.px, -wing.pz
    local homeBearing = atan2(toHomeX * wing.rx + toHomeZ * wing.rz, toHomeX * wing.fx + toHomeZ * wing.fz)
    local desiredBank = max(-0.72, min(0.72, homeBearing * 1.4))
    local desiredNose = (wing.py - groundH < 24.0 and -12.0 or -4.0) * DEG2RAD
    wantQ = (desiredNose - noseDownAngle) * 4.2 + coordTurnPitch * 0.6
    wantP = (desiredBank - bankRightAngle) * 4.8
  elseif wing.launchTimer > 0 then
    wantQ = (-11.0 * DEG2RAD - noseDownAngle) * 5.5
    wantP = (0.0 - bankRightAngle) * 6.5
  elseif wing.mode == "MANU" then
    dE = -rcPitch; dA = rcRoll; dR = rcYaw
  elseif wing.mode == "ACRO" then
    wantQ = rcPitch * cfg.pitchRate * DEG2RAD
    wantP = rcRoll  * cfg.rollRate  * DEG2RAD
    wantR = rcYaw   * cfg.yawRate   * DEG2RAD
  else
    local targetNose = max(-35.0 * DEG2RAD, min(35.0 * DEG2RAD, (-ry * 34.0) * DEG2RAD))
    local targetBank = max(-52.0 * DEG2RAD, min(52.0 * DEG2RAD, ( rx * 50.0) * DEG2RAD))
    wantQ = (targetNose - noseDownAngle) * 3.6 + coordTurnPitch * 0.75
    wantP = (targetBank - bankRightAngle) * 4.2
    wantR = rcYaw * cfg.yawRate * DEG2RAD * 0.9
  end

  if wantP ~= nil then
    local tpa = (VA0 * VA0) / max(25.0, Qc)
    local errQ = wantQ - wing.pitchRate
    local errP = wantP - wing.rollRate
    local errR = wantR - wing.yawRate
    wing.iQ = max(-0.35, min(0.35, wing.iQ + errQ * 0.9 * dt))
    dE = -((wantQ * 0.25 + errQ * 0.55 + wing.iQ) * tpa)
    dA = (wantP * Lp / Ld / max(6.0, Va) + errP * 0.12 * tpa)
    dR = hasRudder and (wantR * Nr / max(1e-4, Ndr) / max(6.0, Va) + errR * 0.15 * tpa) or 0.0
  else
    wing.iQ = 0
  end

  dE = max(-1.0, min(1.0, dE))
  dA = max(-1.0, min(1.0, dA))
  dR = max(-1.0, min(1.0, dR))

  wing.elevL = max(-1.0, min(1.0, dE - dA * 0.80))
  wing.elevR = max(-1.0, min(1.0, dE + dA * 0.80))
  local effE = 0.5 * (wing.elevL + wing.elevR)
  local effA = (wing.elevR - wing.elevL) / 1.6

  local CL_L = evalWingLiftCoeff(alphaEff - dAlphaRoll, stallAngle, wing.elevL)
  local CL_R = evalWingLiftCoeff(alphaEff + dAlphaRoll, stallAngle, wing.elevR)
  local CL   = 0.5 * (CL_L + CL_R)

  local tipBias = 0.012 * cfg.tipStall
  local asymCL  = (evalWingLiftCoeff(alphaEff + tipBias, stallAngle, 0) - evalWingLiftCoeff(alphaEff, stallAngle, 0)) - 4.50 * tipBias
  local Lst     = 4.50 * Lp * cfg.tipStall
  local slopeRatio = max(0.08, (evalWingLiftCoeff(alphaEff + 0.03, stallAngle, 0) - evalWingLiftCoeff(alphaEff - 0.03, stallAngle, 0)) / (0.06 * 4.50))

  local propTorque = -0.08 * wing.motorRpm * cfg.maxThrustAcc * 0.06
  local pAcc = Ld * Qc * effA - Lp * Va * wing.rollRate * slopeRatio - Lb * Va * Va * beta + Ldr * Qc * dR
             + Lst * Va * asymCL + propTorque + aeroEnv.turbRoll * 11.4
  local qAcc = -Md * Qc * effE - Mq * Va * wing.pitchRate + Ma * Va * Va * (alpha - alphaTrim) + aeroEnv.turbPitch * 9.6
  local rAcc = Ndr * Qc * dR - Nr * Va * wing.yawRate + Nb * Va * Va * beta - 0.02 * Qc * effA * Ld / Lp + aeroEnv.turbYaw * 6.6

  wing.rollRate  = wing.rollRate  + pAcc * dt
  wing.pitchRate = wing.pitchRate + qAcc * dt
  wing.yawRate   = wing.yawRate   + rAcc * dt
  if wing.onGround then
    wing.pitchRate = wing.pitchRate * 0.6
    wing.rollRate  = wing.rollRate  * 0.6
    wing.yawRate   = wing.yawRate   * 0.6
  end

  local sp, sr, sy = wing.pitchRate, wing.rollRate, wing.yawRate
  wing.pitchRate = sp + aeroEnv.vibPitch * 0.40
  wing.rollRate  = sr + aeroEnv.vibRoll  * 0.52
  wing.yawRate   = sy + aeroEnv.vibYaw   * 0.31
  stepRotationSO3(wing, dt)
  wing.pitchRate, wing.rollRate, wing.yawRate = sp, sr, sy

  local invVa = 1.0 / Va
  local airDirX, airDirY, airDirZ = vAirX * invVa, vAirY * invVa, vAirZ * invVa
  local uDotAir = wing.ux * airDirX + wing.uy * airDirY + wing.uz * airDirZ
  local liftDirX = wing.ux - uDotAir * airDirX
  local liftDirY = wing.uy - uDotAir * airDirY
  local liftDirZ = wing.uz - uDotAir * airDirZ
  local liftLen = sqrt(liftDirX * liftDirX + liftDirY * liftDirY + liftDirZ * liftDirZ)
  if liftLen > 1e-5 then liftDirX, liftDirY, liftDirZ = liftDirX / liftLen, liftDirY / liftLen, liftDirZ / liftLen end

  local elevDrag = 0.042 * (wing.elevL * wing.elevL + wing.elevR * wing.elevR)
  local CD = 0.032 + 0.145 * CL * CL + elevDrag + 0.55 * beta * beta + (isStalled and (0.28 * sin(abs(alphaEff))) or 0.0)
  local CY = -1.85 * beta

  local liftAcc = qDyn * CL
  local dragAcc = qDyn * CD
  local sideAcc = qDyn * CY

  local rpm3 = wing.motorRpm * wing.motorRpm * wing.motorRpm
  local rpmEff = wing.motorRpm * battStep(1.5 + 38.0 * rpm3, dt, aeroEnv.time)
  local propPitchMaxSpd = 21.0 + cfg.maxThrustAcc
  local propTargetSpd   = propPitchMaxSpd * rpmEff * (0.82 + 0.18 * rpmEff)
  local thrustAcc = 0
  if uBody < propTargetSpd and propTargetSpd > 0.5 then
    local slipRatio = (propTargetSpd - max(0.0, uBody)) / max(4.0, propTargetSpd)
    thrustAcc = rpmEff * cfg.maxThrustAcc * invWingLoad * min(1.0, 1.55 * slipRatio)
  else
    thrustAcc = -0.22 * (uBody - propTargetSpd) * (1.0 - 0.45 * rpmEff) * invWingLoad
  end

  local ax = wing.fx * thrustAcc + liftDirX * liftAcc - airDirX * dragAcc + wing.rx * sideAcc + aeroEnv.joltX * invWingLoad
  local ay = wing.fy * thrustAcc + liftDirY * liftAcc - airDirY * dragAcc + wing.ry * sideAcc - 9.81 + aeroEnv.joltY * invWingLoad
  local az = wing.fz * thrustAcc + liftDirZ * liftAcc - airDirZ * dragAcc + wing.rz * sideAcc + aeroEnv.joltZ * invWingLoad

  wing.vx = wing.vx + ax * dt
  wing.vy = wing.vy + ay * dt
  wing.vz = wing.vz + az * dt
  wing.spd = sqrt(wing.vx * wing.vx + wing.vy * wing.vy + wing.vz * wing.vz)
  wing.totalDistKm = wing.totalDistKm + (wing.spd * dt) * 0.001

  local prevX, prevY, prevZ = wing.px, wing.py, wing.pz
  wing.px = wing.px + wing.vx * dt
  wing.py = wing.py + wing.vy * dt
  wing.pz = wing.pz + wing.vz * dt
  wing.onGround = false
  local crashNow = false

  if wing.py < groundH + 0.35 then
    local sinkSpd = -wing.vy
    wing.py = groundH + 0.35
    wing.onGround = true
    if wing.crashT <= 0 and wing.launchTimer <= 0 and (sinkSpd > 6.0 or (wing.fy < -0.45 and wing.spd > 12.0)) then
      crashNow = true
    end
    if wing.vy < 0 then wing.vy = 0 end
    local hs = sqrt(wing.vx * wing.vx + wing.vz * wing.vz)
    if hs > 1e-3 then
      local dec = min(hs, 0.45 * 9.81 * dt)
      wing.vx = wing.vx - (wing.vx / hs) * dec
      wing.vz = wing.vz - (wing.vz / hs) * dec
    end
    wing.rollRate  = -bankRightAngle * 5.0
    wing.pitchRate = noseDownAngle > 0 and (-noseDownAngle * 5.0) or (wing.pitchRate * 0.5)
  end

  if checkWorldCollision(prevX, prevY, prevZ) then
    crashNow = true
  end

  if crashNow and wing.crashT <= 0 then
    wing.crashT = 2.0
    showToast(sformat("CRASH %d KMH - RESET EN 2S", floor(wing.spd * 3.6)))
    if playTone then playTone(160, 220, 0) end
  end
  if wing.crashT > 0 then
    wing.crashT = wing.crashT - dt
    if wing.crashT <= 0 then
      resetWing()
      return
    end
  end

  if #activeGates > 0 then
    local g = activeGates[wing.activeGate]
    if checkGatePass(g, prevX, prevY, prevZ, wing.px, wing.py, wing.pz) then
      wing.activeGate = (wing.activeGate % #activeGates) + 1
      showToast(sformat("%s G%d/%d OK!", CIRCUITS[circuitIdx].short, wing.activeGate, #activeGates))
      if playTone then playTone(920, 70, 0) end
    end
  end
end

-- =============================================================================
-- MOTOR GRAFICO 3D DMA2D PARA TX16S (30 FPS)
-- =============================================================================
local camX, camY, camZ = 0, 10, 0
local cRx, cRy, cRz = 1, 0, 0
local cUx, cUy, cUz = 0, 1, 0
local cFx, cFy, cFz = 0, 0, 1

local function proj(wx, wy, wz)
  local dx, dy, dz = wx - camX, wy - camY, wz - camZ
  local cz = dx * cFx + dy * cFy + dz * cFz
  if cz < 0.5 then return nil, nil, nil end
  local inv = FOCAL / cz
  local sx = floor(VW * 0.5 + (dx * cRx + dy * cRy + dz * cRz) * inv) * SCALE + OX
  local sy = floor(VH * 0.5 - (dx * cUx + dy * cUy + dz * cUz) * inv) * SCALE + OY
  return sx, sy, cz
end

local function sTri(x1, y1, x2, y2, x3, y3, col)
  drawTri(x1, y1, x2, y2, x3, y3, col)
end

local function sQuad(x1, y1, x2, y2, x3, y3, x4, y4, col)
  drawTri(x1, y1, x2, y2, x3, y3, col)
  drawTri(x1, y1, x3, y3, x4, y4, col)
end

local function projLine3D(x0, y0, z0, x1, y1, z1, col)
  local sx0, sy0 = proj(x0, y0, z0)
  local sx1, sy1 = proj(x1, y1, z1)
  if sx0 and sx1 then
    if (sx0 >= 0 or sx1 >= 0) and (sx0 < 480 or sx1 < 480) and (sy0 >= 0 or sy1 >= 0) and (sy0 < 272 or sy1 < 272) then
      drawLine(sx0, sy0, sx1, sy1, SOLID, col)
    end
  end
end

-- Dibuja Cielo y Tierra usando bloques DMA2D (drawFilledRectangle) + 1 sola cuña triangular
-- acotada a 0..480x272 para minimizar al maximo las scanlines en CPU del STM32F429
local function renderTiltedSkyAndGround()
  local W, H = 480, 272
  local phi = atan2(cRy, cUy)
  local cosR, sinR = cos(phi), sin(phi)
  local hy = (VH * 0.5 + (cFy / max(0.15, sqrt(cFx * cFx + cFz * cFz))) * FOCAL * 0.85) * SCALE
  local cy = max(0, min(H, floor(hy)))

  local topCol = cUy >= 0 and COL_SKY_TOP or COL_GND_DARK
  local botCol = cUy >= 0 and COL_GND_DARK or COL_SKY_TOP

  -- Pendiente del horizonte limitada para usar 2 bloques DMA2D + 1 solo triangulo en pantalla
  local slope = sinR / max(0.28, abs(cosR))
  local dy = floor(slope * 240)
  local yL = max(0, min(H, cy - dy))
  local yR = max(0, min(H, cy + dy))
  local yMin = min(yL, yR)
  local yMax = max(yL, yR)

  if yMin > 0 then
    drawRect(0, 0, W, yMin, topCol)
  end
  if yMax < H then
    drawRect(0, yMax, W, H - yMax, botCol)
  end
  if yMax > yMin then
    -- Rellenar la franja central con botCol por DMA2D y trazar solo 1 triangulo de topCol
    drawRect(0, yMin, W, yMax - yMin, botCol)
    if yL < yR then
      drawTri(0, yL, W, yL, W, yR, topCol)
    else
      drawTri(0, yL, 0, yR, W, yR, topCol)
    end
  end
  drawLine(0, yL, W - 1, yR, SOLID, COL_SKY_HOR)
end

-- Pista A.I.A Chorobobo + Grilla de referencia optica en el suelo
local function renderRunwayAndGrid()
  -- Pista de asfalto A.I.A Chorobobo (orientada a -18 deg respecto a X)
  local rCos, rSin = 0.951, -0.309
  local rCx, rCz   = -35.0, -12.0
  local rHalfL, rHalfW = 140.0, 9.5

  local function rwyPt(l, w)
    return proj(rCx + l * rCos - w * rSin, 0.28, rCz + l * rSin + w * rCos)
  end

  local x1, y1 = rwyPt(-rHalfL, -rHalfW)
  local x2, y2 = rwyPt(-rHalfL,  rHalfW)
  local x3, y3 = rwyPt( rHalfL,  rHalfW)
  local x4, y4 = rwyPt( rHalfL, -rHalfW)
  if x1 and x2 and x3 and x4 then
    sQuad(x1, y1, x2, y2, x3, y3, x4, y4, COL_RWY_ASPH)
    drawLine(x1, y1, x4, y4, SOLID, COL_WHITE)
    drawLine(x2, y2, x3, y3, SOLID, COL_WHITE)
  end

  -- Lineas centrales discontinuas de pista
  for s = -110, 110, 28 do
    local ax, ay = rwyPt(s, 0)
    local bx, by = rwyPt(s + 14, 0)
    if ax and bx then drawLine(ax, ay, bx, by, SOLID, COL_RWY_LINE) end
  end

  -- Grilla de suelo cercana al avion (cruces de referencia cada 40m para sensacion de velocidad a baja altura)
  local stepG = 40.0
  local baseX = floor(camX / stepG) * stepG
  local baseZ = floor(camZ / stepG) * stepG
  for gz = baseZ - 160, baseZ + 160, stepG do
    for gx = baseX - 160, baseX + 160, stepG do
      local sx0, sy0 = proj(gx - 6, 0.25, gz)
      local sx1, sy1 = proj(gx + 6, 0.25, gz)
      if sx0 and sx1 and sx0 >= 0 and sx1 < 480 and sy0 >= 0 and sy0 < 272 then
        drawLine(sx0, sy0, sx1, sy1, SOLID, COL_GND_GRID)
      end
    end
  end
end

-- Renderizado de Edificios 3D, Arboles y Gates
local function renderBuildingsAndTrees()
  for i = 1, #BUILDINGS do
    local b = BUILDINGS[i]
    local dx, dz = b.x - camX, b.z - camZ
    local cz = dx * cFx + dz * cFz
    if cz > 2.0 and cz < 650.0 then
      local hw, hd = b.w * 0.5, b.d * 0.5
      local cY, sY = b.cosY, b.sinY
      local x1, z1 = b.x + (-hw * cY - (-hd) * sY), b.z + (-hw * sY + (-hd) * cY)
      local x2, z2 = b.x + ( hw * cY - (-hd) * sY), b.z + ( hw * sY + (-hd) * cY)
      local x3, z3 = b.x + ( hw * cY - ( hd) * sY), b.z + ( hw * sY + ( hd) * cY)
      local x4, z4 = b.x + (-hw * cY - ( hd) * sY), b.z + (-hw * sY + ( hd) * cY)

      local b1x, b1y = proj(x1, 0.25, z1)
      local b2x, b2y = proj(x2, 0.25, z2)
      local b3x, b3y = proj(x3, 0.25, z3)
      local b4x, b4y = proj(x4, 0.25, z4)
      local t1x, t1y = proj(x1, b.h,  z1)
      local t2x, t2y = proj(x2, b.h,  z2)
      local t3x, t3y = proj(x3, b.h,  z3)
      local t4x, t4y = proj(x4, b.h,  z4)

      if b1x and b2x and b3x and b4x and t1x and t2x and t3x and t4x then
        if not b.open then
          sQuad(b1x, b1y, b2x, b2y, t2x, t2y, t1x, t1y, COL_WALL_L)
          sQuad(b2x, b2y, b3x, b3y, t3x, t3y, t2x, t2y, COL_WALL_D)
          sQuad(b3x, b3y, b4x, b4y, t4x, t4y, t3x, t3y, COL_WALL_L)
          sQuad(b4x, b4y, b1x, b1y, t1x, t1y, t4x, t4y, COL_WALL_D)
        else
          drawLine(b1x, b1y, t1x, t1y, SOLID, COL_WHITE)
          drawLine(b2x, b2y, t2x, t2y, SOLID, COL_WHITE)
          drawLine(b3x, b3y, t3x, t3y, SOLID, COL_WHITE)
          drawLine(b4x, b4y, t4x, t4y, SOLID, COL_WHITE)
        end
        sQuad(t1x, t1y, t2x, t2y, t3x, t3y, t4x, t4y, COL_ROOF)
      end
    end
  end

  for i = 1, #TREES do
    local tr = TREES[i]
    local dx, dz = tr.x - camX, tr.z - camZ
    local cz = dx * cFx + dz * cFz
    if cz > 2.5 and cz < 380.0 then
      local bx, by = proj(tr.x, 0.25, tr.z)
      local tx, ty = proj(tr.x, tr.h, tr.z)
      if bx and tx and bx >= 0 and bx < 480 and by >= 0 and ty < 272 then
        local hPx = max(3, abs(by - ty))
        local rPx = max(2, floor(hPx * 0.42))
        drawLine(bx, by, bx, by - floor(hPx * 0.35), SOLID, COL_TRUNK)
        drawTri(bx - rPx, by - floor(hPx * 0.28), bx + rPx, by - floor(hPx * 0.28), tx, ty, i % 2 == 0 and COL_TREE_G or COL_TREE_D)
      end
    end
  end
end

local function renderGates()
  local nG = #activeGates
  if nG == 0 then return end

  -- Dibujar solo Gate actual + 2 siguientes para mantener 30 FPS clavados
  for offset = 2, 0, -1 do
    local idx = ((wing.activeGate - 1 + offset) % nG) + 1
    local g = activeGates[idx]
    local isAct = (offset == 0)
    local col = isAct and COL_MAG or (offset == 1 and COL_CYAN or COL_GOLD)

    local hw, hh = g.w * 0.5, g.h * 0.5
    local yBot = max(0.5, g.y - hh)
    local yTop = g.y + hh
    local lx = g.x - g.nx * hw
    local lz = g.z - g.nz * hw
    local rx = g.x + g.nx * hw
    local rz = g.z + g.nz * hw

    local blx, bly = proj(lx, yBot, lz)
    local brx, bry = proj(rx, yBot, rz)
    local tlx, tly = proj(lx, yTop, lz)
    local trx, try = proj(rx, yTop, rz)

    if blx and brx and tlx and trx then
      drawLine(blx, bly, tlx, tly, SOLID, col)
      drawLine(brx, bry, trx, try, SOLID, col)
      drawLine(tlx, tly, trx, try, SOLID, col)
      if isAct then
        drawLine(blx + 1, bly, tlx + 1, tly, SOLID, COL_WHITE)
        drawLine(brx - 1, bry, trx - 1, try, SOLID, COL_WHITE)
        drawLine(tlx, tly + 1, trx, try + 1, SOLID, COL_WHITE)
        drawText(floor((tlx + trx) * 0.5) - 8, max(2, min(250, tly - 14)), sformat("G%d", idx), SMLSIZE + COL_GOLD)
      end
    end
  end

  -- Linea guia en el suelo desde la Gate actual a la siguiente
  local g0 = activeGates[wing.activeGate]
  local g1 = activeGates[(wing.activeGate % nG) + 1]
  if g0 and g1 then
    for s = 1, 5 do
      local u = s / 6.0
      local gx = g0.x + (g1.x - g0.x) * u
      local gz = g0.z + (g1.z - g0.z) * u
      local sx, sy = proj(gx, 0.5, gz)
      if sx and sx >= 4 and sx < 476 and sy >= 4 and sy < 268 then
        drawRect(sx - 2, sy - 2, 4, 4, COL_MAG)
      end
    end
  end
end

-- Modelo 3D Solido del Ala Volante / Avion con Cola en Chase Cam
local function renderWingModel3D(lx)
  local px, py, pz = wing.px, wing.py, wing.pz
  local rx, ry, rz = wing.rx, wing.ry, wing.rz
  local ux, uy, uz = wing.ux, wing.uy, wing.uz
  local fx, fy, fz = wing.fx, wing.fy, wing.fz

  local function wp(r, u, f)
    return proj(px + rx * r + ux * u + fx * f, py + ry * r + uy * u + fy * f, pz + rz * r + uz * u + fz * f)
  end

  local cfg = WING_PRESETS[presetIdx]
  if cfg.hasRudder == 0 then
    local sweepZ = cfg.fwdSweep == 1 and 0.30 or -0.35
    local nx, ny   = wp( 0.00,  0.02,  0.95)
    local tx, ty   = wp( 0.00,  0.04, -0.52)
    local rLx, rLy = wp(-0.26,  0.04,  0.18)
    local rRx, rRy = wp( 0.26,  0.04,  0.18)
    local tLx, tLy = wp(-1.45,  0.07,  sweepZ)
    local tRx, tRy = wp( 1.45,  0.07,  sweepZ)
    local wLx, wLy = wp(-1.50,  0.32,  sweepZ - 0.15)
    local wRx, wRy = wp( 1.50,  0.32,  sweepZ - 0.15)
    if nx and tx and rLx and rRx and tLx and tRx then
      local wingCol = cfg.fwdSweep == 1 and COL_WHITE or COL_RED
      sTri(rLx, rLy, tLx, tLy, tx, ty, wingCol)
      sTri(rRx, rRy, tx, ty, tRx, tRy, wingCol)
      sTri(nx, ny, rLx, rLy, tx, ty, COL_BLACK)
      sTri(nx, ny, tx, ty, rRx, rRy, COL_GREY)
      if wLx and wRx then
        sTri(tLx, tLy, tx, ty, wLx, wLy, COL_GOLD)
        sTri(tRx, tRy, wRx, wRy, tx, ty, COL_GOLD)
      end
    end
  else
    local nx, ny   = wp( 0.00, 0.02,  1.10)
    local tx, ty   = wp( 0.00, 0.04, -1.20)
    local wLx, wLy = wp(-1.55, 0.16,  0.05)
    local wRx, wRy = wp( 1.55, 0.16,  0.05)
    local rFx, rFy = wp( 0.00, 0.08,  0.28)
    local rBx, rBy = wp( 0.00, 0.06, -0.22)
    local finX, finY = wp(-lx * 0.22, 0.52, -1.25)
    local hLx, hLy = wp(-0.55, 0.08, -1.15)
    local hRx, hRy = wp( 0.55, 0.08, -1.15)
    if nx and tx and wLx and wRx and rFx and rBx then
      sTri(rFx, rFy, wLx, wLy, rBx, rBy, COL_WHITE)
      sTri(rFx, rFy, rBx, rBy, wRx, wRy, COL_WHITE)
      drawLine(nx, ny, tx, ty, SOLID, COL_RED)
      if hLx and hRx then drawLine(hLx, hLy, hRx, hRy, SOLID, COL_GOLD) end
      if finX then sTri(rBx, rBy, tx, ty, finX, finY, COL_GOLD) end
    end
  end
end

-- OSD Estilo INAV + Telemetria Completa
local fpsCount = 30
local lastFpsTime = 0
local frameCounter = 0

local function renderInavOsd()
  local cfg = WING_PRESETS[presetIdx]

  -- Reticula central INAV
  drawLine(220, 136, 233, 136, SOLID, COL_WHITE)
  drawLine(233, 136, 233, 140, SOLID, COL_WHITE)
  drawLine(233, 140, 247, 140, SOLID, COL_WHITE)
  drawLine(247, 140, 247, 136, SOLID, COL_WHITE)
  drawLine(247, 136, 260, 136, SOLID, COL_WHITE)

  -- Barra superior compacta
  drawRect(0, 0, 480, 18, COL_HUD_BG)
  local wSpdKmh = floor(sqrt(aeroEnv.windX * aeroEnv.windX + aeroEnv.windZ * aeroEnv.windZ) * 3.6)
  drawText(6, 1, sformat("W:%dK RSSI:%d%%", wSpdKmh, aeroEnv.rssiPct), SMLSIZE + COL_WHITE)
  drawText(152, 1, sformat("%s [%s]", cfg.name, CIRCUITS[circuitIdx].short), SMLSIZE + COL_GOLD)
  drawText(340, 1, sformat("%s %dFPS", halfRes and "240x136" or "480x272", fpsCount), SMLSIZE + COL_GREEN)

  -- Columna izquierda: Velocidad aire, suelo, gas y bateria LiPo 3S
  local kmh    = floor(wing.spd * 3.6)
  local airKmh = floor(aeroEnv.airSpd * 3.6)
  drawText(8, 100, sformat("AR %3d KMH", airKmh), SMLSIZE + COL_WHITE)
  drawText(8, 116, sformat("GS %3d KMH", kmh),    SMLSIZE + COL_GREEN)
  drawText(8, 132, sformat("THR %2d%%",  floor(wing.thrCmd * 100)), SMLSIZE + COL_GOLD)

  local cellV = batt.vLoad / batt.cells
  local bCol  = cellV < 3.50 and COL_RED or COL_WHITE
  drawText(8, 198, sformat("%4.1fA %4dmAh", batt.amps, floor(batt.usedMah)), SMLSIZE + COL_WHITE)
  drawText(8, 214, sformat("%5.2fV (%4.2fV/c)", batt.vLoad, cellV), SMLSIZE + bCol)

  -- Columna derecha: Altitud, Variometro, Gate y Distancia
  drawText(392, 100, sformat("ALT %3dm", floor(wing.py)), SMLSIZE + COL_WHITE)
  drawText(392, 116, sformat("VAR %+4.1f", wing.vy), SMLSIZE + (wing.vy >= 0 and COL_GREEN or COL_ORANGE))

  if #activeGates > 0 then
    local g = activeGates[wing.activeGate]
    local dxG, dzG = g.x - wing.px, g.z - wing.pz
    local distG = floor(sqrt(dxG * dxG + dzG * dzG))
    local brgRad = atan2(dxG * wing.rx + dzG * wing.rz, dxG * wing.fx + dzG * wing.fz)
    drawText(392, 198, sformat("GATE %d/%d", wing.activeGate, #activeGates), SMLSIZE + COL_GOLD)
    drawText(392, 214, sformat("DST %3dm", min(999, distG)), SMLSIZE + COL_CYAN)

    -- Flecha de direccion hacia la Gate activa
    local ax, ay = 240, 232
    local sB, cB = sin(brgRad), cos(brgRad)
    drawLine(floor(ax + sB * 8), floor(ay - cB * 8), floor(ax - sB * 6 - cB * 5), floor(ay + cB * 6 - sB * 5), SOLID, COL_GOLD)
    drawLine(floor(ax + sB * 8), floor(ay - cB * 8), floor(ax - sB * 6 + cB * 5), floor(ay + cB * 6 + sB * 5), SOLID, COL_GOLD)
  end

  -- Alertas centrales (!STALL!, !FAILSAFE!, !CRASH!)
  if wing.crashT > 0 then
    drawText(175, 76, "! CRASH - REAPARECIENDO !", SMLSIZE + COL_RED)
  elseif aeroEnv.failsafeTimer > 0 then
    drawText(165, 76, "! FAILSAFE RTH - BORDE MAPA !", SMLSIZE + COL_RED)
  elseif aeroEnv.stallWarn then
    drawText(205, 76, "! STALL WARN !", SMLSIZE + COL_RED)
  end

  if toastTimer > 0 then
    drawRect(130, 42, 220, 18, COL_HUD_BG)
    drawText(140, 44, toastMsg, SMLSIZE + COL_GOLD)
  end

  -- Pie de pantalla con controles rapidos tactiles / botones TX16S
  drawRect(0, 252, 480, 20, COL_HUD_BG)
  local aoaDeg = floor(wing.lastAlpha * RAD2DEG)
  local modeStr = wing.launchTimer > 0 and "LAUNCH" or wing.mode
  drawText(6, 254, sformat("[%s] AOA:%+dd | ENT:MODO | ROL:AVION | MDL:CIRC | TELE:CAM | SYS:RES", modeStr, aoaDeg), SMLSIZE + COL_WHITE)
end

local lastTick = 0

local function init()
  drawLine = lcd.drawLine
  drawTri  = lcd.drawFilledTriangle
  drawRect = lcd.drawFilledRectangle
  drawText = lcd.drawText
  getTime  = _G.getTime
  getValue = _G.getValue
  playTone = _G.playTone
  getFieldInfo = _G.getFieldInfo

  local RGB = lcd.RGB
  COL_SKY_TOP  = RGB(28,  72, 148)
  COL_SKY_HOR  = RGB(95, 155, 225)
  COL_GND_DARK = RGB(34,  68,  32)
  COL_GND_LITE = RGB(48,  92,  42)
  COL_GND_GRID = RGB(62, 112,  52)
  COL_RWY_ASPH = RGB(52,  56,  64)
  COL_RWY_LINE = RGB(235, 238, 242)
  COL_WHITE    = RGB(248, 250, 252)
  COL_BLACK    = RGB(16,  18,  22)
  COL_GOLD     = RGB(255, 195,  15)
  COL_GREEN    = RGB(0,   232, 115)
  COL_RED      = RGB(242,  52,  58)
  COL_CYAN     = RGB(0,   220, 250)
  COL_ORANGE   = RGB(255, 125,  20)
  COL_MAG      = RGB(245,  60, 210)
  COL_GREY     = RGB(120, 130, 145)
  COL_HUD_BG   = RGB(14,  18,  26)
  COL_WALL_L   = RGB(215, 212, 202)
  COL_WALL_D   = RGB(155, 152, 144)
  COL_ROOF     = RGB(185,  62,  44)
  COL_TREE_G   = RGB(38,  112,  42)
  COL_TREE_D   = RGB(26,   82,  30)
  COL_TRUNK    = RGB(92,   64,  38)

  if getFieldInfo then
    local fAil = getFieldInfo("ail"); if fAil then chAil = fAil.id end
    local fEle = getFieldInfo("ele"); if fEle then chEle = fEle.id end
    local fThr = getFieldInfo("thr"); if fThr then chThr = fThr.id end
    local fRud = getFieldInfo("rud"); if fRud then chRud = fRud.id end
  end

  setResolutionMode(true)
  initWorldGeometry()
  resetWing()
  lastTick = getTime()
  lastFpsTime = lastTick
end

local function handleEvents(event, touchState)
  if event == EVT_VIRTUAL_ENTER then
    if wing.mode == "ACRO" then wing.mode = "MANU"
    elseif wing.mode == "MANU" then wing.mode = "HOR"
    else wing.mode = "ACRO" end
    showToast("MODO: " .. wing.mode)
  elseif event == EVT_VIRTUAL_NEXT then
    presetIdx = (presetIdx % #WING_PRESETS) + 1
    showToast("AVION: " .. WING_PRESETS[presetIdx].name)
  elseif event == EVT_VIRTUAL_PREV then
    presetIdx = ((presetIdx - 2 + #WING_PRESETS) % #WING_PRESETS) + 1
    showToast("AVION: " .. WING_PRESETS[presetIdx].name)
  elseif event == EVT_VIRTUAL_MENU or event == EVT_MODEL_FIRST then
    circuitIdx = (circuitIdx % #CIRCUITS) + 1
    activeGates = CIRCUITS[circuitIdx].gates
    wing.activeGate = 1
    showToast("CIRCUITO: " .. CIRCUITS[circuitIdx].name)
  elseif event == EVT_TELEM_FIRST then
    wing.chaseCam = not wing.chaseCam
    showToast(wing.chaseCam and "VISTA: FOLLOW 3D" or "VISTA: FPV CABINA")
  elseif event == EVT_SYS_FIRST then
    setResolutionMode(not halfRes)
    showToast(halfRes and "RES: 240x136 (FAST 30FPS)" or "RES: 480x272 NATIVO")
  end

  if touchState and touchState.tapCount and touchState.tapCount > 0 then
    local tx, ty = touchState.x or 0, touchState.y or 0
    if ty < 32 then
      if tx < 140 then
        windIdx = (windIdx % #WIND_PRESETS) + 1
        showToast("VIENTO: " .. WIND_PRESETS[windIdx].name)
      elseif tx < 320 then
        circuitIdx = (circuitIdx % #CIRCUITS) + 1
        activeGates = CIRCUITS[circuitIdx].gates
        wing.activeGate = 1
        showToast("CIRCUITO: " .. CIRCUITS[circuitIdx].name)
      else
        setResolutionMode(not halfRes)
        showToast(halfRes and "RES: 240x136 (FAST 30FPS)" or "RES: 480x272 NATIVO")
      end
    elseif ty > 235 then
      if tx < 120 then
        if wing.mode == "ACRO" then wing.mode = "MANU"
        elseif wing.mode == "MANU" then wing.mode = "HOR"
        else wing.mode = "ACRO" end
        showToast("MODO: " .. wing.mode)
      elseif tx < 240 then
        presetIdx = (presetIdx % #WING_PRESETS) + 1
        showToast("AVION: " .. WING_PRESETS[presetIdx].name)
      elseif tx < 360 then
        wing.chaseCam = not wing.chaseCam
        showToast(wing.chaseCam and "VISTA: FOLLOW 3D" or "VISTA: FPV CABINA")
      else
        resetWing()
        showToast("HAND LAUNCH! " .. WING_PRESETS[presetIdx].name)
      end
    end
  end
end

local function run(event, touchState)
  local now = getTime()
  local dt  = (now - lastTick) * 0.01
  lastTick  = now
  if dt <= 0 or dt > 0.12 then dt = 0.033 end

  frameCounter = frameCounter + 1
  if (now - lastFpsTime) >= 100 then
    fpsCount = frameCounter
    frameCounter = 0
    lastFpsTime = now
  end
  if toastTimer > 0 then toastTimer = max(0, toastTimer - dt) end

  handleEvents(event, touchState)

  local lx, thr, rx, ry = readSticks()
  -- 2 sub-pasos de fisica por frame para estabilidad aerodinamica total a 30 FPS
  local subDt = dt * 0.5
  stepWingPhysics(subDt, lx, thr, rx, ry)
  stepWingPhysics(subDt, lx, thr, rx, ry)

  -- Configurar camara FPV o Follow 3D
  if wing.chaseCam then
    local dynDist = 5.0 + wing.thrCmd * 0.65
    camX = wing.px - wing.fx * dynDist + wing.ux * 1.25
    camY = max(0.7, wing.py - wing.fy * dynDist + wing.uy * 1.25)
    camZ = wing.pz - wing.fz * dynDist + wing.uz * 1.25
    cRx, cRy, cRz = wing.rx, wing.ry, wing.rz
    cUx, cUy, cUz = wing.ux, wing.uy, wing.uz
    cFx, cFy, cFz = wing.fx, wing.fy, wing.fz
  else
    camX, camY, camZ = wing.px, max(0.6, wing.py), wing.pz
    cRx, cRy, cRz = wing.rx, wing.ry, wing.rz
    cUx, cUy, cUz = wing.ux, wing.uy, wing.uz
    cFx, cFy, cFz = wing.fx, wing.fy, wing.fz
  end

  renderTiltedSkyAndGround()
  renderRunwayAndGrid()
  renderBuildingsAndTrees()
  renderGates()
  if wing.chaseCam then
    renderWingModel3D(lx)
  end
  renderInavOsd()

  return 0
end

return { init = init, run = run }
