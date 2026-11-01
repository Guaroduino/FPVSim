-- TNS|FPV Drone 5"|TNE
-- =============================================================================
-- FPV DRONE 5" QUAD-X 6-DOF SIMULATOR PARA RADIOMASTER TX16S (EdgeTX ColorLCD)
-- Port 1:1 de la fisica por momentos, PID Betaflight con D sobre gyro + I-Relax,
-- 4 motores independientes, AirMode, Propwash, Bateria LiPo 6S/4S con caida V,
-- Desarme por Crash + Modo Tortuga (Flip Over After Crash) y 4 Presets.
-- Optimizado para 30 FPS usando DMA2D (drawFilledTriangle) en 240x136 / 480x272.
-- =============================================================================

local sin, cos, sqrt, atan2 = math.sin, math.cos, math.sqrt, (math.atan2 or math.atan)
local floor, min, max, abs  = math.floor, math.min, math.max, math.abs
local sformat               = string.format

local drawLine, drawTri, drawRect, drawText
local getTime, getValue, playTone, getFieldInfo

local DEG2RAD = 0.01745329252
local RAD2DEG = 57.2957795131

local halfRes = true
local VW, VH  = 240, 136
local SCALE   = 2
local OX, OY  = 0, 0
local FOCAL   = 125.0

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

-- Paleta RGB565 precalculada en init()
local COL_SKY_TOP, COL_SKY_HOR, COL_GND_DARK, COL_GND_GRID
local COL_RWY_ASPH, COL_RWY_LINE, COL_WHITE, COL_BLACK, COL_GOLD, COL_GREEN
local COL_RED, COL_CYAN, COL_ORANGE, COL_MAG, COL_GREY, COL_HUD_BG
local COL_WALL_L, COL_WALL_D, COL_ROOF, COL_TREE_G, COL_TREE_D, COL_TRUNK

-- 4 Presets de FPV Drone 5" (1:1 con DRONE_PRESETS de index.html)
local DRONE_PRESETS = {
  { name = "FREESTYLE", cells = 6, capMah = 1300, rcRate = 520, yawRate = 260, centerSens = 150, expo = 0.48, massKg = 0.62, maxThrustAcc = 31.5, camTilt = 18, airMode = 1, propwash = 1.0 },
  { name = "RACING",    cells = 6, capMah = 1100, rcRate = 640, yawRate = 340, centerSens = 190, expo = 0.35, massKg = 0.48, maxThrustAcc = 39.0, camTilt = 28, airMode = 1, propwash = 0.7 },
  { name = "CINEMATIC", cells = 4, capMah = 1800, rcRate = 320, yawRate = 170, centerSens = 100, expo = 0.60, massKg = 0.78, maxThrustAcc = 25.5, camTilt = 12, airMode = 1, propwash = 0.6 },
  { name = "BANDO 6S",  cells = 6, capMah = 1300, rcRate = 780, yawRate = 400, centerSens = 180, expo = 0.52, massKg = 0.56, maxThrustAcc = 44.0, camTilt = 24, airMode = 1, propwash = 1.2 }
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

-- Bateria LiPo 6S/4S con caida resistiva bajo carga (1:1 con CRAFT_BATT de index.html)
local LIPO_SOC = { 0.00, 0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00 }
local LIPO_OCV = { 3.00, 3.30, 3.45, 3.60, 3.72, 3.82, 3.98, 4.20 }
local batt = { cells = 6, capMah = 1300, usedMah = 0, rIntCell = 0.006, amps = 0, vRest = 25.2, vLoad = 25.2, lowWarnT = -99 }

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

-- Coordenadas Chorobobo HD (2178x1452m)
local WORLD_W_M = 2178.0
local WORLD_H_M = 1452.0
local SPAWN_U   = 0.585
local SPAWN_V   = 0.365

local function uvToWorldX(u) return (u - SPAWN_U) * WORLD_W_M end
local function uvToWorldZ(v) return (SPAWN_V - v) * WORLD_H_M end

-- 6 Edificios de Chorobobo + 2 Estructuras Bando abiertas cerca de pista
local BUILDINGS = {
  { u = 0.6214, v = 0.3807, w = 11, d =  4, h = 3.3, yaw = -0.17, open = true  }, -- Toldo Pits A.I.A
  { u = 0.5485, v = 0.3518, w =  4, d =  4, h = 9.7, yaw =  1.47, open = false }, -- Torre de Agua
  { u = 0.5450, v = 0.4815, w = 10, d = 38, h = 6.7, yaw =  1.57, open = false }, -- Galpon 1
  { u = 0.5468, v = 0.4679, w = 10, d = 37, h = 6.7, yaw =  1.61, open = false }, -- Galpon 2
  { u = 0.5480, v = 0.4500, w = 17, d = 11, h = 6.7, yaw =  1.57, open = false }, -- Galpon 3
  { u = 0.5431, v = 0.4542, w = 13, d =  9, h = 6.7, yaw =  1.54, open = false }  -- Galpon 4
}

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
  { x = -165,z =  75, h = 8.2, w = 6.5 }, { x = -85, z =  92, h = 7.4, w = 5.8 }
}

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

local aeroEnv = {
  time = 0, windX = 2.5, windY = 0, windZ = 1.2,
  turbPitch = 0, turbRoll = 0, turbYaw = 0,
  joltX = 0, joltY = 0, joltZ = 0,
  airSpd = 8, rssiPct = 99, rssiDbm = -50, failsafeTimer = 0
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

  local spdFactor = 0.70 + 0.30 * min(2.2, (aeroEnv.airSpd or 8.0) / 14.0)
  local turbAmp = 1.15 * wCfg.turbPct * spdFactor
  aeroEnv.turbRoll  = (0.48 * sin(t * 2.35 + px * 0.035) + 0.34 * cos(t * 5.2 + pz * 0.055)) * turbAmp
  aeroEnv.turbPitch = (0.40 * cos(t * 2.05 + pz * 0.032) + 0.28 * sin(t * 4.8 - px * 0.050)) * turbAmp
  aeroEnv.turbYaw   = (0.36 * sin(t * 1.85 - px * 0.036) + 0.25 * cos(t * 4.4 + pz * 0.050)) * turbAmp

  aeroEnv.joltX = sin(t * 7.8) * 1.4 * wCfg.turbPct
  aeroEnv.joltY = cos(t * 8.6) * 1.8 * wCfg.turbPct
  aeroEnv.joltZ = cos(t * 7.2) * 1.4 * wCfg.turbPct
end

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

-- Curva Actual Rates de Betaflight
local function evalActualRateRad(rc, centerDegS, maxDegS, expo)
  local absRc = abs(rc)
  local abs3  = absRc * absRc * absRc
  local expoF = absRc * (abs3 * expo + absRc * (1.0 - expo))
  local sign  = rc >= 0 and 1.0 or -1.0
  local degS  = rc * centerDegS + sign * max(0.0, maxDegS - centerDegS) * expoF
  return degS * DEG2RAD
end

-- Estado del Dron FPV 5" (1:1 con fpvDroneSim de index.html)
local drone = {
  px = 0, py = 12, pz = -20,
  vx = 0, vy = 0, vz = 0,
  pitchRate = 0, rollRate = 0, yawRate = 0,
  rx = 1, ry = 0, rz = 0,
  ux = 0, uy = 1, uz = 0,
  fx = 0, fy = 0, fz = 1,
  mode = "ACRO",
  chaseCam = false,
  activeGate = 1,
  thrCmd = 0.35, motorRpm = 0.36,
  mRpm = { 0.36, 0.36, 0.36, 0.36 },
  mCmd = { 0.36, 0.36, 0.36, 0.36 },
  iPitch = 0, iRoll = 0, iYaw = 0,
  lastGyroP = 0, lastGyroR = 0, lastAmps = 0,
  disarmed = false, crashT = 0, turtleT = 0, turtleDir = 1,
  flightTime = 0, totalDistKm = 0
}

local toastMsg   = ""
local toastTimer = 0
local function showToast(msg)
  toastMsg   = msg
  toastTimer = 1.5
end

local function resetDrone()
  local cfg = DRONE_PRESETS[presetIdx]
  drone.px = uvToWorldX(0.5967)
  drone.pz = uvToWorldZ(0.3791)
  drone.py = 12.25
  local rad = 17.0 * DEG2RAD
  local fx0 = cos(rad)
  local fz0 = -sin(rad)
  drone.vx = fx0 * 8.0; drone.vy = 0; drone.vz = fz0 * 8.0
  drone.fx = fx0; drone.fy = 0; drone.fz = fz0
  drone.ux = 0;   drone.uy = 1; drone.uz = 0
  drone.rx = fz0; drone.ry = 0; drone.rz = -fx0

  drone.pitchRate = 0; drone.rollRate = 0; drone.yawRate = 0
  drone.motorRpm = 0.36
  for i = 1, 4 do drone.mRpm[i] = 0.36; drone.mCmd[i] = 0.36 end
  drone.iPitch = 0; drone.iRoll = 0; drone.iYaw = 0
  drone.lastGyroP = 0; drone.lastGyroR = 0; drone.lastAmps = 0
  drone.disarmed = false; drone.crashT = 0; drone.turtleT = 0
  drone.activeGate = 1
  drone.flightTime = 0; drone.totalDistKm = 0
  battReset(cfg.cells, cfg.capMah, 0.006)
end

local function checkWorldCollision(prevX, prevY, prevZ)
  local px, py, pz = drone.px, drone.py, drone.pz
  for i = 1, #BUILDINGS do
    local b = BUILDINGS[i]
    if not b.open and py <= b.h + 0.4 then
      local dx, dz = px - b.x, pz - b.z
      local lx =  dx * b.cosY + dz * b.sinY
      local lz = -dx * b.sinY + dz * b.cosY
      if abs(lx) <= (b.w * 0.5 + 0.35) and abs(lz) <= (b.d * 0.5 + 0.35) then
        drone.px, drone.py, drone.pz = prevX, prevY, prevZ
        drone.vx, drone.vy, drone.vz = -drone.vx * 0.35, -drone.vy * 0.35, -drone.vz * 0.35
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
        drone.px, drone.py, drone.pz = prevX, prevY, prevZ
        drone.vx, drone.vy, drone.vz = -drone.vx * 0.35, -drone.vy * 0.35, -drone.vz * 0.35
        return true
      end
    end
  end
  return false
end

local function checkGatePass(g, prevX, prevY, prevZ, px, py, pz)
  if not g then return false end
  local d0 = (prevX - g.x) * g.tx + (prevZ - g.z) * g.tz
  local d1 = (px - g.x) * g.tx + (pz - g.z) * g.tz
  if not (((d0 <= 0 and d1 >= 0) or (d0 >= 0 and d1 <= 0)) and abs(d1 - d0) < 16.0) then
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

local function stepDronePhysics(dt, lx, thr, rx, ry)
  drone.flightTime = drone.flightTime + dt
  local cfg = DRONE_PRESETS[presetIdx]
  local groundH = 0.25

  updateAeroEnvironment(drone.px, drone.py, drone.pz, dt)

  drone.thrCmd = thr
  local armed = not drone.disarmed

  local vAirX = drone.vx - aeroEnv.windX
  local vAirY = drone.vy - aeroEnv.windY
  local vAirZ = drone.vz - aeroEnv.windZ
  local airSpd = sqrt(vAirX * vAirX + vAirY * vAirY + vAirZ * vAirZ)
  aeroEnv.airSpd = airSpd

  local vBodyR = vAirX * drone.rx + vAirY * drone.ry + vAirZ * drone.rz
  local vBodyU = vAirX * drone.ux + vAirY * drone.uy + vAirZ * drone.uz
  local vBodyF = vAirX * drone.fx + vAirY * drone.fy + vAirZ * drone.fz
  local horizAirSpd = sqrt(vBodyR * vBodyR + vBodyF * vBodyF)

  local curPitch = -drone.fy
  local curRoll  = atan2(-drone.ry, max(0.12, drone.uy))

  local ratePitchCmd = evalActualRateRad(-ry, cfg.centerSens, cfg.rcRate, cfg.expo)
  local rateRollCmd  = evalActualRateRad( rx, cfg.centerSens, cfg.rcRate, cfg.expo)
  local tYaw         = evalActualRateRad( lx, min(140, cfg.yawRate * 0.55), cfg.yawRate, cfg.expo * 0.75)

  local tPitch, tRoll = ratePitchCmd, rateRollCmd
  if aeroEnv.failsafeTimer > 0 then
    local toHomeX, toHomeZ = -drone.px, -drone.pz
    local homeDist = max(1.0, sqrt(toHomeX * toHomeX + toHomeZ * toHomeZ))
    local dirHomeF = (toHomeX * drone.fx + toHomeZ * drone.fz) / homeDist
    local dirHomeR = (toHomeX * drone.rx + toHomeZ * drone.rz) / homeDist
    tPitch = ((dirHomeF * 18.0 * DEG2RAD) - curPitch) * 4.5
    tRoll  = ((dirHomeR * 22.0 * DEG2RAD) - curRoll)  * 4.5
  elseif drone.mode ~= "ACRO" then
    local targetPitchAng = max(-42.0 * DEG2RAD, min(42.0 * DEG2RAD, -ry * 40.0 * DEG2RAD))
    local targetRollAng  = max(-52.0 * DEG2RAD, min(52.0 * DEG2RAD,  rx * 50.0 * DEG2RAD))
    tPitch = (targetPitchAng - curPitch) * 7.0
    tRoll  = (targetRollAng  - curRoll)  * 7.5
  end

  local propwashRoll, propwashPitch = 0, 0
  if vBodyU < -2.8 and horizAirSpd < 4.5 then
    local pwInt = min(0.65, (-2.8 - vBodyU) * 0.16 * (1.0 - horizAirSpd / 4.5)) * cfg.propwash
    propwashRoll  = sin(drone.flightTime * 24.0) * 20.0 * pwInt
    propwashPitch = cos(drone.flightTime * 21.0) * 16.0 * pwInt
  end

  local vRatio = battStep(drone.lastAmps > 0 and drone.lastAmps or 0.8, dt, aeroEnv.time)

  -- Lazo interno PID de rate (2 sub-steps por llamada = 4x por frame)
  local SUB = 2
  local h = dt / SUB
  local invInertia = 0.62 / max(0.30, cfg.massKg)
  local KP, KI, KD, KF = 0.25, 0.16, 0.0060, 0.015
  local KPY, KIY, KFY  = 0.30, 0.12, 0.030
  local relaxPR = max(0.1, 1.0 - sqrt(rx * rx + ry * ry) * 0.85)
  local thrustSum, ampSum = 0, 0
  local mRpm, mCmd = drone.mRpm, drone.mCmd

  for _ = 1, SUB do
    local uPitch, uRoll, uYaw = 0, 0, 0
    if armed then
      local errPitch = tPitch - drone.pitchRate
      local errRoll  = tRoll  - drone.rollRate
      local errYaw   = tYaw   - drone.yawRate
      drone.iPitch = max(-0.20, min(0.20, drone.iPitch + errPitch * KI * relaxPR * h))
      drone.iRoll  = max(-0.20, min(0.20, drone.iRoll  + errRoll  * KI * relaxPR * h))
      drone.iYaw   = max(-0.16, min(0.16, drone.iYaw   + errYaw   * KIY * h))
      local dPitch = -KD * (drone.pitchRate - drone.lastGyroP) / h
      local dRoll  = -KD * (drone.rollRate  - drone.lastGyroR) / h
      uPitch = max(-0.65, min(0.65, errPitch * KP + drone.iPitch + dPitch + tPitch * KF))
      uRoll  = max(-0.65, min(0.65, errRoll  * KP + drone.iRoll  + dRoll  + tRoll  * KF))
      uYaw   = max(-0.45, min(0.45, errYaw   * KPY + drone.iYaw  + tYaw   * KFY))
    end
    drone.lastGyroP = drone.pitchRate
    drone.lastGyroR = drone.rollRate

    local d1 = -uRoll - uPitch - uYaw
    local d2 = -uRoll + uPitch + uYaw
    local d3 =  uRoll - uPitch + uYaw
    local d4 =  uRoll + uPitch - uYaw
    local dMin = min(min(d1, d2), min(d3, d4))
    local dMax = max(max(d1, d2), max(d3, d4))
    local span = dMax - dMin
    local mixScale, thrMix = 1.0, thr
    if cfg.airMode == 1 then
      if span > 0.95 then mixScale = 0.95 / span end
      dMin = dMin * mixScale; dMax = dMax * mixScale
      if thrMix + dMin < 0.05 then thrMix = 0.05 - dMin end
      if thrMix + dMax > 1.00 then thrMix = 1.00 - dMax end
    else
      mixScale = min(1.0, thr / 0.22)
    end

    if armed then
      mCmd[1] = max(0.04, min(1.0, thrMix + d1 * mixScale)) * vRatio
      mCmd[2] = max(0.04, min(1.0, thrMix + d2 * mixScale)) * vRatio
      mCmd[3] = max(0.04, min(1.0, thrMix + d3 * mixScale)) * vRatio
      mCmd[4] = max(0.04, min(1.0, thrMix + d4 * mixScale)) * vRatio
    else
      mCmd[1], mCmd[2], mCmd[3], mCmd[4] = 0, 0, 0, 0
    end

    for i = 1, 4 do
      local prev, cmd = mRpm[i], mCmd[i]
      mRpm[i] = prev + (cmd - prev) * min(1.0, (cmd >= prev and 38.0 or 28.0) * h)
    end

    local T0 = 0.35 * mRpm[1] + 0.65 * mRpm[1] * mRpm[1]
    local T1 = 0.35 * mRpm[2] + 0.65 * mRpm[2] * mRpm[2]
    local T2 = 0.35 * mRpm[3] + 0.65 * mRpm[3] * mRpm[3]
    local T3 = 0.35 * mRpm[4] + 0.65 * mRpm[4] * mRpm[4]
    thrustSum = thrustSum + (T0 + T1 + T2 + T3) * 0.25
    ampSum = ampSum + 32.5 * (T0 * sqrt(T0) + T1 * sqrt(T1) + T2 * sqrt(T2) + T3 * sqrt(T3)) + 0.8

    local tauRoll  = ((T2 + T3) - (T0 + T1)) * 50.0 * invInertia
    local tauPitch = ((T1 + T3) - (T0 + T2)) * 48.0 * invInertia
    local tauYaw   = ((T1 + T2) - (T0 + T3)) * 32.0 * invInertia

    local netPropOmega = (mRpm[2] + mRpm[3] - mRpm[1] - mRpm[4]) * 4.0
    local gyroPitch  =  drone.rollRate  * netPropOmega * 0.04
    local gyroRoll   = -drone.pitchRate * netPropOmega * 0.04
    local eulerPitch = -0.18 * drone.rollRate  * drone.yawRate
    local eulerRoll  =  0.18 * drone.pitchRate * drone.yawRate
    local rotDampPR  = 1.2 + 0.08 * airSpd
    local rotDampY   = 2.0

    drone.pitchRate = drone.pitchRate + (tauPitch + gyroPitch + eulerPitch + aeroEnv.turbPitch * 1.12 + propwashPitch - rotDampPR * drone.pitchRate) * h
    drone.rollRate  = drone.rollRate  + (tauRoll  + gyroRoll  + eulerRoll  + aeroEnv.turbRoll  * 1.28 + propwashRoll  - rotDampPR * drone.rollRate) * h
    drone.yawRate   = drone.yawRate   + (tauYaw   + aeroEnv.turbYaw   * 0.88 - rotDampY * drone.yawRate) * h
    if drone.turtleT > 0 then drone.rollRate = drone.turtleDir * 3.2 end
    stepRotationSO3(drone, h)
  end

  drone.lastAmps = ampSum / SUB
  drone.motorRpm = (mRpm[1] + mRpm[2] + mRpm[3] + mRpm[4]) * 0.25

  local thrustNorm   = thrustSum / SUB
  local agl          = max(0.1, drone.py - groundH)
  local groundEffect = agl < 1.6 and (1.0 + (1.6 - agl) * 0.095) or 1.0
  local transLift    = horizAirSpd > 2.0 and (1.0 + min(0.15, horizAirSpd * 0.013)) or 1.0
  local inflowLoss   = max(0.58, 1.0 - max(0.0, vBodyU) * 0.022)
  local massScale    = 0.62 / max(0.30, cfg.massKg)
  local thrustAcc    = thrustNorm * cfg.maxThrustAcc * massScale * groundEffect * transLift * inflowLoss

  local bladeDrag = 0.095 * drone.motorRpm
  local dragFR    = (0.11 + 0.0105 * airSpd + bladeDrag) * massScale
  local dragU     = (0.28 + 0.024  * airSpd) * massScale
  local fDragR    = -vBodyR * dragFR
  local fDragU    = -vBodyU * dragU
  local fDragF    = -vBodyF * dragFR

  local ax = drone.ux * thrustAcc + drone.rx * fDragR + drone.ux * fDragU + drone.fx * fDragF + aeroEnv.joltX * 0.25 * massScale
  local ay = drone.uy * thrustAcc + drone.ry * fDragR + drone.uy * fDragU + drone.fy * fDragF - 9.81 + aeroEnv.joltY * 0.25 * massScale
  local az = drone.uz * thrustAcc + drone.rz * fDragR + drone.uz * fDragU + drone.fz * fDragF + aeroEnv.joltZ * 0.25 * massScale

  local preVx, preVy, preVz = drone.vx, drone.vy, drone.vz
  drone.vx = drone.vx + ax * dt
  drone.vy = drone.vy + ay * dt
  drone.vz = drone.vz + az * dt

  local gndSpd = sqrt(drone.vx * drone.vx + drone.vy * drone.vy + drone.vz * drone.vz)
  drone.totalDistKm = drone.totalDistKm + (gndSpd * dt) * 0.001

  local prevX, prevY, prevZ = drone.px, drone.py, drone.pz
  drone.px = drone.px + drone.vx * dt
  drone.py = drone.py + drone.vy * dt
  drone.pz = drone.pz + drone.vz * dt

  local onGround = false
  if drone.py < groundH + 0.30 then
    drone.py = groundH + 0.30
    onGround = true
    if drone.vy < 0 then drone.vy = -drone.vy * 0.22 end
    drone.vx = drone.vx * 0.90
    drone.vz = drone.vz * 0.90
  end
  checkWorldCollision(prevX, prevY, prevZ)

  local dvx, dvy, dvz = drone.vx - preVx, drone.vy - preVy, drone.vz - preVz
  local impactDv = sqrt(dvx * dvx + dvy * dvy + dvz * dvz)
  if armed and impactDv > 10.0 then
    drone.disarmed = true
    drone.crashT = 0.0
    drone.iPitch, drone.iRoll, drone.iYaw = 0, 0, 0
    showToast(sformat("CRASH %d KMH - DESARMADO", floor(impactDv * 3.6)))
    if playTone then playTone(140, 220, 0) end
  end

  if drone.disarmed then
    drone.crashT = drone.crashT + dt
    if onGround then
      drone.vx = drone.vx * 0.85; drone.vz = drone.vz * 0.85
      if drone.turtleT <= 0 then
        drone.pitchRate = drone.pitchRate * 0.80
        drone.rollRate  = drone.rollRate  * 0.80
        drone.yawRate   = drone.yawRate   * 0.80
      end
      if drone.uy <= 0.5 then
        if abs(rx) > 0.5 and drone.crashT > 0.5 then
          drone.turtleT   = 0.14
          drone.turtleDir = rx > 0 and 1.0 or -1.0
        end
      elseif drone.crashT > 1.1 then
        drone.disarmed = false
        drone.turtleT  = 0
        showToast("REARMADO OK!")
        if playTone then playTone(680, 80, 0) end
      end
    end
    if drone.turtleT > 0 then drone.turtleT = drone.turtleT - dt end
  end

  if #activeGates > 0 then
    local g = activeGates[drone.activeGate]
    if checkGatePass(g, prevX, prevY, prevZ, drone.px, drone.py, drone.pz) then
      drone.activeGate = (drone.activeGate % #activeGates) + 1
      showToast(sformat("%s G%d/%d OK!", CIRCUITS[circuitIdx].short, drone.activeGate, #activeGates))
      if playTone then playTone(880, 70, 0) end
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

local function renderTiltedSkyAndGround()
  local W, H = 480, 272
  local phi = atan2(cRy, cUy)
  local cosR, sinR = cos(phi), sin(phi)
  local hy = (VH * 0.5 + (cFy / max(0.15, sqrt(cFx * cFx + cFz * cFz))) * FOCAL * 0.85) * SCALE
  local cy = max(0, min(H, floor(hy)))

  local topCol = cUy >= 0 and COL_SKY_TOP or COL_GND_DARK
  local botCol = cUy >= 0 and COL_GND_DARK or COL_SKY_TOP

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
    drawRect(0, yMin, W, yMax - yMin, botCol)
    if yL < yR then
      drawTri(0, yL, W, yL, W, yR, topCol)
    else
      drawTri(0, yL, 0, yR, W, yR, topCol)
    end
  end
  drawLine(0, yL, W - 1, yR, SOLID, COL_SKY_HOR)
end

local function renderRunwayAndGrid()
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

  for s = -110, 110, 28 do
    local ax, ay = rwyPt(s, 0)
    local bx, by = rwyPt(s + 14, 0)
    if ax and bx then drawLine(ax, ay, bx, by, SOLID, COL_RWY_LINE) end
  end

  local stepG = 35.0
  local baseX = floor(camX / stepG) * stepG
  local baseZ = floor(camZ / stepG) * stepG
  for gz = baseZ - 140, baseZ + 140, stepG do
    for gx = baseX - 140, baseX + 140, stepG do
      local sx0, sy0 = proj(gx - 5, 0.25, gz)
      local sx1, sy1 = proj(gx + 5, 0.25, gz)
      if sx0 and sx1 and sx0 >= 0 and sx1 < 480 and sy0 >= 0 and sy0 < 272 then
        drawLine(sx0, sy0, sx1, sy1, SOLID, COL_GND_GRID)
      end
    end
  end
end

local function renderBuildingsAndTrees()
  for i = 1, #BUILDINGS do
    local b = BUILDINGS[i]
    local dx, dz = b.x - camX, b.z - camZ
    local cz = dx * cFx + dz * cFz
    if cz > 1.5 and cz < 580.0 then
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
    if cz > 2.0 and cz < 340.0 then
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

  for offset = 2, 0, -1 do
    local idx = ((drone.activeGate - 1 + offset) % nG) + 1
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

  local g0 = activeGates[drone.activeGate]
  local g1 = activeGates[(drone.activeGate % nG) + 1]
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

-- Modelo 3D Solido del Dron Quad-X 5" en Chase Cam
local ARM_R = {  0.48,  0.48, -0.48, -0.48 }
local ARM_F = { -0.44,  0.44, -0.44,  0.44 }

local function renderDroneModel3D()
  local px, py, pz = drone.px, drone.py, drone.pz
  local rx, ry, rz = drone.rx, drone.ry, drone.rz
  local ux, uy, uz = drone.ux, drone.uy, drone.uz
  local fx, fy, fz = drone.fx, drone.fy, drone.fz

  local function wp(r, u, f)
    return proj(px + rx * r + ux * u + fx * f, py + ry * r + uy * u + fy * f, pz + rz * r + uz * u + fz * f)
  end

  local cx, cy   = wp(0, 0.02, 0)
  local flx, fly = wp(-0.14, 0.04,  0.26)
  local frx, fry = wp( 0.14, 0.04,  0.26)
  local rlx, rly = wp(-0.14, 0.04, -0.26)
  local rrx, rry = wp( 0.14, 0.04, -0.26)
  if flx and frx and rlx and rrx then
    sQuad(flx, fly, frx, fry, rrx, rry, rlx, rly, COL_BLACK)
  end

  for m = 1, 4 do
    local ar, af = ARM_R[m], ARM_F[m]
    local mx, my = wp(ar, 0.04, af)
    if cx and mx then
      drawLine(cx, cy, mx, my, SOLID, COL_GREY)
      local ang = drone.flightTime * (28.0 + drone.mRpm[m] * 55.0) + m * 1.1
      local p1x, p1y = wp(ar + cos(ang) * 0.22, 0.06, af + sin(ang) * 0.22)
      local p2x, p2y = wp(ar - cos(ang) * 0.22, 0.06, af - sin(ang) * 0.22)
      if p1x and p2x then
        drawLine(p1x, p1y, p2x, p2y, SOLID, m <= 2 and COL_ORANGE or COL_GREEN)
      end
    end
  end
end

local fpsCount = 30
local lastFpsTime = 0
local frameCounter = 0

local function renderBetaflightOsd()
  local cfg = DRONE_PRESETS[presetIdx]
  local spd = sqrt(drone.vx * drone.vx + drone.vy * drone.vy + drone.vz * drone.vz)

  -- Reticula central Betaflight
  drawLine(222, 136, 234, 136, SOLID, COL_WHITE)
  drawLine(246, 136, 258, 136, SOLID, COL_WHITE)
  drawLine(240, 128, 240, 144, SOLID, COL_WHITE)

  -- Barra superior
  drawRect(0, 0, 480, 18, COL_HUD_BG)
  local wSpdKmh = floor(sqrt(aeroEnv.windX * aeroEnv.windX + aeroEnv.windZ * aeroEnv.windZ) * 3.6)
  drawText(6, 1, sformat("W:%dK RSSI:%d%%", wSpdKmh, aeroEnv.rssiPct), SMLSIZE + COL_WHITE)
  drawText(146, 1, sformat("Q5 %s [%s]", cfg.name, CIRCUITS[circuitIdx].short), SMLSIZE + COL_GOLD)
  drawText(340, 1, sformat("%s %dFPS", halfRes and "240x136" or "480x272", fpsCount), SMLSIZE + COL_GREEN)

  -- Columna izquierda: Velocidad, Gas, Bateria 6S/4S y 4 Barras RPM de motor
  drawText(8, 100, sformat("GS  %3d KMH", floor(spd * 3.6)), SMLSIZE + COL_GREEN)
  drawText(8, 116, sformat("THR %2d%%", floor(drone.thrCmd * 100)), SMLSIZE + COL_GOLD)
  drawText(8, 132, sformat("TLT %2d DEG", cfg.camTilt), SMLSIZE + COL_CYAN)

  local cellV = batt.vLoad / batt.cells
  local bCol  = cellV < 3.50 and COL_RED or COL_WHITE
  drawText(8, 198, sformat("%5.1fA %4dmAh", batt.amps, floor(batt.usedMah)), SMLSIZE + COL_WHITE)
  drawText(8, 214, sformat("%dS %5.2fV (%4.2fV)", batt.cells, batt.vLoad, cellV), SMLSIZE + bCol)

  -- 4 Barras de RPM reales (M1..M4)
  for m = 1, 4 do
    local bx = 204 + (m - 1) * 18
    local h  = max(2, min(20, floor(drone.mRpm[m] * 20)))
    drawRect(bx, 226, 12, 22, COL_HUD_BG)
    drawRect(bx + 2, 246 - h, 8, h, drone.mRpm[m] > 0.88 and COL_ORANGE or COL_GREEN)
  end

  -- Columna derecha: Altitud, Variometro, Gate
  drawText(392, 100, sformat("ALT %3dm", floor(drone.py)), SMLSIZE + COL_WHITE)
  drawText(392, 116, sformat("VAR %+4.1f", drone.vy), SMLSIZE + (drone.vy >= 0 and COL_GREEN or COL_ORANGE))

  if #activeGates > 0 then
    local g = activeGates[drone.activeGate]
    local dxG, dzG = g.x - drone.px, g.z - drone.pz
    local distG = floor(sqrt(dxG * dxG + dzG * dzG))
    local brgRad = atan2(dxG * drone.rx + dzG * drone.rz, dxG * drone.fx + dzG * drone.fz)
    drawText(392, 198, sformat("GATE %d/%d", drone.activeGate, #activeGates), SMLSIZE + COL_GOLD)
    drawText(392, 214, sformat("DST %3dm", min(999, distG)), SMLSIZE + COL_CYAN)

    local ax, ay = 240, 212
    local sB, cB = sin(brgRad), cos(brgRad)
    drawLine(floor(ax + sB * 8), floor(ay - cB * 8), floor(ax - sB * 6 - cB * 5), floor(ay + cB * 6 - sB * 5), SOLID, COL_GOLD)
    drawLine(floor(ax + sB * 8), floor(ay - cB * 8), floor(ax - sB * 6 + cB * 5), floor(ay + cB * 6 + sB * 5), SOLID, COL_GOLD)
  end

  -- Alertas centrales (!DISARMED / MODO TORTUGA!)
  if drone.disarmed then
    if drone.uy <= 0.5 then
      drawText(130, 74, "! CRASH BOCA ABAJO: MUEVE ROLL (TORTUGA) !", SMLSIZE + COL_RED)
    else
      drawText(175, 74, "! DESARMADO - REARMANDO... !", SMLSIZE + COL_ORANGE)
    end
  elseif aeroEnv.failsafeTimer > 0 then
    drawText(165, 74, "! FAILSAFE RTH - BORDE MAPA !", SMLSIZE + COL_RED)
  end

  if toastTimer > 0 then
    drawRect(130, 42, 220, 18, COL_HUD_BG)
    drawText(140, 44, toastMsg, SMLSIZE + COL_GOLD)
  end

  drawRect(0, 252, 480, 20, COL_HUD_BG)
  drawText(6, 254, sformat("[%s+AIR] | ENT:MODO | ROL:PRESET | MDL:CIRC | TELE:CAM | SYS:RES", drone.mode), SMLSIZE + COL_WHITE)
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
  COL_SKY_TOP  = RGB(24,  64, 142)
  COL_SKY_HOR  = RGB(90, 150, 220)
  COL_GND_DARK = RGB(34,  68,  32)
  COL_GND_GRID = RGB(62, 112,  52)
  COL_RWY_ASPH = RGB(52,  56,  64)
  COL_RWY_LINE = RGB(235, 238, 242)
  COL_WHITE    = RGB(248, 250, 252)
  COL_BLACK    = RGB(22,  26,  32)
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
  resetDrone()
  lastTick = getTime()
  lastFpsTime = lastTick
end

local function handleEvents(event, touchState)
  if event == EVT_VIRTUAL_ENTER then
    drone.mode = (drone.mode == "ACRO") and "HOR" or "ACRO"
    showToast("MODO DRON: " .. drone.mode)
  elseif event == EVT_VIRTUAL_NEXT then
    presetIdx = (presetIdx % #DRONE_PRESETS) + 1
    resetDrone()
    showToast("DRON: " .. DRONE_PRESETS[presetIdx].name)
  elseif event == EVT_VIRTUAL_PREV then
    presetIdx = ((presetIdx - 2 + #DRONE_PRESETS) % #DRONE_PRESETS) + 1
    resetDrone()
    showToast("DRON: " .. DRONE_PRESETS[presetIdx].name)
  elseif event == EVT_VIRTUAL_MENU or event == EVT_MODEL_FIRST then
    circuitIdx = (circuitIdx % #CIRCUITS) + 1
    activeGates = CIRCUITS[circuitIdx].gates
    drone.activeGate = 1
    showToast("CIRCUITO: " .. CIRCUITS[circuitIdx].name)
  elseif event == EVT_TELEM_FIRST then
    drone.chaseCam = not drone.chaseCam
    showToast(drone.chaseCam and "VISTA: FOLLOW 3D" or "VISTA: FPV CAM")
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
        drone.activeGate = 1
        showToast("CIRCUITO: " .. CIRCUITS[circuitIdx].name)
      else
        setResolutionMode(not halfRes)
        showToast(halfRes and "RES: 240x136 (FAST 30FPS)" or "RES: 480x272 NATIVO")
      end
    elseif ty > 235 then
      if tx < 120 then
        drone.mode = (drone.mode == "ACRO") and "HOR" or "ACRO"
        showToast("MODO DRON: " .. drone.mode)
      elseif tx < 240 then
        presetIdx = (presetIdx % #DRONE_PRESETS) + 1
        resetDrone()
        showToast("DRON: " .. DRONE_PRESETS[presetIdx].name)
      elseif tx < 360 then
        drone.chaseCam = not drone.chaseCam
        showToast(drone.chaseCam and "VISTA: FOLLOW 3D" or "VISTA: FPV CAM")
      else
        resetDrone()
        showToast("RESET QUAD-X 5\"")
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
  local subDt = dt * 0.5
  stepDronePhysics(subDt, lx, thr, rx, ry)
  stepDronePhysics(subDt, lx, thr, rx, ry)

  -- Camara con angulo camTilt en FPV o Chase Cam 3D
  local cfg = DRONE_PRESETS[presetIdx]
  if drone.chaseCam then
    local dynDist = 4.6 + drone.thrCmd * 0.55
    camX = drone.px - drone.fx * dynDist + drone.ux * 1.15
    camY = max(0.6, drone.py - drone.fy * dynDist + drone.uy * 1.15)
    camZ = drone.pz - drone.fz * dynDist + drone.uz * 1.15
    cRx, cRy, cRz = drone.rx, drone.ry, drone.rz
    cUx, cUy, cUz = drone.ux, drone.uy, drone.uz
    cFx, cFy, cFz = drone.fx, drone.fy, drone.fz
  else
    local tilt = cfg.camTilt * DEG2RAD
    local cT, sT = cos(tilt), sin(tilt)
    camX, camY, camZ = drone.px, max(0.5, drone.py), drone.pz
    cRx, cRy, cRz = drone.rx, drone.ry, drone.rz
    cUx = cT * drone.ux - sT * drone.fx
    cUy = cT * drone.uy - sT * drone.fy
    cUz = cT * drone.uz - sT * drone.fz
    cFx = sT * drone.ux + cT * drone.fx
    cFy = sT * drone.uy + cT * drone.fy
    cFz = sT * drone.uz + cT * drone.fz
  end

  renderTiltedSkyAndGround()
  renderRunwayAndGrid()
  renderBuildingsAndTrees()
  renderGates()
  if drone.chaseCam then
    renderDroneModel3D()
  end
  renderBetaflightOsd()

  return 0
end

return { init = init, run = run }
