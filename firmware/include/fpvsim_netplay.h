#pragma once
#include <stdint.h>

// ============================================================================
// MULTIJUGADOR LOCAL POR ESP-NOW (consola a consola, sin router ni emparejar)
// Cada consola difunde su pose ~30 veces por segundo en el canal Wi-Fi 1.
// Latencia típica 1-2 ms; paquete de 40 B. Usa ~40-60 KB de SRAM interna para el Wi-Fi.
// ============================================================================
constexpr int NETPLAY_MAX_PEERS = 4;

struct NetPeer {
  bool     active;
  uint8_t  mac[6];
  uint32_t lastSeenMs;
  float    px, py, pz;
  float    qw, qx, qy, qz;
  uint8_t  gate;
  uint8_t  craft;   // 0 = dron, 1 = ala, ...
  uint16_t mapId;   // CRC16 del nombre del mapa: solo se dibujan pares en el mismo mapa
};

bool netplayBegin();
void netplaySendPose(float px, float py, float pz, float qw, float qx, float qy, float qz,
                     uint8_t gate, uint8_t craft, uint16_t mapId);
// Copia los pares activos (vistos en los últimos 2 s). Devuelve cuántos.
int  netplayGetPeers(NetPeer* out, int maxPeers);
uint16_t netplayMapId(const char* mapName);
