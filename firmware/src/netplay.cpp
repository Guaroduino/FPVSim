#include "fpvsim_hal.h"
#include "fpvsim_netplay.h"
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>

static constexpr uint32_t NET_MAGIC   = 0x31565046u; // "FPV1"
static constexpr uint8_t  NET_CHANNEL = 1;

struct __attribute__((packed)) PosePacket {
  uint32_t magic;
  uint16_t mapId;
  uint8_t  gate;
  uint8_t  craft;
  float    px, py, pz;
  float    qw, qx, qy, qz;
};

static NetPeer peers[NETPLAY_MAX_PEERS];
static portMUX_TYPE peersMux = portMUX_INITIALIZER_UNLOCKED;
static const uint8_t BROADCAST_MAC[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
static bool netReady = false;

static void onPacket(const uint8_t* mac, const uint8_t* data, int len) {
  if (len != (int)sizeof(PosePacket)) return;
  PosePacket pk;
  memcpy(&pk, data, sizeof(pk));
  if (pk.magic != NET_MAGIC) return;

  portENTER_CRITICAL(&peersMux);
  int slot = -1, freeSlot = -1, oldest = 0;
  for (int i = 0; i < NETPLAY_MAX_PEERS; i++) {
    if (peers[i].active && memcmp(peers[i].mac, mac, 6) == 0) { slot = i; break; }
    if (!peers[i].active && freeSlot < 0) freeSlot = i;
    if (peers[i].lastSeenMs < peers[oldest].lastSeenMs) oldest = i;
  }
  if (slot < 0) slot = freeSlot >= 0 ? freeSlot : oldest;
  NetPeer& p = peers[slot];
  p.active = true;
  memcpy(p.mac, mac, 6);
  p.lastSeenMs = millis();
  p.px = pk.px; p.py = pk.py; p.pz = pk.pz;
  p.qw = pk.qw; p.qx = pk.qx; p.qy = pk.qy; p.qz = pk.qz;
  p.gate = pk.gate; p.craft = pk.craft; p.mapId = pk.mapId;
  portEXIT_CRITICAL(&peersMux);
}

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) { onPacket(info->src_addr, data, len); }
#else
static void onRecv(const uint8_t* mac, const uint8_t* data, int len) { onPacket(mac, data, len); }
#endif

bool netplayBegin() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(NET_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST_MAC, 6);
  peer.channel = NET_CHANNEL;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) return false;
  netReady = true;
  return true;
}

void netplaySendPose(float px, float py, float pz, float qw, float qx, float qy, float qz,
                     uint8_t gate, uint8_t craft, uint16_t mapId) {
  if (!netReady) return;
  PosePacket pk;
  pk.magic = NET_MAGIC; pk.mapId = mapId; pk.gate = gate; pk.craft = craft;
  pk.px = px; pk.py = py; pk.pz = pz;
  pk.qw = qw; pk.qx = qx; pk.qy = qy; pk.qz = qz;
  esp_now_send(BROADCAST_MAC, (const uint8_t*)&pk, sizeof(pk));
}

int netplayGetPeers(NetPeer* out, int maxPeers) {
  const uint32_t now = millis();
  int n = 0;
  portENTER_CRITICAL(&peersMux);
  for (int i = 0; i < NETPLAY_MAX_PEERS && n < maxPeers; i++) {
    if (!peers[i].active) continue;
    if (now - peers[i].lastSeenMs > 2000) { peers[i].active = false; continue; }
    out[n++] = peers[i];
  }
  portEXIT_CRITICAL(&peersMux);
  return n;
}

uint16_t netplayMapId(const char* name) {
  uint16_t crc = 0xFFFF;
  for (; *name; name++) {
    crc ^= (uint8_t)*name << 8;
    for (int b = 0; b < 8; b++) crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
  }
  return crc;
}
