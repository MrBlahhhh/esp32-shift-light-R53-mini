#include "appverify.h"
#include "appkey.h"
#include "proto.h"
#include <Arduino.h>
#include <esp_random.h>
// This header has no extern "C" guard of its own, unlike md.h. Without the
// wrapper the call links against a C++-mangled name the C library never exports.
extern "C" {
#include <mbedtls/constant_time.h>
}
#include <mbedtls/md.h>
#include <string.h>

// 3 in the core's prebuilt sdkconfig. The board only takes one phone at a time
// (advertising stops on connect), so every slot but one is headroom.
#ifndef CONFIG_BT_NIMBLE_MAX_CONNECTIONS
#define CONFIG_BT_NIMBLE_MAX_CONNECTIONS 3
#endif

struct Link {
  bool     open;
  uint16_t connHandle;
  uint8_t  challenge[SL_AUTH_CHALLENGE_LEN];
  uint8_t  wrongResponses;
  bool     verified;
};

static Link s_links[CONFIG_BT_NIMBLE_MAX_CONNECTIONS];

static Link* findLink(uint16_t connHandle) {
  for (Link& link : s_links) {
    if (link.open && link.connHandle == connHandle) return &link;
  }
  return nullptr;
}

static Link* freeLink() {
  for (Link& link : s_links) {
    if (!link.open) return &link;
  }
  return nullptr;
}

static bool expectedResponse(const uint8_t* challenge, uint8_t* response) {
  uint8_t message[SL_AUTH_PREFIX_LEN + SL_AUTH_CHALLENGE_LEN];
  memcpy(message, SL_AUTH_PREFIX, SL_AUTH_PREFIX_LEN);
  memcpy(message + SL_AUTH_PREFIX_LEN, challenge, SL_AUTH_CHALLENGE_LEN);

  uint8_t mac[32];
  int rc = mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                           SHIFTLIGHT_APP_KEY, sizeof(SHIFTLIGHT_APP_KEY),
                           message, sizeof(message), mac);
  if (rc != 0) {
    Serial.printf("BLE: HMAC failed (%d), app cannot be verified\n", rc);
    return false;
  }
  memcpy(response, mac, SL_AUTH_RESPONSE_LEN);
  return true;
}

void appVerifyOnConnect(uint16_t connHandle) {
  // Reuse the slot if NimBLE hands out a handle whose disconnect we never saw.
  Link* link = findLink(connHandle);
  if (!link) link = freeLink();
  if (!link) {
    Serial.println("BLE: no slot to verify this connection; it can watch but not change settings");
    return;
  }
  memset(link, 0, sizeof(*link));
  link->open       = true;
  link->connHandle = connHandle;
  // The radio is on, so this is the hardware RNG rather than a pseudo-random fill.
  esp_fill_random(link->challenge, sizeof(link->challenge));
}

void appVerifyOnDisconnect(uint16_t connHandle) {
  Link* link = findLink(connHandle);
  if (link) memset(link, 0, sizeof(*link));
}

size_t appVerifyReadValue(uint16_t connHandle, uint8_t* out) {
  const Link* link = findLink(connHandle);
  if (!link) return 0;
  if (link->verified) {
    out[0] = SL_AUTH_VERIFIED;
    return 1;
  }
  memcpy(out, link->challenge, SL_AUTH_CHALLENGE_LEN);
  return SL_AUTH_CHALLENGE_LEN;
}

void appVerifyOnResponse(uint16_t connHandle, const uint8_t* response, size_t len) {
  Link* link = findLink(connHandle);
  if (!link || link->verified) return;
  // Already logged when the limit was reached.
  if (link->wrongResponses >= SL_AUTH_MAX_ATTEMPTS) return;

  uint8_t expected[SL_AUTH_RESPONSE_LEN];
  bool right = len == SL_AUTH_RESPONSE_LEN &&
               expectedResponse(link->challenge, expected) &&
               mbedtls_ct_memcmp(response, expected, SL_AUTH_RESPONSE_LEN) == 0;
  if (right) {
    link->verified = true;
    Serial.println("BLE: app verified, settings open to this connection");
    return;
  }

  link->wrongResponses++;
  if (link->wrongResponses < SL_AUTH_MAX_ATTEMPTS) {
    Serial.printf("BLE: app verification failed (%u of %u tries)\n",
                  (unsigned)link->wrongResponses, (unsigned)SL_AUTH_MAX_ATTEMPTS);
  } else {
    Serial.printf("BLE: app verification failed %u times; ignoring this connection's responses\n",
                  (unsigned)SL_AUTH_MAX_ATTEMPTS);
  }
}

bool appVerified(uint16_t connHandle) {
  const Link* link = findLink(connHandle);
  return link != nullptr && link->verified;
}
