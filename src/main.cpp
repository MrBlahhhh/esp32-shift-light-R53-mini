#include <esp_now.h>
#include <WiFi.h>
#include <FastLED.h>

#define LED_PIN 13        // Pin connected to WS2812B data line
#define NUM_LEDS 8        // Number of LEDs in the strip
#define LED_TYPE WS2812B
#define COLOR_ORDER GRB
#define SIMULATE_RPM      // Comment out to disable RPM simulation and use ESP-NOW

CRGB leds[NUM_LEDS];
uint32_t rpm = 0;         // Current RPM value
bool redBlinkState = false;
unsigned long lastBlinkTime = 0;
const unsigned long blinkInterval = 100; // 100ms for 5Hz blink (on/off) at 7100+ RPM
unsigned long lastSimTime = 0;
const unsigned long simInterval = 100; // Update simulation every 100ms
const unsigned long simPeriod = 10000; // 10-second cycle for RPM simulation

// Function prototypes
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len);
void updateLEDs();
void simulateRPM();

void setup() {
  Serial.begin(115200);
  
  // Initialize FastLED
  FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(75); // Brightness set to 75 (0-255)

  #ifndef SIMULATE_RPM
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Register callback for received data
  esp_now_register_recv_cb(OnDataRecv);
  #endif
}

void loop() {
  #ifdef SIMULATE_RPM
  simulateRPM();
  #endif
  updateLEDs();
  FastLED.show();
}

// ESP-NOW callback function
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
  #ifndef SIMULATE_RPM
  if (len == sizeof(uint32_t)) {
    memcpy(&rpm, incomingData, sizeof(rpm));
    Serial.print("Received RPM: ");
    Serial.println(rpm);
  }
  #endif
}

void updateLEDs() {
  CRGB color;
  
  // Calculate number of LED pairs to light (0 to 4 pairs)
  int numPairs = constrain(map(rpm, 0, 7100, 0, 4), 0, 4);
  
  // Determine color based on RPM
  if (rpm < 3000) {
    color = CRGB(0, 0, 0); // Off below 3000 RPM
  } else if (rpm < 6000) {
    // Solid green from 3000 to 6000 RPM
    color = CRGB(0, 255, 0);
  } else if (rpm <= 7100) {
    // Fade from green to red (6000 to 7100 RPM)
    uint8_t t = map(rpm, 6000, 7100, 0, 255);
    uint8_t red = t;
    uint8_t green = 255 - t; // Scale green down (255 to 0)
    color = CRGB(red, green, 0);
  } else {
    // Blink red at 7100+ RPM
    if (millis() - lastBlinkTime >= blinkInterval) {
      redBlinkState = !redBlinkState;
      lastBlinkTime = millis();
    }
    color = redBlinkState ? CRGB(255, 0, 0) : CRGB(0, 0, 0);
    numPairs = 4; // All LEDs blink at 7100+ RPM
  }

  // Set LEDs from ends to center based on numPairs
  for (int i = 0; i < NUM_LEDS; i++) {
    leds[i] = CRGB(0, 0, 0); // Default to off
    if (numPairs >= 1 && (i == 0 || i == 7)) leds[i] = color; // Pair 1: Ends
    if (numPairs >= 2 && (i == 1 || i == 6)) leds[i] = color; // Pair 2
    if (numPairs >= 3 && (i == 2 || i == 5)) leds[i] = color; // Pair 3
    if (numPairs >= 4 && (i == 3 || i == 4)) leds[i] = color; // Pair 4: Center
  }
}

void simulateRPM() {
  unsigned long currentTime = millis();
  if (currentTime - lastSimTime >= simInterval) {
    // Calculate time within 10-second cycle
    unsigned long cycleTime = currentTime % simPeriod;
    if (cycleTime < simPeriod / 2) {
      // Ramp up from 1000 to 8000 RPM (0 to 5000 ms)
      rpm = 1000 + ((cycleTime * 7000) / (simPeriod / 2));
    } else {
      // Ramp down from 8000 to 1000 RPM (5000 to 10000 ms)
      rpm = 8000 - (((cycleTime - simPeriod / 2) * 7000) / (simPeriod / 2));
    }
    Serial.print("Simulated RPM: ");
    Serial.println(rpm);
    lastSimTime = currentTime;
  }
}