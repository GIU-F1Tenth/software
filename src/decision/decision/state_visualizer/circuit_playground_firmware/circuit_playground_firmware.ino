#include <Adafruit_CircuitPlayground.h>

static const uint32_t SERIAL_BAUD = 115200;
static const uint8_t  NUM_PIXELS  = 10;
static const uint8_t  BRIGHTNESS  = 40;

static const size_t BUF_SIZE = 32;
static char    buf[BUF_SIZE];
static size_t  bufLen = 0;
static bool    inFrame = false;

static void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    CircuitPlayground.setPixelColor(i, r, g, b);
  }
}

static void handleFrame(const char *frame) {
  int r = 0, g = 0, b = 0;
  if (sscanf(frame, "%d,%d,%d", &r, &g, &b) != 3) {
    return;
  }
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  setAllPixels((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

void setup() {
  CircuitPlayground.begin();
  CircuitPlayground.setBrightness(BRIGHTNESS);
  setAllPixels(0, 0, 0);
  Serial.begin(SERIAL_BAUD);
}

void loop() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();

    if (c == '<') {
      inFrame = true;
      bufLen = 0;
      continue;
    }

    if (!inFrame) {
      continue;
    }

    if (c == '>') {
      buf[bufLen] = '\0';
      handleFrame(buf);
      inFrame = false;
      bufLen = 0;
      continue;
    }

    if (bufLen < BUF_SIZE - 1) {
      buf[bufLen++] = c;
    } else {
      inFrame = false;
      bufLen = 0;
    }
  }
}
