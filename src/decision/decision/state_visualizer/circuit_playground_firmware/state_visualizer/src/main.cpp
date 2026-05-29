#include <Adafruit_CircuitPlayground.h>

static const uint32_t SERIAL_BAUD = 115200;
static const uint8_t  NUM_PIXELS  = 10;
static const uint8_t  BRIGHTNESS  = 40;

static const size_t BUF_SIZE = 32;
static char    buf[BUF_SIZE];
static size_t  bufLen = 0;
static bool    inFrame = false;
static char    frameClose = 0;

static void setAllPixels(uint8_t r, uint8_t g, uint8_t b) {
  for (uint8_t i = 0; i < NUM_PIXELS; i++) {
    CircuitPlayground.setPixelColor(i, r, g, b);
  }
}

static void handleColorFrame(const char *frame) {
  int r = 0, g = 0, b = 0;
  if (sscanf(frame, "%d,%d,%d", &r, &g, &b) != 3) {
    return;
  }
  r = constrain(r, 0, 255);
  g = constrain(g, 0, 255);
  b = constrain(b, 0, 255);
  setAllPixels((uint8_t)r, (uint8_t)g, (uint8_t)b);
}

static void handleBeepFrame(const char *frame) {
  int freq = 0, dur = 0;
  if (sscanf(frame, "%d,%d", &freq, &dur) != 2) {
    return;
  }
  freq = constrain(freq, 0, 20000);
  dur  = constrain(dur, 0, 5000);
  if (freq > 0 && dur > 0) {
    // wait=false -> non-blocking, so the serial loop keeps running.
    CircuitPlayground.playTone((uint16_t)freq, (uint16_t)dur, false);
  }
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

    if (c == '<' || c == '[') {
      inFrame = true;
      frameClose = (c == '<') ? '>' : ']';
      bufLen = 0;
      continue;
    }

    if (!inFrame) {
      continue;
    }

    if (c == frameClose) {
      buf[bufLen] = '\0';
      if (frameClose == '>') {
        handleColorFrame(buf);
      } else {
        handleBeepFrame(buf);
      }
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
