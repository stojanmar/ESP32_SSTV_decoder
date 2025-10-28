/*
  SSTV Martin 1 - stable line decoder with phase demodulation
*/
//Ver05 polperiodna tracking rutina za merjenje frekvence
//Ver06 preizkus final version z decode picture
//Veri2s6a kot 6a popravljena hitrost I2s na 46400 dela kar lepo in tudi sinc nove strani
//Veri2s6a2 popravim kontrast
//uvedem porch vstavke
#include <Arduino.h>
#include <driver/i2s.h>
#include <driver/adc.h>
#include <soc/syscon_reg.h>
#include "esp_adc_cal.h"
#include <TFT_eSPI.h>
#include <math.h>

#define I2S_PORT I2S_NUM_0
#define SAMPLE_RATE 48000

#define PIXELS_PER_LINE 320
#define LINES_MAX 256

#define SYNC_MS 4.862f
//#define PORCH_MS 0.572f
//#define COLOR_MS 138.24f
#define FREQ_LOW 1500.0f
#define FREQ_HIGH 2300.0f
#define COLOR_CONTRAST 1.1f //1.15f   // >1.0 increases contrast
#define COLOR_BRIGHTNESS -0.2f // -0.2 = darker, +0.2 = lighter

// --- Martin M1 parameters ---
const float FREQ_SYNC = 1200.0f;
const float FREQ_BLACK = 1500.0f;
const float FREQ_WHITE = 2300.0f;
const float PORCH_MS = 0.572f;
const float COLOR_MS = 138.24f;  // one color channel duration
const float LINE_MS = 486.2f;

TFT_eSPI tft;

// --- globals (define above setup) ---
struct ZeroCrossTracker {
  float thresh = 0.05f;       // hysteresis threshold (tune 0.03–0.1)
  int state = 0;              // -1 or +1
  int samplesSinceCross = 0;  // counter between crossings
  float freqHz = 1900.0f;     // last measured frequency
  float filteredHz = 1900.0f; // smoothed
};

ZeroCrossTracker zc;
float avgFreq = 1900.0f;
bool synced = false;
int lineY = 0;

/*float lineR[PIXELS_PER_LINE];
float lineG[PIXELS_PER_LINE];
float lineB[PIXELS_PER_LINE];*/

unsigned long lastDebug = 0;

int SAMPLES_PORCH = SAMPLE_RATE * (PORCH_MS / 1000.0f);  //  27
int SAMPLES_COLOR = SAMPLE_RATE * (COLOR_MS / 1000.0f);  //  6635
//int SAMPLES_PER_PIXEL = SAMPLES_COLOR / PIXELS_PER_LINE; //  20
int SAMPLES_PER_PIXEL = SAMPLE_RATE * (0.0004576f); // ≈ 22 samples @48kHz
// Buffers for current line
uint8_t lineR[PIXELS_PER_LINE];
uint8_t lineG[PIXELS_PER_LINE];
uint8_t lineB[PIXELS_PER_LINE];

float dcEstimate = 0, DC_ALPHA = 0.001f;

void setupI2S() {
  //adc1_config_width(ADC_WIDTH_BIT_12);
  //adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);

  i2s_config_t cfg = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    //.sample_rate = SAMPLE_RATE,
    .sample_rate = 46300, //46400,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    //.communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    //.intr_alloc_flags = 0,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    //.dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_driver_install(I2S_PORT, &cfg, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC1_CHANNEL_7);
  SET_PERI_REG_MASK(SYSCON_SARADC_CTRL2_REG, SYSCON_SARADC_SAR1_INV);
  i2s_adc_enable(I2S_PORT);
}

int readSamples(float *out, int n) {
  static int16_t raw[512];
  size_t bytes;
  int got = 0;
  while (got < n) {
    int todo = min(n - got, 512);
    if (i2s_read(I2S_PORT, raw, todo * 2, &bytes, portMAX_DELAY) != ESP_OK) break;
    int samples = bytes / 2;
    for (int i = 0; i < samples; i++) {
      //uint16_t v = ((uint16_t)raw[i] >> 4) & 0x0FFF;
      uint16_t v = ((uint16_t)raw[i]) & 0x0FFF; // 12 bitno 0 do 4095
      //Serial.println(v);
      int val = ((int)v) - 2047;
      //Serial.println(val);
      dcEstimate = (1 - DC_ALPHA) * dcEstimate + DC_ALPHA * val;
      //dcEstimate = 0.0f;
      //Serial.println(dcEstimate);
      out[got++] = (val - dcEstimate) / 2048.0f;  //normiranje na -1 do 1
    }
  }
  return got;
}

/*float mapFreqToColor(float freq) {
  float val = (freq - FREQ_BLACK) / (FREQ_WHITE - FREQ_BLACK);
  if (val < 0) val = 0;
  if (val > 1) val = 1;
  return val * 255.0f;
}*/
// With increased contrast and brightness
float mapFreqToColor(float freq) {
  float val = (freq - FREQ_BLACK) / (FREQ_WHITE - FREQ_BLACK);
  //val = (val - 0.5f) * COLOR_CONTRAST + 0.5f + COLOR_BRIGHTNESS;
  val = val * COLOR_CONTRAST + COLOR_BRIGHTNESS;
  if (val < 0) val = 0;
  if (val > 1) val = 1;
  return val * 255.0f;
}

float sampleAverageFreq(int sampleCount) {
  static float buf[512]; // temporary buffer (enough for short reads)
  if (sampleCount > 512) sampleCount = 512;

  readSamples(buf, sampleCount);

  float sum = 0;
  int count = 0;

  for (int i = 0; i < sampleCount; i++) {
    float freq = updateZeroCross(buf[i]);  // your improved function
    //if (freq > 100 && freq < 4000) {       // ignore noise
    if (freq > 200 && freq < 3800) {       // ignore noise
      sum += freq;
      count++;
    }
  }

  if (count == 0) return FREQ_BLACK;
  return sum / count;
}


void decodeLine(int lineNum) {
  // Green channel
  for (int x = 0; x < PIXELS_PER_LINE; x++) {
    float f = sampleAverageFreq(SAMPLES_PER_PIXEL);
    lineG[x] = (uint8_t)mapFreqToColor(f);
  }

  // Blue channel
  for (int x = 0; x < PIXELS_PER_LINE; x++) {
    float f = sampleAverageFreq(SAMPLES_PER_PIXEL);
    lineB[x] = (uint8_t)mapFreqToColor(f);
  }

  // Red channel
  for (int x = 0; x < PIXELS_PER_LINE; x++) {
    float f = sampleAverageFreq(SAMPLES_PER_PIXEL);
    lineR[x] = (uint8_t)mapFreqToColor(f);
  }

  // Draw the full RGB line
  for (int x = 0; x < PIXELS_PER_LINE; x++) {
    uint16_t color = tft.color565(lineR[x], lineG[x], lineB[x]);
    tft.drawPixel(x, lineNum, color);
  }

  // Progress indicator
  //tft.drawFastHLine(0, 0, map(lineNum, 0, LINES_MAX, 0, PIXELS_PER_LINE), TFT_RED);

  //Serial.printf("Drawn line %d\n", lineNum);
}
// --- helper: update zero-cross frequency ---
float updateZeroCross(float sample) {
  static int stableCount = 0;          // counts consecutive samples beyond threshold
  const int STABLE_MIN = 3;            // require 3 samples beyond threshold
  zc.samplesSinceCross++;

  // handle initialization
  if (zc.state == 0) {
    if (sample > zc.thresh)  zc.state = +1;
    else if (sample < -zc.thresh) zc.state = -1;
    return zc.filteredHz;   // <== return current filtered freq
  }

  // --- positive half ---
  if (zc.state > 0) {
    if (sample < -zc.thresh) {
      stableCount++;
      if (stableCount >= STABLE_MIN) {
        zc.freqHz = SAMPLE_RATE / (2.0f * zc.samplesSinceCross);
        zc.samplesSinceCross = 0;
        zc.state = -1;
        stableCount = 0;
      }
    } else stableCount = 0;
  }

  // --- negative half ---
  else if (zc.state < 0) {
    if (sample > zc.thresh) {
      stableCount++;
      if (stableCount >= STABLE_MIN) {
        zc.freqHz = SAMPLE_RATE / (2.0f * zc.samplesSinceCross);
        zc.samplesSinceCross = 0;
        zc.state = +1;
        stableCount = 0;
      }
    } else stableCount = 0;
  }

  // smooth result
  zc.filteredHz = 0.9f * zc.filteredHz + 0.1f * zc.freqHz;
  return zc.filteredHz;    // <== return smoothed frequency
}


float freqToColor(float f) {
  float norm = (f - FREQ_LOW) / (FREQ_HIGH - FREQ_LOW);
  norm = constrain(norm, 0, 1);
  return norm * 255;
}

//float buf[2048];
//uint8_t lineR[PIXELS_PER_LINE], lineG[PIXELS_PER_LINE], lineB[PIXELS_PER_LINE];

void setup() {
  Serial.begin(115200);
  tft.init();
  tft.invertDisplay(1); // invertiraj za tistega z dvema usb vhodoma
  tft.setRotation(1);
  tft.fillScreen(TFT_BLACK);
  setupI2S();
  Serial.println("Ready, waiting sync...");
}

//dve verziji za sstv s sinhonizacijo PRVA
void loop() {
  static bool synced = false;
  static int lineY = 0;
  static unsigned long lastSyncMicros = 0;
  static unsigned long lineStartMicros = 0;

  // --- Measure average frequency over small block ---
  //float f = sampleAverageFreq(128);  // smaller block for better timing
  float f = sampleAverageFreq(32);  // smaller block for better timing
  unsigned long now = micros();

  // --- Debug output every ~100 ms ---
  /*if (now - lastDebug > 100000) {
    Serial.printf("freq=%.1f Hz  synced=%d  line=%d\n", f, synced, lineY);
    lastDebug = now;
  }*/

  // --- Detect sync tone (around 1200 Hz) ---
  bool syncTone = (f > 1100 && f < 1300);

  if (!synced) {
    if (syncTone) {
      synced = true;
      lineY = 0;
      lastSyncMicros = now;
      lineStartMicros = now;
      tft.fillScreen(TFT_BLACK);
      Serial.println("SYNC detected → start frame");
    }
    return;
  }

  // --- When synced, check for next sync pulse ---
  if (syncTone && (now - lastSyncMicros) > 300000) {  // at least 300 ms between lines
    // re-align timing to this sync edge
    lastSyncMicros = now;
    lineStartMicros = now;
    //Serial.printf("Line %d sync @ %.1f Hz\n", lineY, f);

    // --- Decode next line ---
    decodeLine(lineY);
    lineY++;

    if (lineY >= LINES_MAX) {
      synced = false;
      Serial.println("Frame complete → waiting next sync");
    }
    return;
  }

  // --- Timeout (no sync for >500 ms) → lose lock ---
  if (synced && (now - lastSyncMicros) > 600000) {
    synced = false;
    Serial.println("Lost sync → waiting new frame");
  }
}  // end of loop








