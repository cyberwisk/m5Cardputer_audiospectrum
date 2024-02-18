/***
M5Cardputer Audio Spectrum Display, Oscilloscope, and Tuner
Portado por Aurélio Monteiro Avanzi

***/

/*
M5StickC Audio Spectrum Display, Oscilloscope, and Tuner

Copyright 2020 KIRA Ryouta

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

// M5StickC Audio Spectrum             : 2019.06.01 : macsbug
//  https://macsbug.wordpress.com/2019/06/01/
// Audio Spectrum Display with M5STACK : 2017.12.31 : macsbug
//  https://macsbug.wordpress.com/2017/12/31/audio-spectrum-display-with-m5stack/
// https://github.com/tobozo/ESP32-8-Octave-Audio-Spectrum-Display/tree/wrover-kit
// https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display
// https://github.com/kosme/arduinoFFT
 
 
// David Bird (https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display)
// tobozo (https://github.com/tobozo/ESP32-Audio-Spectrum-Waveform-Display)
// macsbug (https://macsbug.wordpress.com/)
// KIRA Ryouta (https://github.com/KKQ-KKQ/m5stickc-audiospectrum)


#pragma GCC optimize ("O3")
#include <M5Cardputer.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "dywapitchtrack.h"
#include "fix_fft.h"

#define PIN_CLK  43
#define PIN_DATA 46
#define SAMPLES 1024 // Must be a power of 2
#define READ_LEN (2 * SAMPLES)
#define TFT_WIDTH 240
#define TFT_HEIGHT 135
#define BANDS 8
#define BANDS_WIDTH ( TFT_WIDTH / BANDS )
#define BANDS_PADDING 8
#define BAR_WIDTH ( BANDS_WIDTH - BANDS_PADDING )
#define NOISE_FLOOR 1
#define AMPLIFIER (1 << 2)
#define MAGNIFY 3
#define RSHIFT 13
#define RSHIFT2 1
#define OSC_NOISEFLOOR 100
#define OSC_SAMPLES DYWAPT_SAMPLESIZE
#define OSC_SKIPCOUNT (OSC_SAMPLES/SAMPLES)
#define OSC_EXTRASKIP 0

//#define MAXBUFSIZE (2*SAMPLES)
#define MAXBUFSIZE OSC_SAMPLES

struct eqBand {
  const char *freqname;
  int peak;
  int lastpeak;
  uint16_t lastval;
};

enum : uint8_t {
  ModeSpectrumBars,
  ModeOscilloscope,
  ModeTuner,
  ModeCount,
};

static uint8_t runmode = 0;

static volatile bool semaphore = false;
static volatile bool needinit = true;

static eqBand audiospectrum[BANDS] = {
  // freqname,peak,lastpeak,lastval,
  { ".1k", 0 },
  { ".2k", 0 },
  { ".5k", 0 },
  { " 1k", 0 },
  { " 2k", 0 },
  { " 4k", 0 },
  { " 8k", 0 },
  { "16k", 0 }
};
 
static int vTemp[2][MAXBUFSIZE];
static uint8_t curbuf = 0;
static uint16_t colormap[TFT_HEIGHT];//color palette for the band meter(pre-fill in setup)

static dywapitchtracker pitchTracker;
static LGFX_Sprite sprite(&M5Cardputer.Display);

static int bufposcount = 0;

static const char *notestr[12] = {
  "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B "
};

static uint16_t oscbuf[2][OSC_SAMPLES];
#if OSC_EXTRASKIP > 0
static int skipcount = 0;
#endif

void i2sInit(){
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  DYWAPT_SAMPLERATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, //is fixed at 12bit,stereo,MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
   };
   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA; 
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, DYWAPT_SAMPLERATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
}
 
void setup() {
  M5.begin();
  setCpuFrequencyMhz(80);
  M5Cardputer.Display.setRotation(1);
  M5Cardputer.Display.fillScreen(BLACK);

  sprite.createSprite(TFT_WIDTH, TFT_HEIGHT);
  sprite.setTextSize(1);

  i2sInit();
  //M5Cardputer.Speaker.tone(440, 50);

  for(uint8_t i=0;i<TFT_HEIGHT;i++) {
    //colormap[i] = M5Cardputer.Display.color565(TFT_HEIGHT-i*.5,i*1.1,0); //RGB
    //colormap[i] = M5Cardputer.Display.color565(TFT_HEIGHT-i*4.4,i*2.5,0);//RGB:rev macsbug
    float r = TFT_HEIGHT - i;
    float g = i;
    float mag = (r > g)? 255./r : 255./g;
    r *= mag;
    g *= mag;
    colormap[i] = M5Cardputer.Display.color565((uint8_t)r,(uint8_t)g,0); // Modified by KKQ-KKQ
  }

  int core = 1 - xPortGetCoreID();
  xTaskCreatePinnedToCore(looptask,"calctask",32768,NULL,1,NULL,core);
}

void initMode() {
  M5Cardputer.Display.fillRect(0, 0,
                  TFT_WIDTH, TFT_HEIGHT, BLACK);
  switch (runmode) {
    case ModeSpectrumBars:
      M5Cardputer.Display.setTextSize(1);
      M5Cardputer.Display.setTextColor(LIGHTGREY);
      for (byte band = 0; band < BANDS; band++) {
        M5Cardputer.Display.setCursor(BANDS_WIDTH*band + 2, 0);
        M5Cardputer.Display.print(audiospectrum[band].freqname);
      }
      break;

    case ModeOscilloscope:
      {
        sprite.setTextColor(GREEN);
        dywapitch_inittracking(&pitchTracker);
      }
      break;

    case ModeTuner:
      sprite.setTextSize(1);
      break;
  }
}

void showSpectrumBars(){
  int *vTemp_ = vTemp[curbuf^1];

  Fixed15FFT::apply_window(vTemp_);
  Fixed15FFT::calc_fft(vTemp_, vTemp_ + SAMPLES);

  int values[BANDS] = {};
  for (int i = 2; i < (SAMPLES/2); i++){ 
    // Don't use sample 0 and only first SAMPLES/2 are usable. 
    // Each array element represents a frequency and its value the amplitude.
    int ampsq = vTemp_[i] * vTemp_[i] + vTemp_[i + SAMPLES] * vTemp_[i + SAMPLES];
    if (ampsq > NOISE_FLOOR) {
      byte bandNum = getBand(i);
      if(bandNum != 8 && ampsq > values[bandNum]) {
        values[bandNum] = ampsq;
      }
    }
  }
  for (byte band = 0; band < BANDS; band++) {
    int log2 = FIX_LOG2<15-RSHIFT>(values[band]);
    if (log2 > -AMPLIFIER) {
      displayBand(band, ((log2 + AMPLIFIER) * MAGNIFY) >> RSHIFT2);
    } else {
      displayBand(band, 0);
    }
  }
  //long vnow = millis();
  for (byte band = 0; band < BANDS; band++) {
    if (audiospectrum[band].peak > 0) {
      audiospectrum[band].peak -= 2;
    }
    if(audiospectrum[band].peak <= 0) {
      audiospectrum[band].peak = 0;
    }
    // only draw if peak changed
    if(audiospectrum[band].lastpeak != audiospectrum[band].peak) {
      // delete last peak
      uint16_t hpos = BANDS_WIDTH*band + (BANDS_PADDING/2);
      M5Cardputer.Display.drawFastHLine(hpos,TFT_HEIGHT-audiospectrum[band].lastpeak,BAR_WIDTH,BLACK);
      audiospectrum[band].lastpeak = audiospectrum[band].peak;
      uint16_t ypos = TFT_HEIGHT - audiospectrum[band].peak;
      M5Cardputer.Display.drawFastHLine(hpos, ypos,
                           BAR_WIDTH, colormap[ypos]);
    }
  } 
}

void displayBand(int band, int dsize){
  uint16_t hpos = BANDS_WIDTH*band + (BANDS_PADDING/2);
  if (dsize < 0) dsize = 0;
  if(dsize>TFT_HEIGHT-10) {
    dsize = TFT_HEIGHT-10; // leave some hspace for text
  }
  if(dsize < audiospectrum[band].lastval) {
    // lower value, delete some lines
    M5Cardputer.Display.fillRect(hpos, TFT_HEIGHT-audiospectrum[band].lastval,
                    BAR_WIDTH, audiospectrum[band].lastval - dsize,BLACK);
  }
  for (int s = 0; s <= dsize; s=s+4){
    uint16_t ypos = TFT_HEIGHT - s;
    M5Cardputer.Display.drawFastHLine(hpos, ypos, BAR_WIDTH, colormap[ypos]);
  }
  if (dsize > audiospectrum[band].peak){audiospectrum[band].peak = dsize;}
  audiospectrum[band].lastval = dsize;
}

byte getBand(int i) {
  if (i >= 2   && i < 4  ) return 0;  // 125Hz
  if (i >= 4   && i < 8  ) return 1;  // 250Hz
  if (i >= 8   && i < 16 ) return 2;  // 500Hz
  if (i >= 16  && i < 32 ) return 3;  // 1000Hz
  if (i >= 32  && i < 64 ) return 4;  // 2000Hz
  if (i >= 64  && i < 128) return 5;  // 4000Hz
  if (i >= 128 && i < 256) return 6;  // 8000Hz
  if (i >= 256 && i < 512) return 7;  // 16000Hz
  return 8;
}

float calcNumSamples(float f) {
  if (f == 0.0) return SAMPLES/2;
  float s = (float)(DYWAPT_SAMPLERATE * 2) / f;
  if (s > (float)OSC_SAMPLES) {
    do {
      s *= 0.5;
    } while (s > (float)OSC_SAMPLES);
  }
  return s;
}

void showFreq(float freq) {
  if (freq > 0) {
    char strbuf[16];
    sprintf(strbuf, "%8.2fHz", freq);
    sprite.drawString(strbuf, 0, 0, 1);
    float fnote = log2(freq)*12 - 36.376316562f;
    int note = fnote + 0.5f;
    if (note >= 0) {
      sprite.setCursor(TFT_WIDTH/2 - 4, 0);
      sprite.print(notestr[note % 12]);
      sprite.print(note / 12 - 1);
      float cent = (fnote - note) * 100;
      sprintf(strbuf, "%.1fcents", cent);
      sprite.drawRightString(strbuf, TFT_WIDTH, 0, 1);
    }
  }
}

void showOscilloscope()
{
  uint16_t i,j;
  uint16_t *oscbuf_ = oscbuf[curbuf^1];
  int *vTemp_ = vTemp[curbuf ^ 1];
  float freq = dywapitch_computepitch(&pitchTracker, vTemp_);
#if OSC_EXTRASKIP > 0
  if (skipcount < OSC_EXTRASKIP) {
    ++skipcount;
    return;
  }
  skipcount = 0;
#endif
  uint16_t s = calcNumSamples(freq);
  float mx = (float)TFT_WIDTH / s;
  float my;
  uint16_t maxV = 0;
  uint16_t minV = 65535;
  uint16_t offset = 0;
  for (i = 0; i < s; ++i) {
    if (maxV < oscbuf_[i]) maxV = oscbuf_[i];
    if (minV > oscbuf_[i]) {
      minV = oscbuf_[i];
      if (i + s <= OSC_SAMPLES) offset = i;
    }
  }
  if (maxV - minV > OSC_NOISEFLOOR) {
    my = (float)(TFT_HEIGHT-10) / (maxV - minV);
  }
  else {
    my = (float)(TFT_HEIGHT-10) / OSC_NOISEFLOOR;
    minV = (((int)maxV + (int)minV) >> 1) - OSC_NOISEFLOOR/2;
  }
  sprite.fillSprite(BLACK);
  uint16_t y = TFT_HEIGHT - (oscbuf_[offset] - minV) * my;
  for (i = 1; i < s; ++i) {
    uint16_t y2 = TFT_HEIGHT - (oscbuf_[offset + i] - minV) * my;
    sprite.drawLine((uint16_t)((i-1) * mx), y,
                    (uint16_t)(i * mx), y2, LIGHTGREY);
    y = y2;
  }
  showFreq(freq);
  sprite.pushSprite(0,0);
}

void showTuner() {
  int *vTemp_ = vTemp[curbuf ^ 1];
  float freq = dywapitch_computepitch(&pitchTracker, vTemp_);
  float fnote;
  int note;
  if (freq > 0) {
    fnote = log2(freq)*12 - 36.376316562;
    note = fnote + 0.5;
  }
  else {
    note = -1;
  }
  uint32_t bgcolor, fgcolor;
  if (note >= 0) {
    float cent = (fnote - note) * 100;
    if (abs(cent) < 2.) {
      bgcolor = GREEN;
      fgcolor = BLACK;
    }
    else {
      bgcolor = DARKGREY;
      fgcolor = BLACK;
    }
    sprite.fillSprite(bgcolor);
    sprite.fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, bgcolor);
    sprite.setTextColor(fgcolor);
    sprite.drawRect(2, 36, TFT_WIDTH-3, TFT_HEIGHT-40, fgcolor);
    sprite.drawRect(TFT_WIDTH/2 + 1, 36, 1, TFT_HEIGHT - 40, fgcolor);
    sprite.fillCircle(((float)TFT_WIDTH/2 + 1) + cent * ((float)(TFT_WIDTH-3)/100), (TFT_HEIGHT+34)/2, 5, fgcolor);
    char strbuf[8];
    sprintf(strbuf, "%s%d", notestr[note % 12], note / 12 - 1);
    sprite.drawCentreString(strbuf, TFT_WIDTH/2, 3, 4);
  }
  else {
    sprite.fillSprite(DARKGREY);
    sprite.drawRect(2, 36, TFT_WIDTH-3, TFT_HEIGHT-40, BLACK);
    sprite.drawLine(TFT_WIDTH/2 + 1, 36, TFT_WIDTH/2 + 1, TFT_HEIGHT - 4, BLACK);
  }
  sprite.pushSprite(0,0);
}

void looptask(void *) {
  while (1) {
    if (needinit) {
      initMode();
      needinit = false;
    }
    if (semaphore) {
      switch(runmode) {
        case ModeSpectrumBars:
          showSpectrumBars();
          break;

        case ModeOscilloscope:
          showOscilloscope();
          break;

        case ModeTuner:
          showTuner();
          break;
      }
      semaphore = false;
    }
    else {
      vTaskDelay(10);
    }
  }
}

void loop() {
   M5Cardputer.update();
  if (!M5Cardputer.BtnA.isPressed()) {
    ++runmode;
    if (runmode >= ModeCount) runmode = 0;

    bufposcount = 0;
    needinit = true;;
  }
  uint16_t i,j;
  j = bufposcount * SAMPLES;
  uint16_t *adcBuffer = &oscbuf[curbuf][j];
  size_t bytesread;
  i2s_read(I2S_NUM_0,(char*)adcBuffer,READ_LEN,&bytesread,portMAX_DELAY);
  int32_t dc = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    dc += adcBuffer[i];
  }
  dc /= SAMPLES;

  switch(runmode) {
    case ModeSpectrumBars:
      for (int i = 0; i < SAMPLES; ++i) {
        
        vTemp[curbuf][i] = (int)adcBuffer[i] - dc;
        vTemp[curbuf][i + SAMPLES] = 0;
      }
      curbuf ^= 1;
      semaphore = true;
      break;
    case ModeOscilloscope:
      for (i = 0; i < SAMPLES; ++i) {
        vTemp[curbuf][i + j] = (int)adcBuffer[i] - dc;
      }
      if (++bufposcount >= OSC_SKIPCOUNT) {
        bufposcount = 0;
        curbuf ^= 1;
        semaphore = true;
      }
      break;
    case ModeTuner:
      j = bufposcount * SAMPLES;
      for (i = 0; i < SAMPLES; ++i) {
        vTemp[curbuf][i + j] = (int)adcBuffer[i] - dc;
      }
      if (++bufposcount >= OSC_SKIPCOUNT) {
        bufposcount = 0;
        curbuf ^= 1;
        semaphore = true;
      }
      break;
  }
}
