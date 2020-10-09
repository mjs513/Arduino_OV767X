// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */

#include <Arduino.h>
#include <Wire.h>

#include "OV767X.h"

// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 32))
#endif

#ifndef portInputRegister
#define portInputRegister(P) ((P == 0) ? &NRF_P0->IN : &NRF_P1->IN)
#endif

extern "C" {
  // defined in utility/ov7670.c:
  struct ov7670_fract {
    uint32_t numerator;
    uint32_t denominator;
  };

  void* ov7670_alloc();
  void ov7670_free(void*);

  int ov7670_reset(void*, uint32_t val);
  int ov7670_detect(void*);
  void ov7670_configure(void*, int devtype, int format, int wsize, int clock_speed, int pll_bypass, int pclk_hb_disable);
  int ov7670_s_power(void*, int on);
  int ov7675_set_framerate(void*, struct ov7670_fract *tpf);

  int ov7670_s_sat_hue(void*, int sat, int hue);
  int ov7670_s_brightness(void*, int value);
  int ov7670_s_contrast(void*, int value);
  int ov7670_s_hflip(void*, int value);
  int ov7670_s_vflip(void*, int value);
  int ov7670_s_gain(void*, int value);
  int ov7670_s_autogain(void*, int value);
  int ov7670_s_exp(void*, int value);
  int ov7670_s_autoexp(void*, int value);
  int ov7670_s_test_pattern(void*, int value);
};

const int OV760_D[8] = {
  OV7670_D0, OV7670_D1, OV7670_D2, OV7670_D3, OV7670_D4, OV7670_D5, OV7670_D6, OV7670_D7
};

OV767X::OV767X() :
  _ov7670(NULL),
  _saturation(128),
  _hue(0)
{
  setPins(OV7670_VSYNC, OV7670_HREF, OV7670_PLK, OV7670_XCLK, OV760_D);
}

OV767X::~OV767X()
{
  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}

int OV767X::begin(int resolution, int format, int fps)
{
  Serial.println("OV767X::begin");
  switch (resolution) {
    case VGA:
      _width = 640;
      _height = 480;
      break;

    case CIF:
      _width = 352;
      _height = 240;
      break;

    case QVGA:
      _width = 320;
      _height = 240;
      break;

    case QCIF:
      _width = 176;
      _height = 144;
      break;

    case QQVGA:
      _width = 160;
      _height = 120;
      break;

    default:
      return 0;
  }

  _grayscale = false;
  switch (format) {
    case YUV422:
    case RGB444:
    case RGB565:
      _bytesPerPixel = 2;
      break;
      
    case GRAYSCALE:
      format = YUV422;    // We use YUV422 but discard U and V bytes
      _bytesPerPixel = 2; // 2 input bytes per pixel of which 1 is discarded
      _grayscale = true;
      break;      

    default:
      return 0;
  }

// The only frame rates which work on the Nano 33 BLE are 1 and 5 FPS
//  if (fps != 1 && fps != 5)
//    return 0;

  _ov7670 = ov7670_alloc();
  if (!_ov7670) {
    end();

    return 0;
  }

  pinMode(_vsyncPin, INPUT);
  pinMode(_hrefPin, INPUT);
  pinMode(_pclkPin, INPUT);
//  pinMode(_xclkPin, OUTPUT);
  Serial.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin, _xclkPin);

  for (int i = 0; i < 8; i++) {
    pinMode(_dPins[i], INPUT);
    Serial.printf("  _dpins(%d)=%d\n", i, _dPins[i]);
  }

  _vsyncPort = portInputRegister(digitalPinToPort(_vsyncPin));
  _vsyncMask = digitalPinToBitMask(_vsyncPin);
  _hrefPort = portInputRegister(digitalPinToPort(_hrefPin));
  _hrefMask = digitalPinToBitMask(_hrefPin);
  _pclkPort = portInputRegister(digitalPinToPort(_pclkPin));
  _pclkMask = digitalPinToBitMask(_pclkPin);

  beginXClk();

  Wire.begin();

  delay(1000);

  if (ov7670_detect(_ov7670)) {
    end();

    return 0;
  }

  ov7670_configure(_ov7670, 0 /*OV7670 = 0, OV7675 = 1*/, format, resolution, 16 /* MHz */, 
                    0 /*pll bypass*/, 1 /* pclk_hb_disable */);

  if (ov7670_s_power(_ov7670, 1)) {
    end();

    return 0;
  }

  struct ov7670_fract tpf;

  tpf.numerator = 1;
  tpf.denominator = fps;

  ov7675_set_framerate(_ov7670, &tpf);

  return 1;
}

void OV767X::end()
{
  endXClk();

  pinMode(_xclkPin, INPUT);

  Wire.end();

  if (_ov7670) {
    ov7670_free(_ov7670);
  }
}

int OV767X::width() const
{
  return _width;
}

int OV767X::height() const
{
  return _height;
}

int OV767X::bitsPerPixel() const
{
  return _bytesPerPixel * 8;
}

int OV767X::bytesPerPixel() const
{
  return _bytesPerPixel;
}

//
// Optimized Data Reading Explanation:
//
// In order to keep up with the data rate of 5 FPS, the inner loop that reads
// data from the camera board needs to be as quick as possible. The 64Mhz ARM
// Cortex-M4 in the Nano 33 would not be able to keep up if we read each bit
// one at a time from the various GPIO pins and combined them into a byte.
// Instead, we chose specific GPIO pins which all occupy a single GPIO "PORT"
// In this case, P1 (The Nano 33 exposes some bits of P0 and some of P1).
// The bits on P1 are not connected to sequential GPIO pins, so the order
// chosen may look a bit odd. Below is a map showing the GPIO pin numbers
// and the bit position they correspond with on P1 (bit 0 is on the right)
//
//    20-19-18-17-16-15-14-13-12-11-10-09-08-07-06-05-04-03-02-01-00  (bit)
// ~ +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
// ~ |xx|xx|xx|xx|xx|04|06|05|03|02|01|xx|12|xx|xx|xx|xx|00|10|11|xx| (pin)
// ~ +--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+--+
//
// The most efficient way to read 8-bits of data with the arrangement above
// is to wire the pins for P1 bits 2,3,10,11,12,13,14,15. This allows other
// features such as SPI to still work and gives us 2 groups of contiguous
// bits (0-1, 10-15). With 2 groups of bits, we can read, mask, shift and
// OR them together to form an 8-bit byte with the minimum number of operations.
//
void OV767X::readFrame(void* buffer)
{
//uint32_t ulPin = 33; // P1.xx set of GPIO is in 'pin' 32 and above
//NRF_GPIO_Type * port;

  //port = nrf_gpio_pin_port_decode(&ulPin);


  uint8_t* b = (uint8_t*)buffer;
  int bytesPerRow = _width * _bytesPerPixel;

  // Falling edge indicates start of frame
  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  digitalWriteFast(33, HIGH);
  for (int i = 0; i < _height; i++) {
  // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH

#if defined(__IMXRT1062__)  // Teensy 4.x
      uint32_t in = GPIO6_DR >> 18; // read all bits in parallel
      in = (in & 0x3) | ((in & 0x3f0)>>2);
      digitalToggleFast(32);
#else
      uint32_t in = port->IN; // read all bits in parallel
      in >>= 2; // place bits 0 and 1 at the "bottom" of the register
      in &= 0x3f03; // isolate the 8 bits we care about
      in |= (in >> 6); // combine the upper 6 and lower 2 bits
#endif

      if (!(j & 1) || !_grayscale) {
        *b++ = in;
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }
    digitalToggleFast(33);

    while ((*_hrefPort & _hrefMask) != 0); // wait for LOW
    interrupts();
  }
  digitalWriteFast(33, LOW);

}

void OV767X::testPattern(int pattern)
{
  ov7670_s_test_pattern(_ov7670, pattern);
}

void OV767X::noTestPattern()
{
  ov7670_s_test_pattern(_ov7670, 0);
}

void OV767X::setSaturation(int saturation)
{
  _saturation = saturation;

  ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

void OV767X::setHue(int hue)
{
  _hue = hue;

   ov7670_s_sat_hue(_ov7670, _saturation, _hue);
}

void OV767X::setBrightness(int brightness)
{
  ov7670_s_brightness(_ov7670, brightness);
}

void OV767X::setContrast(int contrast)
{
  ov7670_s_contrast(_ov7670, contrast);
}

void OV767X::horizontalFlip()
{
  ov7670_s_hflip(_ov7670, 1);
}

void OV767X::noHorizontalFlip()
{
  ov7670_s_hflip(_ov7670, 0);
}

void OV767X::verticalFlip()
{
  ov7670_s_vflip(_ov7670, 1);
}

void OV767X::noVerticalFlip()
{
  ov7670_s_vflip(_ov7670, 0);
}

void OV767X::setGain(int gain)
{
  ov7670_s_gain(_ov7670, gain);
}

void OV767X::autoGain()
{
  ov7670_s_autogain(_ov7670, 1);
}

void OV767X::setExposure(int exposure)
{
  ov7670_s_exp(_ov7670, exposure);
}

void OV767X::autoExposure()
{
  ov7670_s_autoexp(_ov7670, 0 /* V4L2_EXPOSURE_AUTO */);
}

void OV767X::setPins(int vsync, int href, int pclk, int xclk, const int dpins[8])
{
  _vsyncPin = vsync;
  _hrefPin = href;
  _pclkPin = pclk;
  _xclkPin = xclk;

  memcpy(_dPins, dpins, sizeof(_dPins));
}

void OV767X::beginXClk()
{
  // Generates 8 MHz signal using PWM... Will speed up.
  #if defined(__IMXRT1062__)  // Teensy 4.x
    analogWriteFrequency(_xclkPin, 16000000);
    analogWrite(_xclkPin, 127); delay(100); // 9mhz works, but try to reduce to debug timings with logic analyzer

  #else
  // Generates 16 MHz signal using I2S peripheral
  NRF_I2S->CONFIG.MCKEN = (I2S_CONFIG_MCKEN_MCKEN_ENABLE << I2S_CONFIG_MCKEN_MCKEN_Pos);
  NRF_I2S->CONFIG.MCKFREQ = I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV2  << I2S_CONFIG_MCKFREQ_MCKFREQ_Pos;
  NRF_I2S->CONFIG.MODE = I2S_CONFIG_MODE_MODE_MASTER << I2S_CONFIG_MODE_MODE_Pos;

  NRF_I2S->PSEL.MCK = (digitalPinToPinName(_xclkPin) << I2S_PSEL_MCK_PIN_Pos);

  NRF_I2S->ENABLE = 1;
  NRF_I2S->TASKS_START = 1;
#endif  
}

void OV767X::endXClk()
{
  #if defined(__IMXRT1062__)  // Teensy 4.x
  analogWrite(OV7670_XCLK, 0);
  #else
  NRF_I2S->TASKS_STOP = 1;
  #endif
}

//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure. 
DMAChannel OV767X::_dmachannel;
DMASetting OV767X::_dmasettings[2];
uint32_t OV767X::_dmaBuffer[DMABUFFER_SIZE];
extern "C" void xbar_connect(unsigned int input, unsigned int output); // in pwm.c

void dumpDMA_TCD(DMABaseClass *dmabc)
{
  Serial.printf("%x %x:", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf("SA:%x SO:%d AT:%x NB:%x SL:%d DA:%x DO: %d CI:%x DL:%x CS:%x BI:%x\n", (uint32_t)dmabc->TCD->SADDR,
    dmabc->TCD->SOFF, dmabc->TCD->ATTR, dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR, 
    dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA, dmabc->TCD->CSR, dmabc->TCD->BITER);
}

void xbar01_isr() {
  // Curious to see if this will signal or not...
  digitalToggleFast(33);
  XBARA1_CTRL0 |=  XBARA_CTRL_STS0;

}

void OV767X::readFrameDMA(void* buffer)
{
  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
  // OV7670_PLK   4
  *(portConfigRegister(_pclkPin)) = 3; // set to XBAR mode (xbar 8)

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT08, XBARA1_OUT_DMA_CH_MUX_REQ30);

  // Tell XBAR to dDMA on Rising 
  attachInterruptVector(IRQ_XBAR1_01, &xbar01_isr);
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0 | XBARA_CTRL_IEN0;

  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_8);  // Make sure it is input mode
  IOMUXC_XBAR1_IN08_SELECT_INPUT = 0; // Make sure this signal goes to this pin...

  // Need to switch the IO pins back to GPI1 from GPIO6
  IOMUXC_GPR_GPR26 &= ~(0x0FCC0000u); 


  // lets figure out how many bytes we will tranfer per setting...
  int bytesPerRow = _width * _bytesPerPixel;
  _rows_per_dma = DMABUFFER_SIZE / (bytesPerRow * 2);
  uint16_t bytes_per_dma = _rows_per_dma * bytesPerRow; // 

  // configure DMA channels
  //  _dmasettings[0].begin();
  _pixels_per_dma = bytes_per_dma / _bytesPerPixel;  // probably can hard code to 2...
  _rows_left_dma = _height;
  _frame_buffer_pointer = (uint16_t *)buffer;
  _dma_done = false;
  _dma_index = 0;

  _dmasettings[0].source(GPIO1_DR); // setup source.
  _dmasettings[0].destinationBuffer(_dmaBuffer, bytes_per_dma * 4);  // 32 bits per logical byte
  _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
  _dmasettings[0].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one

  _dmasettings[1].source(GPIO1_DR); // setup source.
  _dmasettings[1].destinationBuffer(&_dmaBuffer[bytes_per_dma], bytes_per_dma * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);


  // Falling edge indicates start of frame
  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  digitalWriteFast(32, HIGH);

  // We have the start of a frame, so lets start the dma.
  dumpDMA_TCD(&_dmachannel);
  dumpDMA_TCD(&_dmasettings[0]);
  dumpDMA_TCD(&_dmasettings[1]);
  Serial.printf("pclk pin: %d config:%x control:%x\n", _pclkPin, *(portConfigRegister(_pclkPin)), *(portControlRegister(_pclkPin)));
  Serial.printf("IOMUXC_GPR_GPR26:%x\n", IOMUXC_GPR_GPR26);
  Serial.printf("XBAR CTRL0:%x CTRL1:%x\n", XBARA1_CTRL0, XBARA1_CTRL1);


  _dmachannel.begin(true);
  _dmachannel.enable();
 
  // clear any previous status. 
  XBARA1_CTRL0 |=  XBARA_CTRL_STS0;

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  elapsedMillis em = 0;
  while ((em < 1000) && !_dma_done) ; // wait up to a second...
  if (!_dma_done) Serial.println("Dma did not complete");
  digitalWriteFast(32, LOW);

  dumpDMA_TCD(&_dmachannel);
  dumpDMA_TCD(&_dmasettings[0]);
  dumpDMA_TCD(&_dmasettings[1]);

}

void OV767X::dmaInterrupt() {
  Camera.processDMAInterrupt();  // lets get back to the main object...
}

void OV767X::processDMAInterrupt() {
  _dmachannel.clearInterrupt(); // tell system we processed it.
    digitalToggleFast(33);

  // lets guess which buffer completed.
  _dma_index++;
  uint32_t *buffer = (_dma_index & 1) ? _dmaBuffer : (uint32_t*)_dmasettings[1].TCD->DADDR;
  uint16_t pixels_per_dma = _pixels_per_dma;      
  // process the pixels...
  while (pixels_per_dma--) {
    uint8_t lsb = *buffer++ >> 18;  
    lsb = (lsb & 0x3) | ((lsb & 0x3f0)>>2);
    uint8_t msb = *buffer++ >> 18;  
    msb = (msb & 0x3) | ((msb & 0x3f0)>>2);
    *_frame_buffer_pointer = (uint16_t)(msb << 8) | lsb;
  }      

  // see if we are done or ... 
  if (!_rows_left_dma) {
      _dma_done = true;
  } else {
    _rows_left_dma -= _rows_per_dma;
    if (_rows_left_dma <= (2*_rows_per_dma)) _dmasettings[1].disableOnCompletion(); 
  }

}


OV767X Camera;
