// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */
#define DEBUG_CAMERA

#include <Arduino.h>
#include <Wire.h>

#include "OV767X.h"
#include "arm_math.h"
#define  DEBUG_FLEXIO
// if not defined in the variant
#ifndef digitalPinToBitMask
#define digitalPinToBitMask(P) (1 << (digitalPinToPinName(P) % 64))
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
  _hue(0),
  _frame_buffer_pointer(NULL)
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
    
  // BUGBUG::: see where frame is
  pinMode(49, OUTPUT);
    _hw_config = HM01B0_TEENSY_MICROMOD_FLEXIO_8BIT;
    
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

  _ov7670 = ov7670_alloc();
  if (!_ov7670) {
    end();

    return 0;
  }

  pinMode(_vsyncPin, INPUT);
  pinMode(_hrefPin, INPUT);
  pinMode(_pclkPin, INPUT);
//  pinMode(_xclkPin, OUTPUT);
#ifdef DEBUG_CAMERA
  Serial.printf("  VS=%d, HR=%d, PC=%d XC=%d\n", _vsyncPin, _hrefPin, _pclkPin, _xclkPin);
#endif

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
    Serial.println("Camera detect failed");

    return 0;
  }

  Serial.printf("Calling ov7670_configure\n");
  ov7670_configure(_ov7670, 0 /*OV7670 = 0, OV7675 = 1*/, format, resolution, 16 /* MHz */,
                   0 /*pll bypass*/, 1 /* pclk_hb_disable */);

  if (ov7670_s_power(_ov7670, 1)) {
    end();
    Serial.println("Camera ov7670_s_power failed");

    return 0;
  }

  struct ov7670_fract tpf;

  tpf.numerator = 1;
  tpf.denominator = fps;
  
//flexIO/DMA
	flexio_configure();
	setVSyncISRPriority(102);
	setDMACompleteISRPriority(192);

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


#define FLEXIO_USE_DMA
void OV767X::readFrame(void* buffer){
	readFrameFlexIO(buffer);
	
}


bool OV767X::readContinuous(bool(*callback)(void *frame_buffer), void *fb1, void *fb2) {
	//set_mode(HIMAX_MODE_STREAMING_NFRAMES, 1);

	return startReadFlexIO(callback, fb1, fb2);

}

void OV767X::stopReadContinuous() {
	
  stopReadFlexIO();

}

void OV767X::readFrameGPIO(void* buffer)
{

  uint8_t* b = (uint8_t*)buffer;
//  bool _grayscale;  // ????  member variable ?????????????
  int bytesPerRow = _width * _bytesPerPixel;

  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint32_t in =  (GPIO7_PSR >> 4); // read all bits in parallel
	  //uint32_t in = mmBus;

      if (!(j & 1) || !_grayscale) {
        *b++ = in;
      }
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0) ;  // wait for LOW
    interrupts();
  }

}


void OV767X::readFrame4BitGPIO(void* buffer)
{

  uint8_t* b = (uint8_t*)buffer;
//  bool _grayscale;  // ????  member variable ?????????????
  uint8_t in0 = 0;
  
  int bytesPerRow = _width * _bytesPerPixel;


  // Falling edge indicates start of frame
  //pinMode(PCLK_PIN, INPUT); // make sure back to input pin...
  // lets add our own glitch filter.  Say it must be hig for at least 100us
  elapsedMicros emHigh;
  do {
    while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
    emHigh = 0;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
  } while (emHigh < 2);

  for (int i = 0; i < _height; i++) {
    // rising edge indicates start of line
    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW
    noInterrupts();

    for (int j = 0; j < bytesPerRow; j++) {
      // rising edges clock each data byte
      while ((*_pclkPort & _pclkMask) == 0); // wait for HIGH

      //uint32_t in = ((_frame_buffer_pointer)? GPIO1_DR : GPIO6_DR) >> 18; // read all bits in parallel
      uint8_t in =  (GPIO7_PSR >> 4); // read all bits in parallel
	  //uint32_t in = mmBus; 
	  in &= 0x0F;
	  
	  if((j + 1) % 2) {
		  in = (in0 << 4) | (in);
		  if (!(j & 1) || !_grayscale) {
			*b++ = in;
		  }
	  } else {
		  in0 = in;
	  }
	  
      while (((*_pclkPort & _pclkMask) != 0) && ((*_hrefPort & _hrefMask) != 0)) ; // wait for LOW bail if _href is lost
    }

    while ((*_hrefPort & _hrefMask) != 0) ;  // wait for LOW
    interrupts();
  }

}


bool OV767X::flexio_configure()
{
    #define CNT_SHIFTERS 8

    // Going to try this using my FlexIO library.

    // BUGBUG - starting off not going to worry about maybe first pin I choos is on multipl Flex IO controllers (yet)
    uint8_t tpclk_pin; 
    _pflex = FlexIOHandler::mapIOPinToFlexIOHandler(_pclkPin, tpclk_pin);
    if (!_pflex) {
        Serial.printf("HM01B0 PCLK(%u) is not a valid Flex IO pin\n", _pclkPin);
        return false;
    }
    _pflexio = &(_pflex->port());

    // Quick and dirty:
    uint8_t thsync_pin = _pflex->mapIOPinToFlexPin(_hrefPin);
    uint8_t tg0 = _pflex->mapIOPinToFlexPin(_dPins[0]);
    uint8_t tg1 = _pflex->mapIOPinToFlexPin(_dPins[1]);
    uint8_t tg2 = _pflex->mapIOPinToFlexPin(_dPins[2]);
    uint8_t tg3 = _pflex->mapIOPinToFlexPin(_dPins[3]);

    // make sure the minimum here is valid: 
    if ((thsync_pin == 0xff) || (tg0 == 0xff) || (tg1 == 0xff) || (tg2 == 0xff) || (tg3 == 0xff)) {
        Serial.printf("HM01B0 Some pins did not map to valid Flex IO pin\n");
        Serial.printf("    HSYNC(%u %u) G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _hrefPin, thsync_pin, _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    } 
    // Verify that the G numbers are consecutive... Should use arrays!
    if ((tg1 != (tg0+1)) || (tg2 != (tg0+2)) || (tg3 != (tg0+3))) {
        Serial.printf("HM01B0 Flex IO pins G0-G3 are not consective\n");
        Serial.printf("    G0(%u %u) G1(%u %u) G2(%u %u) G3(%u %u)", 
            _dPins[0], tg0, _dPins[1], tg1, _dPins[2], tg2, _dPins[3], tg3 );
        return false;
    }
    if (_dPins[4] != 0xff) {
        uint8_t tg4 = _pflex->mapIOPinToFlexPin(_dPins[4]);
        uint8_t tg5 = _pflex->mapIOPinToFlexPin(_dPins[5]);
        uint8_t tg6 = _pflex->mapIOPinToFlexPin(_dPins[6]);
        uint8_t tg7 = _pflex->mapIOPinToFlexPin(_dPins[7]);
        if ((tg4 != (tg0+4)) || (tg5 != (tg0+5)) || (tg6 != (tg0+6)) || (tg7 != (tg0+7))) {
            Serial.printf("HM01B0 Flex IO pins G4-G7 are not consective with G0-3\n");
            Serial.printf("    G0(%u %u) G4(%u %u) G5(%u %u) G6(%u %u) G7(%u %u)", 
                _dPins[0], tg0, _dPins[4], tg4, _dPins[5], tg5, _dPins[6], tg6, _dPins[7], tg7 );
            return false;
        }
        _hw_config = HM01B0_TEENSY_MICROMOD_FLEXIO_8BIT;
        Serial.println("Custom - Flexio is 8 bit mode");
    } else {
      // only 8 bit mode supported
      // _hw_config = HM01B0_TEENSY_MICROMOD_FLEXIO_4BIT;
      Serial.println("Custom - Flexio 4 bit mode not supported");
      return false;
    }

#if (CNT_SHIFTERS == 4)
    // lets try to claim for shifters 0-3 or 4-7
    // Needs Shifter 3 (maybe 7 would work as well?)
    for (_fshifter = 0; _fshifter < 4; _fshifter++) {
      if (!_pflex->claimShifter(_fshifter)) break;
    }

    if (_fshifter < 4) {
      // failed on 0-3 - released any we claimed
      Serial.printf("Failed to claim 0-3(%u) shifters trying 4-7\n", _fshifter);
      while (_fshifter > 0) _pflex->freeShifter(--_fshifter);  // release any we grabbed

      for (_fshifter = 4; _fshifter < 8; _fshifter++) {
        if (!_pflex->claimShifter(_fshifter)) {
          Serial.printf("HM01B0 Flex IO: Could not claim Shifter %u\n", _fshifter);
          while (_fshifter > 4) _pflex->freeShifter(--_fshifter);  // release any we grabbed
          return false;
        }
      }
      _fshifter = 4;
    } else {
      _fshifter = 0;
    }


    // ?????????? dma source... 
    _fshifter_mask = 1 << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 
#else
    // all 8 shifters.
    for (_fshifter = 0; _fshifter < 8; _fshifter++) {
      if (!_pflex->claimShifter(_fshifter)) {
        Serial.printf("HM01B0 Flex IO: Could not claim Shifter %u\n", _fshifter);
        while (_fshifter > 4) _pflex->freeShifter(--_fshifter);  // release any we grabbed
        return false;
      }
    }
    _fshifter = 0;
    _fshifter_mask = 1 /*0xff */; // 8 channels << _fshifter;   // 4 channels.
    _dma_source = _pflex->shiftersDMAChannel(_fshifter); // looks like they use 
#endif    
    
    // Now request one timer
    uint8_t _ftimer = _pflex->requestTimers(); // request 1 timer. 
    if (_ftimer == 0xff) {
        Serial.printf("HM01B0 Flex IO: failed to request timer\n");
        return false;
    }

    _pflex->setIOPinToFlexMode(_hrefPin);
    _pflex->setIOPinToFlexMode(_pclkPin);
    _pflex->setIOPinToFlexMode(_dPins[0]);
    _pflex->setIOPinToFlexMode(_dPins[1]);
    _pflex->setIOPinToFlexMode(_dPins[2]);
    _pflex->setIOPinToFlexMode(_dPins[3]);
    _pflex->setIOPinToFlexMode(_dPins[4]);
    _pflex->setIOPinToFlexMode(_dPins[5]);
    _pflex->setIOPinToFlexMode(_dPins[6]);
    _pflex->setIOPinToFlexMode(_dPins[7]);



    // We already configured the clock to allow access.
    // Now sure yet aoub configuring the actual colock speed...

/*
    CCM_CSCMR2 |= CCM_CSCMR2__pflex->CLK_SEL(3); // 480 MHz from USB PLL

    CCM_CS1CDR = (CCM_CS1CDR
        & ~(CCM_CS1CDR__pflex->CLK_PRED(7) | CCM_CS1CDR__pflex->CLK_PODF(7)))
        | CCM_CS1CDR__pflex->CLK_PRED(1) | CCM_CS1CDR__pflex->CLK_PODF(1);


    CCM_CCGR3 |= CCM_CCGR3_FLEXIO2(CCM_CCGR_ON);
*/    
    // clksel(0-3PLL4, Pll3 PFD2 PLL5, *PLL3_sw)
    // clk_pred(0, 1, 2, 7) - divide (n+1)
    // clk_podf(0, *7) divide (n+1)
    // So default is 480mhz/16
    // Clock select, pred, podf:
    _pflex->setClockSettings(3, 1, 1);


#ifdef DEBUG_FLEXIO
    Serial.println("FlexIO Configure");
    Serial.printf(" CCM_CSCMR2 = %08X\n", CCM_CSCMR2);
    uint32_t div1 = ((CCM_CS1CDR >> 9) & 7) + 1;
    uint32_t div2 = ((CCM_CS1CDR >> 25) & 7) + 1;
    Serial.printf(" div1 = %u, div2 = %u\n", div1, div2);
    Serial.printf(" FlexIO Frequency = %.2f MHz\n", 480.0 / (float)div1 / (float)div2);
    Serial.printf(" CCM_CCGR3 = %08X\n", CCM_CCGR3);
    Serial.printf(" FlexIO CTRL = %08X\n", _pflexio->CTRL);
    Serial.printf(" FlexIO Config, param=%08X\n", _pflexio->PARAM);
#endif
    
		Serial.println("8Bit FlexIO");
      // SHIFTCFG, page 2927
      //  PWIDTH: number of bits to be shifted on each Shift clock
      //          0 = 1 bit, 1-3 = 4 bit, 4-7 = 8 bit, 8-15 = 16 bit, 16-31 = 32 bit
      //  INSRC: Input Source, 0 = pin, 1 = Shifter N+1 Output
      //  SSTOP: Stop bit, 0 = disabled, 1 = match, 2 = use zero, 3 = use one
      //  SSTART: Start bit, 0 = disabled, 1 = disabled, 2 = use zero, 3 = use one
      // setup the for shifters
      for (uint8_t i = 0; i < (CNT_SHIFTERS - 1); i++) {
        _pflexio->SHIFTCFG[i] = FLEXIO_SHIFTCFG_PWIDTH(7) | FLEXIO_SHIFTCFG_INSRC;
      }
      _pflexio->SHIFTCFG[CNT_SHIFTERS-1] = FLEXIO_SHIFTCFG_PWIDTH(7);
          
      // Timer model, pages 2891-2893
      // TIMCMP, page 2937
      // using 4 shifters
      _pflexio->TIMCMP[_ftimer] = (8U * CNT_SHIFTERS) -1 ;
      
      // TIMCTL, page 2933
      //  TRGSEL: Trigger Select ....
      //          4*N - Pin 2*N input
      //          4*N+1 - Shifter N status flag
      //          4*N+2 - Pin 2*N+1 input
      //          4*N+3 - Timer N trigger output
      //  TRGPOL: 0 = active high, 1 = active low
      //  TRGSRC: 0 = external, 1 = internal
      //  PINCFG: timer pin, 0 = disable, 1 = open drain, 2 = bidir, 3 = output
      //  PINSEL: which pin is used by the Timer input or output
      //  PINPOL: 0 = active high, 1 = active low
      //  TIMOD: mode, 0 = disable, 1 = 8 bit baud rate, 2 = 8 bit PWM, 3 = 16 bit
      _pflexio->TIMCTL[_ftimer] = FLEXIO_TIMCTL_TIMOD(3)
          | FLEXIO_TIMCTL_PINSEL(tpclk_pin) // "Pin" is 16 = PCLK
          | FLEXIO_TIMCTL_TRGSEL(4 * (thsync_pin/2)) // "Trigger" is 12 = HSYNC
          | FLEXIO_TIMCTL_TRGSRC;


    // SHIFTCTL, page 2926
    //  TIMSEL: which Timer is used for controlling the logic/shift register
    //  TIMPOL: 0 = shift of positive edge, 1 = shift on negative edge
    //  PINCFG: 0 = output disabled, 1 = open drain, 2 = bidir, 3 = output
    //  PINSEL: which pin is used by the Shifter input or output
    //  PINPOL: 0 = active high, 1 = active low
    //  SMOD: 0 = disable, 1 = receive, 2 = transmit, 4 = match store,
    //        5 = match continuous, 6 = state machine, 7 = logic
    // 4 shifters
    uint32_t shiftctl = FLEXIO_SHIFTCTL_TIMSEL(_ftimer) | FLEXIO_SHIFTCTL_SMOD(1)
        | FLEXIO_SHIFTCTL_PINSEL(tg0);    

    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) {
      _pflexio->SHIFTCTL[_fshifter + i] = shiftctl; // 4 = D0
    }

    // TIMCFG, page 2935
    //  TIMOUT: Output
    //          0 = output is logic one when enabled and is not affected by timer reset
    //          1 = output is logic zero when enabled and is not affected by timer reset
    //          2 = output is logic one when enabled and on timer reset
    //          3 = output is logic zero when enabled and on timer reset
    //  TIMDEC: Decrement
    //          0 = on FlexIO clock, Shift clock equals Timer output
    //          1 = on Trigger input (both edges), Shift clock equals Timer output
    //          2 = on Pin input (both edges), Shift clock equals Pin input
    //          3 = on Trigger input (both edges), Shift clock equals Trigger input
    //  TIMRST: Reset
    //          0 = never reset
    //          2 = on Timer Pin equal to Timer Output
    //          3 = on Timer Trigger equal to Timer Output
    //          4 = on Timer Pin rising edge
    //          6 = on Trigger rising edge
    //          7 = on Trigger rising or falling edge
    //  TIMDIS: Disable
    //          0 = never disabled
    //          1 = disabled on Timer N-1 disable
    //          2 = disabled on Timer compare
    //          3 = on Timer compare and Trigger Low
    //          4 = on Pin rising or falling edge
    //          5 = on Pin rising or falling edge provided Trigger is high
    //          6 = on Trigger falling edge
    //  TIMENA
    //          0 = always enabled
    //          1 = enabled on Timer N-1 enable
    //          2 = enabled on Trigger high
    //          3 = enabled on Trigger high and Pin high
    //          4 = enabled on Pin rising edge
    //          5 = enabled on Pin rising edge and Trigger high
    //          6 = enabled on Trigger rising edge
    //          7 = enabled on Trigger rising or falling edge
    //  TSTOP Stop bit, 0 = disabled, 1 = on compare, 2 = on disable, 3 = on either
    //  TSTART: Start bit, 0 = disabled, 1 = enabled
    _pflexio->TIMCFG[_ftimer] = FLEXIO_TIMCFG_TIMOUT(1) | FLEXIO_TIMCFG_TIMDEC(2)
        | FLEXIO_TIMCFG_TIMENA(6) | FLEXIO_TIMCFG_TIMDIS(6);

    // CTRL, page 2916
    _pflexio->CTRL = FLEXIO_CTRL_FLEXEN; // enable after everything configured
    
#ifdef DEBUG_FLEXIO
    Serial.printf(" FLEXIO:%u Shifter:%u Timer:%u\n", _pflex->FlexIOIndex(), _fshifter, _ftimer);
    Serial.print("     SHIFTCFG = ");
    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) Serial.printf(" %08X", _pflexio->SHIFTCFG[_fshifter + i]);
    Serial.print("\n     SHIFTCTL = ");
    for (uint8_t i = 0; i < CNT_SHIFTERS; i++) Serial.printf(" %08X", _pflexio->SHIFTCTL[_fshifter + i]);
    Serial.printf("\n     TIMCMP = %08X\n", _pflexio->TIMCMP[_ftimer]);
    Serial.printf("     TIMCFG = %08X\n", _pflexio->TIMCFG[_ftimer]);
    Serial.printf("     TIMCTL = %08X\n", _pflexio->SHIFTCTL[_fshifter]);
#endif
return true;
}


void dumpDMA_TCD(DMABaseClass *dmabc, const char *psz_title) {
  if (psz_title)
    Serial.print(psz_title);
  Serial.printf("%x %x: ", (uint32_t)dmabc, (uint32_t)dmabc->TCD);

  Serial.printf(
      "SA:%x SO:%d AT:%x (SM:%x SS:%x DM:%x DS:%x) NB:%x SL:%d DA:%x DO: %d CI:%x DL:%d CS:%x BI:%x\n",
      (uint32_t)dmabc->TCD->SADDR, dmabc->TCD->SOFF, dmabc->TCD->ATTR,
      (dmabc->TCD->ATTR >> 11) & 0x1f, (dmabc->TCD->ATTR >> 8) & 0x7,
      (dmabc->TCD->ATTR >> 3) & 0x1f, (dmabc->TCD->ATTR >> 0) & 0x7,
      dmabc->TCD->NBYTES, dmabc->TCD->SLAST, (uint32_t)dmabc->TCD->DADDR,
      dmabc->TCD->DOFF, dmabc->TCD->CITER, dmabc->TCD->DLASTSGA,
      dmabc->TCD->CSR, dmabc->TCD->BITER);
}

void OV767X::readFrameFlexIO(void* buffer)
{
    //flexio_configure(); // one-time hardware setup
    // wait for VSYNC to go high and then low with a sort of glitch filter
    elapsedMillis emWaitSOF;
    elapsedMicros emGlitch;
    for (;;) {
      if (emWaitSOF > 2000) {
        Serial.println("Timeout waiting for Start of Frame");
        return;
      }
      while ((*_vsyncPort & _vsyncMask) == 0);
      emGlitch = 0;
      while ((*_vsyncPort & _vsyncMask) != 0);
      if (emGlitch > 500) break;
    }

    _pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    _pflexio->SHIFTERR = _fshifter_mask;
    uint32_t *p = (uint32_t *)buffer;

#ifndef FLEXIO_USE_DMA
    // read FlexIO by polling
    uint32_t *p_end = (uint32_t *)buffer + (_width*_height/4)*_bytesPerPixel;

    while (p < p_end) {
        while ((_pflexio->SHIFTSTAT & _fshifter_mask) == 0) {
            // wait for FlexIO shifter data
        }
        *p++ = _pflexio->SHIFTBUF[_fshifter]; // should use DMA...
    }
#else
    // read FlexIO by DMA
    digitalWrite(49, HIGH);

    // Lets try like other implementation.
    const uint32_t frame_size_bytes = _width*_height*_bytesPerPixel;
    //uint32_t length_uint32 = frame_size_bytes / 4;
#if 1

    _dmachannel.begin();
    _dmachannel.triggerAtHardwareEvent(_dma_source);
    active_dma_camera = this;
    _dmachannel.attachInterrupt(dmaInterruptFlexIO);


#if (CNT_SHIFTERS == 4)
#define FXIO_SHFT_COUNT         4u          /* 4 shifters */
#define DMA_TRSF_SIZE           8u          /* 8 bytes */
#define DMA_MINOR_LOOP_SIZE     16u         /* 16 bytes */
#define DMA_MAJOR_LOOP_SIZE     (frame_size_bytes / DMA_MINOR_LOOP_SIZE)


    uint32_t soff, smod = 0u, size=0u;
    while(1u << size < DMA_TRSF_SIZE) /* size = log2(DMA_TRSF_SIZE) */
    {
        size++;
    }

    if(DMA_TRSF_SIZE == DMA_MINOR_LOOP_SIZE)
    {
        soff = 0u;
    }
    else
    {
        soff = DMA_TRSF_SIZE;
        while(1u << smod < DMA_MINOR_LOOP_SIZE) /* smod = log2(DMA_MINOR_LOOP_SIZE) */
        {
            smod++;
        }
    }
    /* Configure DMA TCD */
    _dmachannel.TCD->SADDR = &_pflexio->SHIFTBUF[_fshifter];
    _dmachannel.TCD->SOFF = soff;
    _dmachannel.TCD->ATTR = DMA_TCD_ATTR_SMOD(smod) |
                            DMA_TCD_ATTR_SSIZE(size) |
                            DMA_TCD_ATTR_DMOD(0u) |
                            DMA_TCD_ATTR_DSIZE(size);
    _dmachannel.TCD->NBYTES_MLNO = DMA_MINOR_LOOP_SIZE;
    _dmachannel.TCD->SLAST = 0u;
    _dmachannel.TCD->DADDR = p;
    _dmachannel.TCD->DOFF = DMA_TRSF_SIZE;
    _dmachannel.TCD->CITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
    _dmachannel.TCD->DLASTSGA = -frame_size_bytes;
    _dmachannel.TCD->CSR = 0u;
    _dmachannel.TCD->CSR |= DMA_TCD_CSR_DREQ;
    _dmachannel.TCD->BITER_ELINKNO = DMA_MAJOR_LOOP_SIZE;
#else
    // see if I configure for all 8 buffers
    #define SHIFT_BUFFERS_SIZE 32u
    #define SHIFT_BUFFERS_MOD 5u
    _dmachannel.TCD->SADDR = &_pflexio->SHIFTBUF[_fshifter];
    _dmachannel.TCD->SOFF = 0;
    _dmachannel.TCD->ATTR = DMA_TCD_ATTR_SMOD(0u) |
                            DMA_TCD_ATTR_SSIZE(SHIFT_BUFFERS_MOD) |
                            DMA_TCD_ATTR_DMOD(0u) |
                            DMA_TCD_ATTR_DSIZE(SHIFT_BUFFERS_MOD);
    _dmachannel.TCD->NBYTES_MLNO = SHIFT_BUFFERS_SIZE;
    _dmachannel.TCD->SLAST = 0u;
    _dmachannel.TCD->DADDR = p;
    _dmachannel.TCD->DOFF = SHIFT_BUFFERS_SIZE;
    _dmachannel.TCD->CITER_ELINKNO = (frame_size_bytes / SHIFT_BUFFERS_SIZE);
    _dmachannel.TCD->DLASTSGA = -frame_size_bytes;
    _dmachannel.TCD->CSR = 0u;
    _dmachannel.TCD->CSR |= DMA_TCD_CSR_DREQ;
    _dmachannel.TCD->BITER_ELINKNO = (frame_size_bytes / SHIFT_BUFFERS_SIZE);
#endif    
    /* Configure DMA MUX Source */
    //DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] = DMAMUX->CHCFG[FLEXIO_CAMERA_DMA_CHN] &
    //                                        (~DMAMUX_CHCFG_SOURCE_MASK) | 
    //                                        DMAMUX_CHCFG_SOURCE(FLEXIO_CAMERA_DMA_MUX_SRC);
    /* Enable DMA channel. */
    
    _dmachannel.disableOnCompletion();
    _dmachannel.interruptAtCompletion();
    _dmachannel.clearComplete();

    volatile uint32_t *mux = &DMAMUX_CHCFG0 +  _dmachannel.channel;
    Serial.printf("\nDMA CR: %08X Channel: %u %08X\n", DMA_CR, _dmachannel.channel, *mux);
    dumpDMA_TCD(&_dmachannel,"CH: ");
#else

    // Total length of bytes transfered
    // do it over 2 
    // first pass split into two
    _dmasettings[0].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[0].destinationBuffer(p, length / 2);
    _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);

    _dmasettings[1].source(_pflexio->SHIFTBUF[_fshifter]);
    _dmasettings[1].destinationBuffer(&p[length_uint32 / 2], length / 2);
    _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
    _dmasettings[1].disableOnCompletion();
    _dmasettings[1].interruptAtCompletion();

    _dmachannel = _dmasettings[0];

    _dmachannel.clearComplete();
    dumpDMA_TCD(&_dmachannel," CH: ");
    dumpDMA_TCD(&_dmasettings[0], " 0: ");
    dumpDMA_TCD(&_dmasettings[1], " 1: ");
#endif


    _dma_state = DMA_STATE_ONE_FRAME;
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dmachannel.enable();
    
    Serial.printf("Flexio DMA: length: %d\n", frame_size_bytes);

    elapsedMillis timeout = 0;
    //while (!_dmachannel.complete()) {
    while (_dma_state == DMA_STATE_ONE_FRAME) {
        // wait - we should not need to actually do anything during the DMA transfer
        if (_dmachannel.error()) {
            Serial.println("DMA error");
            if (_pflexio->SHIFTSTAT) Serial.printf(" SHIFTSTAT %08X\n", _pflexio->SHIFTSTAT);
            Serial.flush();
            uint64_t i = *(uint64_t*)&_pflexio->SHIFTBUF[0];
            Serial.printf("Result: %llx\n", i);


            _dmachannel.clearError();
            break;
        }
        if (timeout > 500) {
            Serial.println("Timeout waiting for DMA");
            if (_pflexio->SHIFTSTAT & _fshifter_mask) Serial.printf(" SHIFTSTAT bit was set (%08X)\n", _pflexio->SHIFTSTAT);
            Serial.printf(" DMA channel #%u\n", _dmachannel.channel);
            Serial.printf(" DMAMUX = %08X\n", *(&DMAMUX_CHCFG0 + _dmachannel.channel));
            Serial.printf(" _pflexio->SHIFTSDEN = %02X\n", _pflexio->SHIFTSDEN);
            Serial.printf(" TCD CITER = %u\n", _dmachannel.TCD->CITER_ELINKNO);
            Serial.printf(" TCD CSR = %08X\n", _dmachannel.TCD->CSR);
            break;
        }
    }
    digitalWrite(49, LOW);
    arm_dcache_delete(buffer, frame_size_bytes);
    dumpDMA_TCD(&_dmachannel,"CM: ");
//    dumpDMA_TCD(&_dmasettings[0], " 0: ");
//    dumpDMA_TCD(&_dmasettings[1], " 1: ");
#endif
}



bool OV767X::startReadFlexIO(bool(*callback)(void *frame_buffer), void *fb1, void *fb2)
{
#ifdef FLEXIO_USE_DMA
    if (fb1 == nullptr || fb2 == nullptr) return false;
    _frame_buffer_1 = (uint8_t *)fb1;
    _frame_buffer_2 = (uint8_t *)fb2;
    _callback = callback;
    active_dma_camera = this;
    //Serial.printf("startReadFrameFlexIO called buffers %x %x\n", (uint32_t)fb1, (uint32_t)fb2);

    //flexio_configure(); // one-time hardware setup
    dma_flexio.begin();
    const uint32_t length = _width*_height;
    dma_flexio.source(_pflexio->SHIFTBUF[_fshifter]);
    dma_flexio.destinationBuffer((uint32_t *)fb1, length);
    dma_flexio.transferSize(4);
    dma_flexio.transferCount(length / 4);
    dma_flexio.disableOnCompletion();
    dma_flexio.clearComplete();
    dma_flexio.triggerAtHardwareEvent(_dma_source);
    dma_flexio.interruptAtCompletion();
    dma_flexio.attachInterrupt(dmaInterruptFlexIO);
    _pflexio->SHIFTSDEN = _fshifter_mask;
    _dma_frame_count = 0;
    _dma_active = false;

    // wait for VSYNC to be low
    while ((*_vsyncPort & _vsyncMask) != 0);
    //NVIC_SET_PRIORITY(IRQ_GPIO6789, 102);
    //NVIC_SET_PRIORITY(dma_flexio.channel & 0xf, 102);
    attachInterrupt(_vsyncPin, &frameStartInterruptFlexIO, RISING);
    return true;
#else
    return false;
#endif
}


void OV767X::frameStartInterruptFlexIO()
{
	active_dma_camera->processFrameStartInterruptFlexIO();
}

void OV767X::processFrameStartInterruptFlexIO()
{
    if (!_dma_active) {
    	_pflexio->SHIFTSTAT = _fshifter_mask; // clear any prior shift status
    	_pflexio->SHIFTERR = _fshifter_mask;

    	// TODO: could a prior status have a DMA request still be pending?
    	void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
    	const uint32_t length = _width*_height;
    	//dma_flexio.TCD->DADDR = dest;
    	dma_flexio.destinationBuffer((uint32_t *)dest, length);
    	dma_flexio.transferSize(4);
    	dma_flexio.transferCount(length / 4);
    	dma_flexio.enable();
    	//Serial.println("VSYNC");
        _dma_active = true;
    }
	asm("DSB");
}

void OV767X::dmaInterruptFlexIO()
{
	active_dma_camera->processDMAInterruptFlexIO();
}

void OV767X::processDMAInterruptFlexIO()
{

  if (_dma_state == DMA_STATE_ONE_FRAME) {
    _dmachannel.clearInterrupt();
    _dma_state = DMA_STATE_STOPPED;
    asm("DSB");
    return;

  }

	dma_flexio.clearInterrupt();
	if (dma_flexio.error()) return; // TODO: report or handle error??
	void *dest = (_dma_frame_count & 1) ? _frame_buffer_2 : _frame_buffer_1;
	const uint32_t length = _width*_height;
	_dma_frame_count++;
	arm_dcache_delete(dest, length);
	if (_callback) (*_callback)(dest); // TODO: use EventResponder
    _dma_active = false;
	asm("DSB");
}


bool OV767X::stopReadFlexIO()
{
	detachInterrupt(_vsyncPin);
	dma_flexio.disable();
	_frame_buffer_1 = nullptr;
	_frame_buffer_2 = nullptr;
	_callback = nullptr;
	return true;
}


//======================================== DMA JUNK
//================================================================================
// experiment with DMA
//================================================================================
// Define our DMA structure.
DMAChannel OV767X::_dmachannel;
DMASetting OV767X::_dmasettings[4];
uint32_t OV767X::_dmaBuffer1[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
uint32_t OV767X::_dmaBuffer2[DMABUFFER_SIZE] __attribute__ ((used, aligned(32)));
extern "C" void xbar_connect(unsigned int input, unsigned int output); // in pwm.c

OV767X *OV767X::active_dma_camera = nullptr;


//===================================================================
// Start a DMA operation -
//===================================================================
bool OV767X::startReadFrameDMA(bool(*callback)(void *frame_buffer), uint8_t *fb1, uint8_t *fb2)
{
  // First see if we need to allocate frame buffers.
  if (fb1) _frame_buffer_1 = fb1;
  else if (_frame_buffer_1 == nullptr) {
    _frame_buffer_1 = (uint8_t*)malloc(_width * _height );
    if (_frame_buffer_1 == nullptr) return false;
  }
  if (fb2) _frame_buffer_2 = fb2;
  else if (_frame_buffer_2 == nullptr) {
    _frame_buffer_2 = (uint8_t*)malloc(_width * _height);
    if (_frame_buffer_2 == nullptr) return false; // BUGBUG should we 32 byte align?
  }
  // remember the call back if passed in
  _callback = callback;
  active_dma_camera = this;

  Serial.printf("startReadFrameDMA called buffers %x %x\n", (uint32_t)_frame_buffer_1, (uint32_t)_frame_buffer_2);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  // lets figure out how many bytes we will tranfer per setting...
  //  _dmasettings[0].begin();
  _frame_row_buffer_pointer = _frame_buffer_pointer = (uint8_t *)_frame_buffer_1;

  // configure DMA channels
  _dmachannel.begin();
  _dmasettings[0].source(GPIO2_PSR); // setup source.
  _dmasettings[0].destinationBuffer(_dmaBuffer1, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[0].replaceSettingsOnCompletion(_dmasettings[1]);
  _dmasettings[0].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  _dmasettings[1].source(GPIO2_PSR); // setup source.
  _dmasettings[1].destinationBuffer(_dmaBuffer2, DMABUFFER_SIZE * 4);  // 32 bits per logical byte
  _dmasettings[1].replaceSettingsOnCompletion(_dmasettings[0]);
  _dmasettings[1].interruptAtCompletion();  // we will need an interrupt to process this.
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  GPIO2_GDIR = 0; // set all as input...
  GPIO2_DR = 0; // see if I can clear it out...

  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.attachInterrupt(dmaInterrupt);
  _dmachannel.triggerAtHardwareEvent(DMAMUX_SOURCE_XBAR1_0);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Lets try to setup the DMA setup...
  // first see if we can convert the _pclk to be an XBAR Input pin...
    // OV7670_PLK   4
  // OV7670_PLK   8    //8       B1_00   FlexIO2:16  XBAR IO14

  _save_pclkPin_portConfigRegister = *(portConfigRegister(_pclkPin));
  *(portConfigRegister(_pclkPin)) = 1; // set to XBAR mode 14

  // route the timer outputs through XBAR to edge trigger DMA request
  CCM_CCGR2 |= CCM_CCGR2_XBAR1(CCM_CCGR_ON);
  xbar_connect(XBARA1_IN_IOMUX_XBAR_INOUT14, XBARA1_OUT_DMA_CH_MUX_REQ30);
  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Tell XBAR to dDMA on Rising
  XBARA1_CTRL0 = XBARA_CTRL_STS0 | XBARA_CTRL_EDGE0(1) | XBARA_CTRL_DEN0/* | XBARA_CTRL_IEN0 */ ;

  IOMUXC_GPR_GPR6 &= ~(IOMUXC_GPR_GPR6_IOMUXC_XBAR_DIR_SEL_14);  // Make sure it is input mode
  IOMUXC_XBAR1_IN14_SELECT_INPUT = 1; // Make sure this signal goes to this pin...


#if defined (ARDUINO_TEENSY_MICROMOD)
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR27 &= ~_hrefMask; //
#else
  // Need to switch the IO pins back to GPI1 from GPIO6
  _save_IOMUXC_GPR_GPR26 = IOMUXC_GPR_GPR26;  // save away the configuration before we change...
  IOMUXC_GPR_GPR26 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  IOMUXC_GPR_GPR26 &= ~_hrefMask; //
#endif

  // Need to switch the IO pins back to GPI1 from GPIO6
  //_save_IOMUXC_GPR_GPR27 = IOMUXC_GPR_GPR27;  // save away the configuration before we change...
  //IOMUXC_GPR_GPR27 &= ~(0x0ff0u);

  // lets also un map the _hrefPin to GPIO1
  //IOMUXC_GPR_GPR27 &= ~_hrefMask; //


  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);

  // Falling edge indicates start of frame
//  while ((*_vsyncPort & _vsyncMask) == 0); // wait for HIGH
//  while ((*_vsyncPort & _vsyncMask) != 0); // wait for LOW
//  DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);

// Debug stuff for now

  // We have the start of a frame, so lets start the dma.
#ifdef DEBUG_CAMERA
  dumpDMA_TCD(&_dmachannel," CH: ");
  dumpDMA_TCD(&_dmasettings[0], " 0: ");
  dumpDMA_TCD(&_dmasettings[1], " 1: ");

  Serial.printf("pclk pin: %d config:%lx control:%lx\n", _pclkPin, *(portConfigRegister(_pclkPin)), *(portControlRegister(_pclkPin)));
  Serial.printf("IOMUXC_GPR_GPR26-29:%lx %lx %lx %lx\n", IOMUXC_GPR_GPR26, IOMUXC_GPR_GPR27, IOMUXC_GPR_GPR28, IOMUXC_GPR_GPR29);
  Serial.printf("GPIO1: %lx %lx, GPIO6: %lx %lx\n", GPIO1_DR, GPIO1_PSR, GPIO6_DR, GPIO6_PSR);
  Serial.printf("XBAR CTRL0:%x CTRL1:%x\n\n", XBARA1_CTRL0, XBARA1_CTRL1);
#endif
  _dma_state = DMASTATE_RUNNING;
  _dma_last_completed_frame = nullptr;
  _dma_frame_count = 0;

  // Now start an interrupt for start of frame. 
  attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);

  //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
  return true;
}

//===================================================================
// stopReadFrameDMA - stop doing the reading and then exit.
//===================================================================
bool OV767X::stopReadFrameDMA()
{

  // hopefully it start here (fingers crossed)
  // for now will hang here to see if completes...
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, HIGH);
  elapsedMillis em = 0;
  // tell the background stuff DMA stuff to exit.
  // Note: for now let it end on on, later could disable the DMA directly.
  _dma_state = DMASTATE_STOP_REQUESTED;

  while ((em < 1000) && (_dma_state == DMASTATE_STOP_REQUESTED)) ; // wait up to a second...
  if (_dma_state != DMA_STATE_STOPPED) {
    Serial.println("*** stopReadFrameDMA DMA did not exit correctly...");
    Serial.printf("  Bytes Left: %u frame buffer:%x Row:%u Col:%u\n", _bytes_left_dma, (uint32_t)_frame_buffer_pointer, _frame_row_index, _frame_col_index);
  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);

#ifdef DEBUG_CAMERA
  dumpDMA_TCD(&_dmachannel, nullptr);
  dumpDMA_TCD(&_dmasettings[0], nullptr);
  dumpDMA_TCD(&_dmasettings[1], nullptr);
  Serial.println();
#endif
  // Lets restore some hardware pieces back to the way we found them.
#if defined (ARDUINO_TEENSY_MICROMOD)
  IOMUXC_GPR_GPR27 = _save_IOMUXC_GPR_GPR27;  // Restore... away the configuration before we change...
#else
  IOMUXC_GPR_GPR26 = _save_IOMUXC_GPR_GPR26;  // Restore... away the configuration before we change...
#endif
  *(portConfigRegister(_pclkPin)) = _save_pclkPin_portConfigRegister;

  return (em < 1000); // did we stop...
}

//===================================================================
// Our Frame Start interrupt.
//===================================================================
void  OV767X::frameStartInterrupt() {
  active_dma_camera->processFrameStartInterrupt();  // lets get back to the main object...
}

void  OV767X::processFrameStartInterrupt() {
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height; // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again. 
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.enable();
  
  detachInterrupt(_vsyncPin);
}

//===================================================================
// Our DMA interrupt.
//===================================================================
void OV767X::dmaInterrupt() {
  active_dma_camera->processDMAInterrupt();  // lets get back to the main object...
}


// This version assumes only called when HREF...  as set pixclk to only fire
// when set.
void OV767X::processDMAInterrupt() {
  _dmachannel.clearInterrupt(); // tell system we processed it.
  asm("DSB");
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, HIGH);

  if (_dma_state == DMA_STATE_STOPPED) {
    Serial.println("OV767X::dmaInterrupt called when DMA_STATE_STOPPED");
    return; //
  }


  // lets guess which buffer completed.
  uint32_t *buffer;
  uint16_t buffer_size;
  _dma_index++;
  if (_dma_index & 1) {
    buffer = _dmaBuffer1;
    buffer_size = _dmasettings[0].TCD->CITER;

  } else {
    buffer = _dmaBuffer2;
    buffer_size = _dmasettings[1].TCD->CITER;
  }
  // lets try dumping a little data on 1st 2nd and last buffer.
#ifdef DEBUG_CAMERA_VERBOSE
  if ((_dma_index < 3) || (buffer_size  < DMABUFFER_SIZE)) {
    Serial.printf("D(%d, %d, %lu) %u : ", _dma_index, buffer_size, _bytes_left_dma, pixformat);
    for (uint16_t i = 0; i < 8; i++) {
      uint16_t b = buffer[i] >> 4;
      Serial.printf(" %lx(%02x)", buffer[i], b);
    }
    Serial.print("...");
    for (uint16_t i = buffer_size - 8; i < buffer_size; i++) {
      uint16_t b = buffer[i] >> 4;
      Serial.printf(" %lx(%02x)", buffer[i], b);
    }
    Serial.println();
  }
#endif

  for (uint16_t buffer_index = 0; buffer_index < buffer_size; buffer_index++) {
    if (!_bytes_left_dma || (_frame_row_index >= _height)) break;

    // only process if href high...
    uint16_t b = *buffer >> 4;
    *_frame_buffer_pointer++ = b;
    _frame_col_index++;
    if (_frame_col_index == _width) {
        // we just finished a row.
        _frame_row_index++;
        _frame_col_index = 0;
    }
    _bytes_left_dma--; // for now assuming color 565 image...
    buffer++;
  }

  if ((_frame_row_index == _height) || (_bytes_left_dma == 0)) { // We finished a frame lets bail
    _dmachannel.disable();  // disable the DMA now...
    //DebugDigitalWrite(OV7670_DEBUG_PIN_2, LOW);
#ifdef DEBUG_CAMERA_VERBOSE
    Serial.println("EOF");
#endif
    _frame_row_index = 0;
    _dma_frame_count++;

    bool swap_buffers = true;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);
    _dma_last_completed_frame = _frame_row_buffer_pointer;
    if (_callback) swap_buffers = (*_callback)(_dma_last_completed_frame);

    if (swap_buffers) {
        if (_frame_row_buffer_pointer != _frame_buffer_1) _frame_row_buffer_pointer = _frame_buffer_2;
        else _frame_row_buffer_pointer = _frame_buffer_2;    
    }

    _frame_buffer_pointer = _frame_row_buffer_pointer;

    //DebugDigitalToggle(OV7670_DEBUG_PIN_1);


    if (_dma_state == DMASTATE_STOP_REQUESTED) {
#ifdef DEBUG_CAMERA
      Serial.println("OV767X::dmaInterrupt - Stop requested");
#endif
      _dma_state = DMA_STATE_STOPPED;
    } else {
      // We need to start up our ISR for the next frame. 
#if 1
  // bypass interrupt and just restart DMA... 
  _bytes_left_dma = (_width + _frame_ignore_cols) * _height; // for now assuming color 565 image...
  _dma_index = 0;
  _frame_col_index = 0;  // which column we are in a row
  _frame_row_index = 0;  // which row
  _save_lsb = 0xffff;
  // make sure our DMA is setup properly again. 
  _dmasettings[0].transferCount(DMABUFFER_SIZE);
  _dmasettings[0].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmasettings[1].transferCount(DMABUFFER_SIZE);
  _dmasettings[1].TCD->CSR &= ~(DMA_TCD_CSR_DREQ); // Don't disable on this one
  _dmachannel = _dmasettings[0];  // setup the first on...
  _dmachannel.enable();

#else
      attachInterrupt(_vsyncPin, &frameStartInterrupt, RISING);
#endif
    }
  } else {

    if (_bytes_left_dma == (2 * DMABUFFER_SIZE)) {
      if (_dma_index & 1) _dmasettings[0].disableOnCompletion();
      else _dmasettings[1].disableOnCompletion();
    }

  }
  //DebugDigitalWrite(OV7670_DEBUG_PIN_3, LOW);
}

typedef struct {
    uint32_t frameTimeMicros;
    uint16_t vsyncStartCycleCount;
    uint16_t vsyncEndCycleCount;
    uint16_t hrefCount;
    uint32_t cycleCount;
    uint16_t pclkCounts[350]; // room to spare.
    uint32_t hrefStartTime[350];
    uint16_t pclkNoHrefCount;
} frameStatics_t;

frameStatics_t fstat;

void OV767X::captureFrameStatistics()
{
   memset((void*)&fstat, 0, sizeof(fstat));

   // lets wait for the vsync to go high;
    while ((*_vsyncPort & _vsyncMask) != 0); // wait for HIGH
    // now lets wait for it to go low    
    while ((*_vsyncPort & _vsyncMask) == 0) fstat.vsyncStartCycleCount ++; // wait for LOW

    while ((*_hrefPort & _hrefMask) == 0); // wait for HIGH
    while ((*_pclkPort & _pclkMask) != 0); // wait for LOW

    uint32_t microsStart = micros();
    fstat.hrefStartTime[0] = microsStart;
    // now loop through until we get the next _vsynd
    // BUGBUG We know that HSYNC and PCLK on same GPIO VSYNC is not...
    uint32_t regs_prev = 0;
    //noInterrupts();
    while ((*_vsyncPort & _vsyncMask) != 0) {

        fstat.cycleCount++;
        uint32_t regs = (*_hrefPort & (_hrefMask | _pclkMask ));
        if (regs != regs_prev) {
            if ((regs & _hrefMask) && ((regs_prev & _hrefMask) ==0)) {
                fstat.hrefCount++;
                fstat.hrefStartTime[fstat.hrefCount] = micros();
            }
            if ((regs & _pclkMask) && ((regs_prev & _pclkMask) ==0)) fstat.pclkCounts[fstat.hrefCount]++;
            if ((regs & _pclkMask) && ((regs_prev & _hrefMask) ==0)) fstat.pclkNoHrefCount++;
            regs_prev = regs;
        }
    }
    while ((*_vsyncPort & _vsyncMask) == 0) fstat.vsyncEndCycleCount++; // wait for LOW
    //interrupts();
    fstat.frameTimeMicros = micros() - microsStart;

    // Maybe return data. print now
    Serial.printf("*** Frame Capture Data: elapsed Micros: %u loops: %u\n", fstat.frameTimeMicros, fstat.cycleCount);
    Serial.printf("   VSync Loops Start: %u end: %u\n", fstat.vsyncStartCycleCount, fstat.vsyncEndCycleCount);
    Serial.printf("   href count: %u pclk ! href count: %u\n    ", fstat.hrefCount,  fstat.pclkNoHrefCount);
    for (uint16_t ii=0; ii < fstat.hrefCount + 1; ii++) {
        Serial.printf("%3u(%u) ", fstat.pclkCounts[ii], (ii==0)? 0 : fstat.hrefStartTime[ii] - fstat.hrefStartTime[ii-1]);
        if (!(ii & 0x0f)) Serial.print("\n    ");
    }
    Serial.println();
}

OV767X Camera;
