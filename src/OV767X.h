// SPDX-License-Identifier: GPL-2.0-only
/*
 * This file is part of the Arduino_OX767X library.
 * Copyright (c) 2020 Arduino SA. All rights reserved.
 */

#ifndef _OV767X_H_
#define _OV767X_H_

#include <Arduino.h>
#if defined(__IMXRT1062__)  // Teensy 4.x
#include <DMAChannel.h>
//#define OV7670_VSYNC 2    // Lets setup for T4.1 CSI pins
#define USE_CSI_PINS

#if defined USE_CSI_PINS
#define OV7670_PLK   40 //40 // AD_B1_04 CSI_PIXCLK
#define OV7670_XCLK_JUMPER 41 // BUGBUG CSI 41 is NOT a PWM pin so we jumper to it...
#define OV7670_XCLK  37  //41 // AD_B1_05 CSI_MCLK
#define OV7670_HREF  16 // AD_B1_07 CSI_HSYNC
#define OV7670_VSYNC 17 // AD_B1_06 CSI_VSYNC

#define OV7670_D0    27 // AD_B1_15 CSI_D2
#define OV7670_D1    26 // AD_B1_14 CSI_D3
#define OV7670_D2    39 // AD_B1_13 CSI_D4
#define OV7670_D3    38 // AD_B1_12 CSI_D5
#define OV7670_D4    21 // AD_B1_11 CSI_D6
#define OV7670_D5    20 // AD_B1_10 CSI_D7
#define OV7670_D6    23 // AD_B1_09 CSI_D8
#define OV7670_D7    22 // AD_B1_08 CSI_D9
#elif 1
#define OV7670_PLK   4 //40 // AD_B1_04 CSI_PIXCLK
#define OV7670_XCLK  5  //41 // AD_B1_05 CSI_MCLK
#define OV7670_HREF  40 // AD_B1_07 CSI_HSYNC
#define OV7670_VSYNC 41 // AD_B1_06 CSI_VSYNC

#define OV7670_D0    27 // AD_B1_02 1.18
#define OV7670_D1    15 // AD_B1_03 1.19
#define OV7670_D2    17 // AD_B1_06 1.22
#define OV7670_D3    16 // AD_B1_07 1.23
#define OV7670_D4    22 // AD_B1_08 1.24
#define OV7670_D5    23 // AD_B1_09 1.25
#define OV7670_D6    20 // AD_B1_10 1.26
#define OV7670_D7    21 // AD_B1_11 1.27

#else
// For T4.1 can choose same or could choose a contiguous set of pins only one shift required.
// Like:  Note was going to try GPI pins 1.24-21 but save SPI1 pins 26,27 as no ...
#define OV7670_PLK   4
#define OV7670_XCLK  5
#define OV7670_HREF  40 // AD_B1_04 1.20 T4.1...
#define OV7670_VSYNC 41 // AD_B1_05 1.21 T4.1...

#define OV7670_D0    17 // AD_B1_06 1.22
#define OV7670_D1    16 // AD_B1_07 1.23
#define OV7670_D2    22 // AD_B1_08 1.24
#define OV7670_D3    23 // AD_B1_09 1.25
#define OV7670_D4    20 // AD_B1_10 1.26
#define OV7670_D5    21 // AD_B1_11 1.27
#define OV7670_D6    38 // AD_B1_12 1.28
#define OV7670_D7    39 // AD_B1_13 1.29
#endif
//      #define OV7670_D6    26 // AD_B1_14 1.30
//      #define OV7670_D7    27 // AD_B1_15 1.31


#endif
enum
{
  YUV422 = 0,
  RGB444 = 1,
  RGB565 = 2,
  // SBGGR8 = 3
  GRAYSCALE = 4
};

enum
{
  VGA = 0,  // 640x480
  CIF = 1,  // 352x240
  QVGA = 2, // 320x240
  QCIF = 3,  // 176x144
  QQVGA = 4,  // 160x120
};

class OV767X
{
public:
  OV767X();
  virtual ~OV767X();

  int begin(int resolution, int format, int fps); // Supported FPS: 1, 5, 10, 15, 30
  void end();

  // must be called after Camera.begin():
  int width() const;
  int height() const;
  int bitsPerPixel() const;
  int bytesPerPixel() const;

  void readFrame(void* buffer);

  // Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
  // So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA 
  // buffers, with the option of setting your own buffers if desired.
  bool startReadFrameDMA(void(*callback)(void *frame_buffer)=nullptr);
  bool stopReadFrameDMA();
  inline uint32_t frameCount() {return _dma_frame_count;}
  inline void *frameBuffer() {return _dma_last_completed_frame;}
  // TBD Allow user to set all of the buffers...
  void readFrameDMA(void* buffer);


  void testPattern(int pattern = 2);
  void noTestPattern();

  void setSaturation(int saturation); // 0 - 255
  void setHue(int hue); // -180 - 180
  void setBrightness(int brightness); // 0 - 255
  void setContrast(int contrast); // 0 - 127
  void horizontalFlip();
  void noHorizontalFlip();
  void verticalFlip();
  void noVerticalFlip();
  void setGain(int gain); // 0 - 255
  void autoGain();
  void setExposure(int exposure); // 0 - 65535
  void autoExposure();
  void showRegisters();

  // must be called before Camera.begin()
  void setPins(int vsync, int href, int pclk, int xclk, const int dpins[8]);


private:
  void beginXClk();
  void endXClk();

private:
  int _vsyncPin;
  int _hrefPin;
  int _pclkPin;
  int _xclkPin;
  int _dPins[8];

  int _width;
  int _height;
  int _bytesPerPixel;
  bool _grayscale;

  void* _ov7670;

  volatile uint32_t* _vsyncPort;
  uint32_t _vsyncMask;
  volatile uint32_t* _hrefPort;
  uint32_t _hrefMask;
  volatile uint32_t* _pclkPort;
  uint32_t _pclkMask;

  int _saturation;
  int _hue;

  // Lets try adding some DMA support.
  #if defined(__IMXRT1062__)  // Teensy 4.x
      enum {DMABUFFER_SIZE=1280};  // 640x480  so 640*2*2
      static DMAChannel _dmachannel;
      static DMASetting _dmasettings[2];
      static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
      static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

      void (*_callback)(void *frame_buffer) =nullptr ;
      uint32_t  _dma_frame_count;
      uint16_t *_dma_last_completed_frame;
  // TBD Allow user to set all of the buffers...

      uint32_t _save_IOMUXC_GPR_GPR26;
      uint32_t _save_pclkPin_portConfigRegister;

      uint32_t _bytes_left_dma;
      uint16_t  _save_lsb;
      uint16_t  _frame_col_index;  // which column we are in a row
      uint16_t  _frame_row_index;  // which row

      uint16_t *_frame_buffer_1 = nullptr;
      uint16_t *_frame_buffer_2 = nullptr;
      uint16_t *_frame_buffer_pointer;
      uint16_t *_frame_row_buffer_pointer; // start of the row
      uint16_t _dma_index;
      enum {DMASTATE_INITIAL=0, DMASTATE_RUNNING, DMASTATE_STOP_REQUESTED, DMA_STATE_STOPPED};
      volatile uint8_t _dma_state;
  static void dmaInterrupt(); 
  void processDMAInterrupt();
  static void frameStartInterrupt();
  void processFrameStartInterrupt();

  #endif
};

extern OV767X Camera;

#endif
