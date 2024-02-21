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
#include <Wire.h>
#include <FlexIO_t4.h>


//#define OV7670_VSYNC 2    // Lets setup for T4.1 CSI pins
//#define USE_CSI_PINS


#define OV7670_USE_DEBUG_PINS
#ifdef  OV7670_USE_DEBUG_PINS
#define OV7670_DEBUG_PIN_1 14
#define OV7670_DEBUG_PIN_2 15
#define OV7670_DEBUG_PIN_3 3
#define DebugDigitalWrite(pin, val) digitalWriteFast(pin, val)
#define DebugDigitalToggle(pin) digitalToggleFast(pin)
#else
#define DebugDigitalWrite(pin, val) 
#define DebugDigitalToggle(pin)
#endif


#ifdef ARDUINO_TEENSY_MICROMOD
/*
HM01B0 pin      pin#    NXP     Usage
----------      ----    ---     -----
FVLD/VSYNC      33      EMC_07  GPIO
LVLD/HSYNC      32      B0_12   FlexIO2:12
MCLK            7       B1_01   PWM
PCLK            8       B1_00   FlexIO2:16
D0              40      B0_04   FlexIO2:4
D1              41      B0_05   FlexIO2:5
D2              42      B0_06   FlexIO2:6
D3              43      B0_07   FlexIO2:7
D4              44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
D5              45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
D6              6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
D7              9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode
TRIG            5       EMC_08  ???
INT             29      EMC_31  ???
SCL             19      AD_B1_0 I2C
SDA             18      AD_B1_1 I2C
*/

#define OV7670_PLK   8    //8       B1_00   FlexIO2:16
#define OV7670_XCLK  7    //7       B1_01   PWM
#define OV7670_HREF  32   //32      B0_12   FlexIO2:12, pin 46 on sdram board
#define OV7670_VSYNC 33   //33      EMC_07  GPIO, 21 pon sdram board
#define OV7670_RST   17  // reset pin 

#define OV7670_D0    40   //40      B0_04   FlexIO2:4
#define OV7670_D1    41   //41      B0_05   FlexIO2:5
#define OV7670_D2    42   //42      B0_06   FlexIO2:6
#define OV7670_D3    43   //43      B0_07   FlexIO2:7
#define OV7670_D4    44   //44      B0_08   FlexIO2:8  - probably not needed, use 4 bit mode
#define OV7670_D5    45   //45      B0_09   FlexIO2:9  - probably not needed, use 4 bit mode
#define OV7670_D6    6    //6       B0_10   FlexIO2:10 - probably not needed, use 4 bit mode
#define OV7670_D7    9    //9       B0_11   FlexIO2:11 - probably not needed, use 4 bit mode

#elif defined USE_CSI_PINS
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
  OV7670 = 0,
  OV7675 = 1
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

  int begin(int resolution, int format, int fps,  int camera_name = OV7670, bool use_gpio = false); // Supported FPS: 1, 5, 10, 15, 30
  void end();

  // must be called after Camera.begin():
  int width() const;
  int height() const;
  int bitsPerPixel() const;
  int bytesPerPixel() const;
/********************************************************************************************/
	//-------------------------------------------------------
	//Generic Read Frame base on _hw_config
	void readFrame(void* buffer, bool use_dma=true);
	
	//normal Read mode
	void readFrameGPIO(void* buffer);
	bool readContinuous(bool(*callback)(void *frame_buffer), void *fb1, void *fb2);
	void stopReadContinuous();

	//FlexIO is default mode for the camera
	void readFrameFlexIO(void* buffer, bool use_dma=true);
	bool startReadFlexIO(bool (*callback)(void *frame_buffer), void *fb1, void *fb2);
	bool stopReadFlexIO();

	// Lets try a dma version.  Doing one DMA that is synchronous does not gain anything
	// So lets have a start, stop... Have it allocate 2 frame buffers and it's own DMA 
	// buffers, with the option of setting your own buffers if desired.
	bool startReadFrameDMA(bool (*callback)(void *frame_buffer)=nullptr, uint8_t *fb1=nullptr, uint8_t *fb2=nullptr);
	void changeFrameBuffer(uint8_t *fbFrom, uint8_t *fbTo) {
		if (_frame_buffer_1 == fbFrom) _frame_buffer_1 = fbTo;
		else if (_frame_buffer_2 == fbFrom) _frame_buffer_2 = fbTo;
	}
	bool stopReadFrameDMA();	
	inline uint32_t frameCount() {return _dma_frame_count;}
	inline void *frameBuffer() {return _dma_last_completed_frame;}
	void captureFrameStatistics();
	
	void setVSyncISRPriority(uint8_t priority) {NVIC_SET_PRIORITY(IRQ_GPIO6789, priority); }
	void setDMACompleteISRPriority(uint8_t priority) {NVIC_SET_PRIORITY(_dmachannel.channel & 0xf, priority); }
    /********************************************************************************************/

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

  bool _use_gpio = false;

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

	bool flexio_configure();

	// DMA STUFF
	enum {DMABUFFER_SIZE=1296};  // 640x480  so 640*2*2
	static DMAChannel _dmachannel;
	static DMASetting _dmasettings[4];
	static uint32_t _dmaBuffer1[DMABUFFER_SIZE];
	static uint32_t _dmaBuffer2[DMABUFFER_SIZE];

	bool (*_callback)(void *frame_buffer) = nullptr ;
	uint32_t  _dma_frame_count;
	uint8_t *_dma_last_completed_frame;
	// TBD Allow user to set all of the buffers...


	// Added settings for configurable flexio
	FlexIOHandler *_pflex;
    IMXRT_FLEXIO_t *_pflexio;
	uint8_t _fshifter;
	uint8_t _fshifter_mask;
    uint8_t _ftimer;
    uint8_t _dma_source;



	#if defined (ARDUINO_TEENSY_MICROMOD)
	uint32_t _save_IOMUXC_GPR_GPR27;
	#else
	uint32_t _save_IOMUXC_GPR_GPR26;
	#endif      
	uint32_t _save_pclkPin_portConfigRegister;

	uint32_t _bytes_left_dma;
	uint16_t  _save_lsb;
	uint16_t  _frame_col_index;  // which column we are in a row
	uint16_t  _frame_row_index;  // which row
	const uint16_t  _frame_ignore_cols = 0; // how many cols to ignore per row
	uint8_t *_frame_buffer_1 = nullptr;
	uint8_t *_frame_buffer_2 = nullptr;
	uint8_t *_frame_buffer_pointer;
	uint8_t *_frame_row_buffer_pointer; // start of the row
	uint8_t _dma_index;
	bool 	_dma_active;
	enum {DMASTATE_INITIAL=0, DMASTATE_RUNNING, DMASTATE_STOP_REQUESTED, DMA_STATE_STOPPED, DMA_STATE_ONE_FRAME};
	volatile uint8_t _dma_state;
	static void dmaInterrupt(); 
	void processDMAInterrupt();
#if 0
	static void frameStartInterrupt();
	void processFrameStartInterrupt();
#endif	
	static void dmaInterruptFlexIO();
	void processDMAInterruptFlexIO();
//	static void frameStartInterruptFlexIO();
//	void processFrameStartInterruptFlexIO();
	static OV767X *active_dma_camera;


};

extern OV767X Camera;

#endif
