/*
  OV767X - Camera Test Pattern

  This sketch waits for the letter 'c' on the Serial Monitor,
  it then reads a frame from the OmniVision OV7670 camera and
  prints the data to the Serial Monitor as a hex string.

  The website https://rawpixels.net - can be used the visualize the data:
    width: 176
    height: 144
    RGB5
    Little Endian

  Circuit:

 
  This example code is in the public domain.
*/
//#define USE_ILI9488
//#define USE_ST7789

#ifdef ARDUINO_TEENSY41
extern "C" {
  extern uint8_t external_psram_size;
}
#endif

#include <Arduino_OV767X.h>
#include "arm_math.h"

#if defined(USE_ST7789)
#include <ST7735_t3.h>  // Hardware-specific library
#include <ST7789_t3.h>  // Hardware-specific library

// Hacked up for TeensyMM
#define TFT_MISO 12
#define TFT_MOSI 11  //a12
#define TFT_SCK 13   //a13
#define TFT_DC 34    //9
#define TFT_CS 38    //10
#define TFT_RST 39   //8

#define RED ST77XX_RED
#define GREEN ST77XX_GREEN
#define BLUE ST77XX_BLUE
#define BLACK ST77XX_BLACK
#define CENTER ST7789_t3::CENTER

ST7789_t3 tft = ST7789_t3(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCK, TFT_RST);
 uint16_t pixels[320 * 240];  // QCIF: 176x144 X 2 bytes per pixel (RGB565)

#elif defined(USE_ILI9488)
#include <ILI9488_t3.h>
//uint16_t pixels[640 * 480] EXTMEM; // QCIF: 176x144 X 2 bytes per pixel (RGB565)
EXTMEM uint16_t pixels[320 * 240]  __attribute__ ((aligned(32))); // QCIF: 176x144 X 2 bytes per pixel (RGB565)
//#define TFT_CS   0  // AD_B0_02
//#define TFT_DC   1  // AD_B0_03
//#define TFT_RST 255
#define TFT_CS 10  // AD_B0_02
#define TFT_DC 25  // AD_B0_03
#define TFT_RST 24
#define RED ILI9488_RED
#define GREEN ILI9488_GREEN
#define BLUE ILI9488_BLUE
#define BLACK ILI9488_BLACK
#define CENTER ILI9488_t3::CENTER
ILI9488_t3 tft = ILI9488_t3(TFT_CS, TFT_DC, TFT_RST);
//RAFB frame_buffer[320 * 480] EXTMEM;
#else
#include <ILI9341_t3n.h>
DMAMEM uint16_t pixels[320 * 240]  __attribute__ ((aligned(32)));  // QCIF: 176x144 X 2 bytes per pixel (RGB565);  // QCIF: 176x144 X 2 bytes per pixel (RGB565)
//#define TFT_CS   0  // AD_B0_02
//#define TFT_DC   1  // AD_B0_03
//#define TFT_RST 255

#define TFT_DC  0   // "TX1" on left side of Sparkfun ML Carrier
#define TFT_CS  4   // "CS" on left side of Sparkfun ML Carrier
#define TFT_RST 1  // "RX1" on left side of Sparkfun ML Carrier


//#define TFT_DC   1  // AD_B0_03
//#define TFT_RST 255


#define RED ILI9341_RED
#define GREEN ILI9341_GREEN
#define BLUE ILI9341_BLUE
#define BLACK ILI9341_BLACK
#define CENTER ILI9341_t3n::CENTER
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);

#endif

uint16_t FRAME_WIDTH, FRAME_HEIGHT;
 uint16_t frameBuffer[(320) * 240];
uint8_t sendImageBuf[(320) * 240 * 2];
uint8_t frameBuffer2[(320) * 240] DMAMEM;

bool g_continuous_flex_mode = false;
void *volatile g_new_flexio_data = nullptr;
uint32_t g_flexio_capture_count = 0;
uint32_t g_flexio_redraw_count = 0;
elapsedMillis g_flexio_runtime;


bool g_continuous_mode = false;
bool g_dma_mode = false;
elapsedMillis g_emCycles;
uint32_t g_count_frames_output = 0;
const int pinCamReset = 30;


#define MCP(m) (uint16_t)(((m & 0xF8) << 8) | ((m & 0xFC) << 3) | (m >> 3))

static const uint16_t mono_palette[256] PROGMEM = {
  MCP(0x00), MCP(0x01), MCP(0x02), MCP(0x03), MCP(0x04), MCP(0x05), MCP(0x06), MCP(0x07), MCP(0x08), MCP(0x09), MCP(0x0a), MCP(0x0b), MCP(0x0c), MCP(0x0d), MCP(0x0e), MCP(0x0f),
  MCP(0x10), MCP(0x11), MCP(0x12), MCP(0x13), MCP(0x14), MCP(0x15), MCP(0x16), MCP(0x17), MCP(0x18), MCP(0x19), MCP(0x1a), MCP(0x1b), MCP(0x1c), MCP(0x1d), MCP(0x1e), MCP(0x1f),
  MCP(0x20), MCP(0x21), MCP(0x22), MCP(0x23), MCP(0x24), MCP(0x25), MCP(0x26), MCP(0x27), MCP(0x28), MCP(0x29), MCP(0x2a), MCP(0x2b), MCP(0x2c), MCP(0x2d), MCP(0x2e), MCP(0x2f),
  MCP(0x30), MCP(0x31), MCP(0x32), MCP(0x33), MCP(0x34), MCP(0x35), MCP(0x36), MCP(0x37), MCP(0x38), MCP(0x39), MCP(0x3a), MCP(0x3b), MCP(0x3c), MCP(0x3d), MCP(0x3e), MCP(0x3f),
  MCP(0x40), MCP(0x41), MCP(0x42), MCP(0x43), MCP(0x44), MCP(0x45), MCP(0x46), MCP(0x47), MCP(0x48), MCP(0x49), MCP(0x4a), MCP(0x4b), MCP(0x4c), MCP(0x4d), MCP(0x4e), MCP(0x4f),
  MCP(0x50), MCP(0x51), MCP(0x52), MCP(0x53), MCP(0x54), MCP(0x55), MCP(0x56), MCP(0x57), MCP(0x58), MCP(0x59), MCP(0x5a), MCP(0x5b), MCP(0x5c), MCP(0x5d), MCP(0x5e), MCP(0x5f),
  MCP(0x60), MCP(0x61), MCP(0x62), MCP(0x63), MCP(0x64), MCP(0x65), MCP(0x66), MCP(0x67), MCP(0x68), MCP(0x69), MCP(0x6a), MCP(0x6b), MCP(0x6c), MCP(0x6d), MCP(0x6e), MCP(0x6f),
  MCP(0x70), MCP(0x71), MCP(0x72), MCP(0x73), MCP(0x74), MCP(0x75), MCP(0x76), MCP(0x77), MCP(0x78), MCP(0x79), MCP(0x7a), MCP(0x7b), MCP(0x7c), MCP(0x7d), MCP(0x7e), MCP(0x7f),
  MCP(0x80), MCP(0x81), MCP(0x82), MCP(0x83), MCP(0x84), MCP(0x85), MCP(0x86), MCP(0x87), MCP(0x88), MCP(0x89), MCP(0x8a), MCP(0x8b), MCP(0x8c), MCP(0x8d), MCP(0x8e), MCP(0x8f),
  MCP(0x90), MCP(0x91), MCP(0x92), MCP(0x93), MCP(0x94), MCP(0x95), MCP(0x96), MCP(0x97), MCP(0x98), MCP(0x99), MCP(0x9a), MCP(0x9b), MCP(0x9c), MCP(0x9d), MCP(0x9e), MCP(0x9f),
  MCP(0xa0), MCP(0xa1), MCP(0xa2), MCP(0xa3), MCP(0xa4), MCP(0xa5), MCP(0xa6), MCP(0xa7), MCP(0xa8), MCP(0xa9), MCP(0xaa), MCP(0xab), MCP(0xac), MCP(0xad), MCP(0xae), MCP(0xaf),
  MCP(0xb0), MCP(0xb1), MCP(0xb2), MCP(0xb3), MCP(0xb4), MCP(0xb5), MCP(0xb6), MCP(0xb7), MCP(0xb8), MCP(0xb9), MCP(0xba), MCP(0xbb), MCP(0xbc), MCP(0xbd), MCP(0xbe), MCP(0xbf),
  MCP(0xc0), MCP(0xc1), MCP(0xc2), MCP(0xc3), MCP(0xc4), MCP(0xc5), MCP(0xc6), MCP(0xc7), MCP(0xc8), MCP(0xc9), MCP(0xca), MCP(0xcb), MCP(0xcc), MCP(0xcd), MCP(0xce), MCP(0xcf),
  MCP(0xd0), MCP(0xd1), MCP(0xd2), MCP(0xd3), MCP(0xd4), MCP(0xd5), MCP(0xd6), MCP(0xd7), MCP(0xd8), MCP(0xd9), MCP(0xda), MCP(0xdb), MCP(0xdc), MCP(0xdd), MCP(0xde), MCP(0xdf),
  MCP(0xe0), MCP(0xe1), MCP(0xe2), MCP(0xe3), MCP(0xe4), MCP(0xe5), MCP(0xe6), MCP(0xe7), MCP(0xe8), MCP(0xe9), MCP(0xea), MCP(0xeb), MCP(0xec), MCP(0xed), MCP(0xee), MCP(0xef),
  MCP(0xf0), MCP(0xf1), MCP(0xf2), MCP(0xf3), MCP(0xf4), MCP(0xf5), MCP(0xf6), MCP(0xf7), MCP(0xf8), MCP(0xf9), MCP(0xfa), MCP(0xfb), MCP(0xfc), MCP(0xfd), MCP(0xfe), MCP(0xff)
};


#define BMPIMAGEOFFSET 66
const char bmp_header[BMPIMAGEOFFSET] PROGMEM = {
  0x42, 0x4D, 0x36, 0x58, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x42, 0x00, 0x00, 0x00, 0x28, 0x00,
  0x00, 0x00, 0x40, 0x01, 0x00, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x01, 0x00, 0x10, 0x00, 0x03, 0x00,
  0x00, 0x00, 0x00, 0x58, 0x02, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0xC4, 0x0E, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xF8, 0x00, 0x00, 0xE0, 0x07, 0x00, 0x00, 0x1F, 0x00,
  0x00, 0x00
};

void setup() {
  while (!Serial && millis() < 4000)
    ;
  Serial.begin(9600);
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  SerialUSB1.begin(115200);
#endif
  pinMode(pinCamReset, OUTPUT);
  delay(10);
  digitalWriteFast(pinCamReset, LOW);
  delay(10);
  digitalWriteFast(pinCamReset, HIGH);  // subsequent resets via SCB

#ifdef ARDUINO_TEENSY41
  Serial.printf("EXT Memory size: %u\n", external_psram_size);
  extern unsigned long _extram_start;
  extern unsigned long _extram_end;
  Serial.printf("EXT Memory size: %u EXTMEM variables: %lx End: %lx\n", external_psram_size,
                (uint32_t)(&_extram_end - &_extram_start),
                (uint32_t)&_extram_end);
#endif

#if defined(USE_ST7789)
  tft.init(240, 320);  // Init ST7789 320x240
#elif defined(USE_ILI9488)
  tft.begin(32000000);
  //RAFB *frame_buffer = (RAFB*)sdram_malloc(sizeof(RAFB) * tft.width() * tft.height() + 32);
  //Serial.printf("frame_buffer: %lx", (uint32_t)frame_buffer);
  //frame_buffer =   (RAFB*)(((uint32_t)frame_buffer + 32) & 0xffffffe0);
  //Serial.printf("frame_buffer aligned: %lx", (uint32_t)frame_buffer);

  //tft.setFrameBuffer(frameBuffer);
#else
  tft.begin(18000000);
#endif
  tft.setRotation(1);
  tft.fillScreen(RED);
  delay(500);
  tft.fillScreen(GREEN);
  delay(500);
  tft.fillScreen(BLUE);
  delay(500);
  tft.fillScreen(BLACK);
  delay(500);

  //  while (!Serial);

  Serial.println("OV767X Camera Capture");
  Serial.flush();
  Serial.println();

  if (!Camera.begin(QVGA, RGB565, 5, OV7675)) {
    Serial.println("Failed to initialize camera!");
    while (1)
      ;
  }

  Serial.println("Camera settings:");
  Serial.print("\twidth = ");
  Serial.println(Camera.width());
  Serial.print("\theight = ");
  Serial.println(Camera.height());
  Serial.print("\tbits per pixel = ");
  Serial.println(Camera.bitsPerPixel());
  Serial.println();
  Serial.printf("TFT Width = %u Height = %u\n\n", tft.width(), tft.height());
  FRAME_HEIGHT = tft.height();
  FRAME_WIDTH =  tft.width();

  Camera.setContrast(0x30);
  Camera.setBrightness(0x80);
  Camera.autoExposure();
  //Camera.autoGain();
  //Camera.setDMACompleteISRPriority(192);  // lower than default

  showCommandList();
}


bool camera_flexio_callback(void *pfb) {
  //Serial.println("Flexio callback");
  g_new_flexio_data = pfb;
  return true;
}

// Quick and Dirty
#define UPDATE_ON_CAMERA_FRAMES

uint8_t *pfb_last_frame_returned = nullptr;

bool camera_flexio_callback_video(void *pfb) {
  pfb_last_frame_returned = (uint8_t *)pfb;
#ifdef UPDATE_ON_CAMERA_FRAMES
  tft.setOrigin(-2, -2);
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT);

  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t *)pfb_last_frame_returned, mono_palette);
  pfb_last_frame_returned = nullptr;
  tft.setOrigin(0, 0);
  uint16_t *pframebuf = tft.getFrameBuffer();
  if ((uint32_t)pframebuf >= 0x20200000u) arm_dcache_flush(pframebuf, FRAME_WIDTH * FRAME_HEIGHT);
#endif
  //Serial.print("#");
  return true;
}

void frame_complete_cb() {
  //Serial.print("@");
#ifndef UPDATE_ON_CAMERA_FRAMES
  if (!pfb_last_frame_returned) return;
  tft.setOrigin(-2, -2);
  if ((uint32_t)pfb_last_frame_returned >= 0x20200000u)
    arm_dcache_delete(pfb_last_frame_returned, FRAME_WIDTH * FRAME_HEIGHT);

  tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t *)pfb_last_frame_returned, mono_palette);
  pfb_last_frame_returned = nullptr;
  tft.setOrigin(0, 0);
  uint16_t *pfb = tft.getFrameBuffer();
  if ((uint32_t)pfb >= 0x20200000u) arm_dcache_flush(pfb, FRAME_WIDTH * FRAME_HEIGHT);
#endif
}

void loop() {
  char ch;
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
  while (SerialUSB1.available()) {
    ch = SerialUSB1.read();
    if (0x30 == ch) {
      Serial.print(F("ACK CMD CAM start single shoot ... "));
      send_image(&SerialUSB1);
      Serial.println(F("READY. END"));
    }
  }
#endif
  if (Serial.available()) {
    ch = Serial.read();
    switch (ch) {
      case 'p':
        {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
          send_raw();
          Serial.println("Image Sent!");
          ch = ' ';
#else
          Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
          break;
        }

      case 'f':
        {
          Serial.println("Reading frame");
          Serial.printf("Buffer: %p halfway: %p end:%p\n", pixels, &pixels[Camera.width()*Camera.height() / 2], &pixels[Camera.width()*Camera.height()]);
          memset((uint8_t *)pixels, 0, sizeof(pixels));
          //digitalWriteFast(14, HIGH);
          Camera.readFrame(pixels);
          //digitalWriteFast(14, LOW);
          Serial.println("Finished reading frame");
          Serial.flush();

          // Lets print out some of the first bytes and last bytes of the first couple of rows.
          for (volatile uint16_t *pfb = pixels; pfb < (pixels + 4 * Camera.width()); pfb += Camera.width()) {
            Serial.printf("\n%08x: ", (uint32_t)pfb);
            for (uint16_t i = 0; i < 8; i++) Serial.printf("%04x ", pfb[i]);
            Serial.print("..");
            Serial.print("..");
            for (uint16_t i = Camera.width() - 8; i < Camera.width(); i++) Serial.printf("%04x ", pfb[i]);
          }
          Serial.println("\n");

          // Lets dump out some of center of image.
          Serial.println("Show Center pixels\n");
          for (volatile uint16_t *pfb = pixels + Camera.width() * ((Camera.height() / 2) - 8); pfb < (pixels + Camera.width() * (Camera.height() / 2 + 8)); pfb += Camera.width()) {
            Serial.printf("\n%08x: ", (uint32_t)pfb);
            for (uint16_t i = 0; i < 8; i++) Serial.printf("%04x ", pfb[i]);
            Serial.print("..");
            for (uint16_t i = (Camera.width() / 2) - 4; i < (Camera.width() / 2) + 4; i++) Serial.printf("%04x ", pfb[i]);
            Serial.print("..");
            for (uint16_t i = Camera.width() - 8; i < Camera.width(); i++) Serial.printf("%04x ", pfb[i]);
          }
          Serial.println("\n...");

          int numPixels = Camera.width() * Camera.height();
          Serial.printf("TFT(%u, %u) Camera(%u, %u)\n", tft.width(), tft.height(), Camera.width(), Camera.height());
          //int camera_width = Camera.width();
          #if 1
          //byte swap
          for (int i = 0; i < numPixels; i++) pixels[i] = (pixels[i] >> 8) | (((pixels[i] & 0xff) << 8));

          if ((Camera.width() <= tft.width()) && (Camera.height() <= tft.height())) {
            if ((Camera.width() != tft.width()) || (Camera.height() != tft.height())) tft.fillScreen(BLACK);
            tft.writeRect(CENTER, CENTER, Camera.width(), Camera.height(), pixels);
          } else {
            Serial.println("sub image");
            tft.writeSubImageRect(0, 0, tft.width(), tft.height(), (Camera.width() - tft.width()) / 2, (Camera.height() - tft.height()),
                                  Camera.width(), Camera.height(), pixels);
          }
          #else
          Serial.println("sub image1");
          tft.writeSubImageRect(0, 0, tft.width(), tft.height(), 0, 0, Camera.width(), Camera.height(), pixels);
          #endif
          ch = ' ';
          g_continuous_flex_mode = false;
          break;
        }
      case 'F':
        {
          if (!g_continuous_flex_mode) {
            if (Camera.readContinuous(&camera_flexio_callback, frameBuffer, frameBuffer2)) {
              Serial.println("* continuous mode started");
              g_flexio_capture_count = 0;
              g_flexio_redraw_count = 0;
              g_continuous_flex_mode = true;
            } else {
              Serial.println("* error, could not start continuous mode");
            }
          } else {
            Camera.stopReadContinuous();
            g_continuous_flex_mode = false;
            Serial.println("* continuous mode stopped");
          }
          break;
        }
      case 'V':
        {
          if (!g_continuous_flex_mode) {
            if (Camera.readContinuous(&camera_flexio_callback_video, frameBuffer, frameBuffer2)) {
              Serial.println("Before Set frame complete CB");
              if (!tft.useFrameBuffer(true)) Serial.println("Failed call to useFrameBuffer");
              tft.setFrameCompleteCB(&frame_complete_cb, false);
              Serial.println("Before UPdateScreen Async");
              tft.updateScreenAsync(true);
              Serial.println("* continuous mode (Video) started");
              g_flexio_capture_count = 0;
              g_flexio_redraw_count = 0;
              g_continuous_flex_mode = 2;
            } else {
              Serial.println("* error, could not start continuous mode");
            }
          } else {
            Camera.stopReadContinuous();
            tft.endUpdateAsync();
            g_continuous_flex_mode = 0;
            Serial.println("* continuous mode stopped");
          }
          ch = ' ';
          break;
        }
      case '1':
        {
          tft.fillScreen(BLACK);
          break;
        }
      case 0x30:
        {
#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
          SerialUSB1.println(F("ACK CMD CAM start single shoot. END"));
          send_image(&SerialUSB1);
          SerialUSB1.println(F("READY. END"));
#else
          Serial.println("*** Only works in USB Dual or Triple Serial Mode ***");
#endif
          break;
        }
      case '?':
        {
          showCommandList();
          ch = ' ';
          break;
        }
      default:
        break;
    }
    while (Serial.read() != -1)
      ;  // lets strip the rest out
  }


  if (g_continuous_flex_mode) {
    if (g_new_flexio_data) {
      //Serial.println("new FlexIO data");
      tft.setOrigin(-2, -2);
      tft.writeRect8BPP(0, 0, FRAME_WIDTH, FRAME_HEIGHT, (uint8_t *)g_new_flexio_data, mono_palette);
      tft.setOrigin(0, 0);
      tft.updateScreenAsync();
      g_new_flexio_data = nullptr;
      g_flexio_redraw_count++;
      if (g_flexio_runtime > 10000) {
        // print some stats on actual speed, but not too much
        // printing too quickly to be considered "spew"
        float redraw_rate = (float)g_flexio_redraw_count / (float)g_flexio_runtime * 1000.0f;
        g_flexio_runtime = 0;
        g_flexio_redraw_count = 0;
        Serial.printf("redraw rate = %.2f Hz\n", redraw_rate);
      }
    }
  }
}

// Pass 8-bit (each) R,G,B, get back 16-bit packed color
uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_image(Stream *imgSerial) {
  memset((uint8_t *)pixels, 0, sizeof(pixels));
  Camera.readFrame(pixels);

  imgSerial->write(0xFF);
  imgSerial->write(0xAA);

  imgSerial->write((const uint8_t *)&bmp_header, sizeof(bmp_header));

  uint32_t idx = 0;
  for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
    idx = i * 2;
    imgSerial->write((pixels[i] >> 8) & 0xFF);
    imgSerial->write( (pixels[i]) & 0xFF );
    delayMicroseconds(8);
  }
  imgSerial->write(0xBB);
  imgSerial->write(0xCC);

  imgSerial->println(F("ACK CMD CAM Capture Done. END"));
  delay(50);
}

//#if defined(USB_DUAL_SERIAL) || defined(USB_TRIPLE_SERIAL)
void send_raw() {
  memset((uint8_t *)frameBuffer, 0, sizeof(frameBuffer));
  Camera.readFrame(pixels);
  uint32_t idx = 0;
  for (int i = 0; i < FRAME_HEIGHT * FRAME_WIDTH; i++) {
    idx = i * 2;
    SerialUSB1.write((pixels[i] >> 8) & 0xFF);
    SerialUSB1.write( (pixels[i]) & 0xFF );
  }
  
}
#endif


void showCommandList() {
  Serial.println("Send the 'f' character to read a frame using FlexIO (changes hardware setup!)");
  Serial.println("Send the 'F' to start/stop continuous using FlexIO (changes hardware setup!)");
  Serial.println("Send the 'V' character DMA to TFT async continueous  ...");
  Serial.println("Send the 'p' character to snapshot to PC on USB1");
  Serial.println("Send the '1' character to blank the display");
  Serial.println("Send the 'z' character to send current screen BMP to SD");
  Serial.println();
}
