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

      Teensy T4.x using ILI9341_t3n library.
      // T4
      #define OV7670_PLK   4
      #define OV7670_XCLK  5
      // Changed to GPIO/D order
      #define OV7670_D0    14 // AD_B1_02 1.18
      #define OV7670_D1    15 // AD_B1_03 1.19
      #define OV7670_D2    17 // AD_B1_06 1.22
      #define OV7670_D3    16 // AD_B1_07 1.23
      #define OV7670_D4    22 // AD_B1_08 1.24
      #define OV7670_D5    23 // AD_B1_09 1.25
      #define OV7670_D6    20 // AD_B1_10 1.26
      #define OV7670_D7    21 // AD_B1_11 1.27

      // Note These HREF should be on GPIO 1.x so probably 0 or 1 or bottom pins (24, 25, 26, 27)
      #define OV7670_HREF  40 // AD_B1_04 1.20 T4.1...
      #define OV7670_VSYNC 41 // AD_B1_05 1.21 T4.1...

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
  //      #define OV7670_D6    26 // AD_B1_14 1.30
  //      #define OV7670_D7    27 // AD_B1_15 1.31


  This example code is in the public domain.
*/
#define USE_ILI9488

#ifdef ARDUINO_TEENSY41
extern "C" {
  extern  uint8_t external_psram_size;
}
#endif

#include <Arduino_OV767X.h>
#ifdef USE_ILI9488
#include <ILI9488_t3.h>
uint16_t pixels[640 * 480] EXTMEM; // QCIF: 176x144 X 2 bytes per pixel (RGB565)
//uint16_t pixels[320 * 240]; // QCIF: 176x144 X 2 bytes per pixel (RGB565)
#define TFT_CS   0  // AD_B0_02
#define TFT_DC   1  // AD_B0_03
#define TFT_RST 255
#define RED   ILI9488_RED
#define GREEN ILI9488_GREEN
#define BLUE  ILI9488_BLUE
#define BLACK ILI9488_BLACK
#define CENTER ILI9488_t3::CENTER
ILI9488_t3 tft = ILI9488_t3(TFT_CS, TFT_DC);
//RAFB frame_buffer[320 * 480] EXTMEM;
#else
#include <ILI9341_t3n.h>
uint16_t pixels[320 * 240]; // QCIF: 176x144 X 2 bytes per pixel (RGB565)
//#define TFT_CS   0  // AD_B0_02
//#define TFT_DC   1  // AD_B0_03
//#define TFT_RST 255
#define TFT_CS   10  // AD_B0_02
#define TFT_DC  9   // AD_B0_03
#define TFT_RST 8//#define TFT_CS   0  // AD_B0_02
//#define TFT_DC   1  // AD_B0_03
//#define TFT_RST 255


#define RED   ILI9341_RED
#define GREEN ILI9341_GREEN
#define BLUE  ILI9341_BLUE
#define BLACK ILI9341_BLACK
#define CENTER ILI9341_t3n::CENTER
ILI9341_t3n tft = ILI9341_t3n(TFT_CS, TFT_DC, TFT_RST);

#endif
bool g_continuous_mode = false;
bool g_dma_mode = false;
elapsedMillis g_emCycles;
uint32_t      g_count_frames_output = 0;
const int pinCamReset = 14;

void setup() {
  while (!Serial && millis() < 4000) ;
  Serial.begin(9600);
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

#ifdef USE_ILI9488
  tft.begin(16000000);
  RAFB *frame_buffer = (RAFB*)extmem_malloc(sizeof(RAFB) * tft.width() * tft.height() + 32);
  Serial.printf("frame_buffer: %lx", (uint32_t)frame_buffer);
  frame_buffer =   (RAFB*)(((uint32_t)frame_buffer + 32) & 0xffffffe0);
  Serial.printf("frame_buffer aligned: %lx", (uint32_t)frame_buffer);

  tft.setFrameBuffer(frame_buffer);
#else
  tft.begin();
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



  Serial.println("OV767X Camera Capture"); Serial.flush();
  Serial.println();
#ifdef OV7670_XCLK_JUMPER
  pinMode(OV7670_XCLK_JUMPER, INPUT);
#endif
  if (!Camera.begin(VGA, RGB565, 5)) {
    Serial.println("Failed to initialize camera!");
    while (1);
  }

  Serial.println("Camera settings:");
  Serial.print("\twidth = ");
  Serial.println(Camera.width());
  Serial.print("\theight = ");
  Serial.println(Camera.height());
  Serial.print("\tbits per pixel = ");
  Serial.println(Camera.bitsPerPixel());
  Serial.println();

  Serial.println("Send the 'c' character to read a frame ...");
  Serial.println("Send the 'd' character to read a frame using DMA ...");
  Serial.println("Send the 's' character to start/stop continuous display mode");
  Serial.println("send the 'r' Show Camera register settings");
  Serial.println();
  pinMode(31, OUTPUT);
  digitalWrite(31, LOW);
  pinMode(32, OUTPUT);
  digitalWrite(32, LOW);
  pinMode(33, OUTPUT);
  digitalWrite(33, LOW);

}

uint16_t *last_dma_frame_buffer = nullptr;

void camera_dma_callback(void *pfb) {
  //Serial.printf("Callback: %x\n", (uint32_t)pfb);
  tft.writeRect(CENTER, CENTER, Camera.width(), Camera.height(), (uint16_t*)pfb);
  tft.updateScreenAsync();

  last_dma_frame_buffer = (uint16_t*)pfb;
}

void loop() {
  if (Serial.available()) {
    int ch = Serial.read();
    while (Serial.read() != -1); // get rid of the rest...
    switch (ch) {
      case 'r':
        Camera.showRegisters();
        break;
      case 'c':
        {
          Serial.println("Reading frame");
          memset((uint8_t*)pixels, 0, sizeof(pixels));
          Camera.readFrame(pixels);


          int numPixels = Camera.width() * Camera.height();
          int camera_width = Camera.width();

          for (int i = 0; i < numPixels; i++) pixels[i] = (pixels[i] >> 8) | (((pixels[i] & 0xff) << 8));

          tft.fillScreen(BLACK);

          if ((Camera.width() <= tft.width()) && (Camera.height() <= tft.height()))
            tft.writeRect(CENTER, CENTER, Camera.width(), Camera.height(), pixels);
          else
            tft.writeSubImageRect(0, 0, tft.width(), tft.height(),  (Camera.width() - tft.width()) / 2, (Camera.height() - tft.height()),
                                  Camera.width(), Camera.height(), pixels);
          for (int i = 0; i < numPixels; i++) {
            unsigned short p = pixels[i];

            if (p < 0x1000) {
              Serial.print('0');
            }

            if (p < 0x0100) {
              Serial.print('0');
            }

            if (p < 0x0010) {
              Serial.print('0');
            }

            Serial.print(p, HEX);
            if ((i % camera_width) == (camera_width - 1)) Serial.println();
          }
          g_continuous_mode = false;
          Serial.println();
          break;
        }
      case 'd':
        {
#if 0
          Serial.println("Reading DMA frame");
          Serial.println();
          memset((uint8_t*)pixels, 0, sizeof(pixels));
          Camera.readFrameDMA(pixels);


          int camera_width = Camera.width();

          tft.fillScreen(ILI9341_BLACK);
          tft.writeRect(ILI9341_t3n::CENTER, ILI9341_t3n::CENTER, Camera.width(), Camera.height(), pixels);

          uint16_t *p = pixels;
          for (int row = 0; row < Camera.height(); row++) {
            for (int col = 0; col < camera_width; col++) {
              Serial.printf("%04x", *p++);
            }
            Serial.println();
          }
          g_continuous_mode = false;
          Serial.println();
          break;
#else
          if (g_dma_mode) {
            Serial.println("*** stopReadFrameDMA ***");
            Camera.stopReadFrameDMA();
            Serial.println("*** return from stopReadFrameDMA ***");
            tft.endUpdateAsync();
            tft.useFrameBuffer(false);
            g_dma_mode = false;
          } else {
            Camera.startReadFrameDMA(&camera_dma_callback);
            Serial.println("*** Return from startReadFrameDMA ***");
            tft.useFrameBuffer(true);
            //        tft.updateScreenAsync(true);
            //        Serial.println("*** Return from updateScreenAsync ***");
            g_dma_mode = true;
          }
          break;
#endif
        }

      case 's':
        if (g_continuous_mode) {
          g_continuous_mode = false;
          Serial.println("*** Continuous mode turned off");
          tft.useFrameBuffer(false);
        } else {
          g_continuous_mode = true;
          tft.useFrameBuffer(true);
          Serial.println("*** Continuous mode turned on");
          g_emCycles = 0;
          g_count_frames_output = 0;
        }
        break;
    }
  }

  if (g_continuous_mode) {
    Camera.readFrame(pixels);
    int numPixels = Camera.width() * Camera.height();

    for (int i = 0; i < numPixels; i++) pixels[i] = (pixels[i] >> 8) | (((pixels[i] & 0xff) << 8));
    tft.waitUpdateAsyncComplete();  // wait while any other is pending...
    tft.fillScreen(BLACK);
    tft.writeRect(CENTER, CENTER, Camera.width(), Camera.height(), pixels);
    tft.updateScreenAsync(); // start an async update...
    g_count_frames_output++;
    if (g_emCycles >= 1000) {
      Serial.print("Cycles Per second: ");
      Serial.println((float)(g_count_frames_output * 1000.0) / (float)g_emCycles, 2);
      //    Serial.printf("Cycle time: %lu\n", (uint32_t)g_emCycles);
      g_emCycles = 0;
      g_count_frames_output = 0;
    }
  }
  if (g_dma_mode) {
    if (last_dma_frame_buffer) {
      //      tft.writeRect(ILI9341_t3n::CENTER, ILI9341_t3n::CENTER, Camera.width(), Camera.height(), last_dma_frame_buffer);
      last_dma_frame_buffer = nullptr;
    }
  }

}
