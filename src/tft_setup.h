//###############################################################################//
//WizPlug with 1.44", 128 x 128, ST7735 LCD

#define ST7735_DRIVER      // Define additional parameters below for this display

#define TFT_RGB_ORDER TFT_BGR  // Colour order Blue-Green-Red

#define TFT_WIDTH  128
#define TFT_HEIGHT 128

#define ST7735_GREENTAB3

#define TFT_CS     7   // Chip select control pin
#define TFT_DC     11  // Data Command control pin (RS pin)
#define TFT_RST    10  // Reset pin (could connect to NodeMCU RST, see next line)
#define TFT_MOSI   15
#define TFT_MISO   12
#define TFT_SCLK   14

#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH, 96 characters
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH, 96 characters
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH, only characters 1234567890:-.apm
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH, only characters 1234567890:-.
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH, only characters 1234567890:-.
#define LOAD_GFXFF  // FreeFonts. Include access to the 48 Adafruit_GFX free fonts FF1 to FF48 and custom fonts

#define SMOOTH_FONT

// For the RP2040 processor define the SPI port channel used (default 0 if undefined)
#define TFT_SPI_PORT 1 // Set to 0 if SPI0 pins are used, or 1 if spi1 pins used

#define SPI_FREQUENCY  27000000
#define SPI_READ_FREQUENCY  20000000
// #define SPI_TOUCH_FREQUENCY  2500000
