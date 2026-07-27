/*
  ================================================================================
  MODULE: 0.96" 128x64 SPI OLED Display (Yellow & Blue Bicolor SSD1306)
  PLATFORM: STM32 (e.g. STM32F103C8T6 Blue Pill)
  ENVIRONMENT: PlatformIO (VS Code Extension) / Arduino IDE
  INTERFACE: 4-Wire Hardware SPI Mode
  ================================================================================
  
  [1] PLATFORMIO CONFIGURATION (platformio.ini):
  ---------------------------------------------
  Environment: [env:oled_spi_test]
  Platform: ststm32
  Board: bluepill_f103c8
  Framework: arduino

  Libraries (Auto-managed via platformio.ini):
  - adafruit/Adafruit SSD1306 @ ^2.5.7
  - adafruit/Adafruit GFX Library @ ^1.11.5

  [2] HARDWARE WIRING (SPI Mode):
  -------------------------------
  +--------------------+----------------------------------+
  | OLED Module Pin    | STM32 Board Pin                  |
  +--------------------+----------------------------------+
  | GND                | GND                              |
  | VCC                | 3.3V                             |
  | D0 / CLK / SCLK    | PA5 (SPI1_SCK)                   |
  | D1 / DIN / MOSI    | PA7 (SPI1_MOSI)                  |
  | RES / RST          | PA2 (Reset Pin)                  |
  | DC / D/C           | PA3 (Data / Command Selection)   |
  | CS                 | PA4 (Chip Select)                |
  +--------------------+----------------------------------+

  [3] SCREEN LAYOUT (Yellow-Blue Dual-Color OLED):
  -------------------------------------------------
  - Rows 0 to 15 (Top 16px): YELLOW Section (Header/Status)
  - Rows 16 to 63 (Bottom 48px): BLUE Section (Data/Graphs)

  ================================================================================
*/

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Hardware SPI Pin Definitions for STM32
#define OLED_MOSI PA7  // D1 / DIN
#define OLED_CLK  PA5  // D0 / CLK
#define OLED_CS   PA4  // CS
#define OLED_DC   PA3  // DC
#define OLED_RESET PA2 // RES / RST

// Create SSD1306 display instance using Hardware SPI
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &SPI, OLED_DC, OLED_RESET, OLED_CS);

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000);

  Serial.println(F("\n==========================================="));
  Serial.println(F("  STM32 + 0.96\" SPI OLED Display Test     "));
  Serial.println(F("==========================================="));

  // Initialize SSD1306 OLED Display with SPI
  if (!display.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("[ERROR] SSD1306 OLED initialization failed!"));
    Serial.println(F("Check wiring: SCK=PA5, MOSI=PA7, CS=PA4, DC=PA3, RES=PA2, VCC=3.3V"));
    while (1) {
      delay(500);
    }
  }

  Serial.println(F("[SUCCESS] OLED Display Initialized!"));

  // Clear initial display buffer
  display.clearDisplay();
  display.display();
  delay(500);
}

void loop() {
  static uint32_t counter = 0;

  display.clearDisplay();

  // -------------------------------------------------------------
  // TOP SECTION (Rows 0-15: YELLOW AREA)
  // -------------------------------------------------------------
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 2);
  display.print(F("MAZE-CYPHER ROBOT"));
  
  // Horizontal dividing line between Yellow and Blue zones
  display.drawFastHLine(0, 15, 128, SSD1306_WHITE);

  // -------------------------------------------------------------
  // BOTTOM SECTION (Rows 16-63: BLUE AREA)
  // -------------------------------------------------------------
  display.setCursor(0, 20);
  display.print(F("Status: RUNNING"));

  display.setCursor(0, 32);
  display.print(F("Counter: "));
  display.setTextSize(2);
  display.setCursor(55, 30);
  display.print(counter);

  // Draw animated progress bar
  int barWidth = (counter % 100) * 124 / 100;
  display.drawRect(0, 52, 128, 10, SSD1306_WHITE);
  display.fillRect(2, 54, barWidth, 6, SSD1306_WHITE);

  // Render buffer to physical OLED screen
  display.display();

  counter++;
  delay(100);
}
