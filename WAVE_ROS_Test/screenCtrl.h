// <<<<<<<<<<=== === ===SSD1306: 0x3C=== === ===>>>>>>>>>>
// 0.91inch OLED
bool screenDefaultMode = true;
long lastScreenUpdateMillis;
int  screenUpdateInterval = 500;
int  setTimeOutSecs = 20;

String screenLine_0;
String screenLine_1;
String screenLine_2;
String screenLine_3;

#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH   128 // OLED display width, in pixels
#define SCREEN_HEIGHT  32 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);


void InitScreen(){
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.display();
  Serial.println("setup screen complete");
}


// Updata all data and flash the screen.
void allDataUpdate(){
  if(screenDefaultMode){
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    display.print(F("Serial:"));display.print("iets");
    
    display.display();
  }
  else{
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);

    display.print(F("MAC:"));display.println("joe");
    Serial.println("test");
    display.println(screenLine_0);
    display.println(screenLine_1);
    display.println(screenLine_2);
    display.println(screenLine_3);

    display.display();
  }
}