//Nur fuer den YUN
#include <Console.h>

#include "SPI.h"
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

// For the Adafruit shield, these are the default.
#define TFT_DC 9
#define TFT_CS 10

// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);
// If using the breakout, change pins as desired
//Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_MOSI, TFT_CLK, TFT_RST, TFT_MISO);

//debug on/off
boolean debugPWR = true; //debug Anteil Strom- Spannungsmessung ON OFF
boolean debugDIM = false; //debug Dimmer Display ON OFF

//Eingangspin
int pinSpannung = A0; //AnalogIn Spannung
int pinStrom = A1; //AnalogIn Strom     
float strom;

void setup() {
  //Nur fuer den YUN
  // initialize Console communication:
  Bridge.begin();
  Console.begin();
  Console.println("You're connected to the Console!!!!");
 
  tft.begin();

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Console.print("Display Power Mode: 0x"); Console.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Console.print("MADCTL Mode: 0x"); Console.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Console.print("Pixel Format: 0x"); Console.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Console.print("Image Format: 0x"); Console.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Console.print("Self Diagnostic: 0x"); Console.println(x, HEX);
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(2);
  //tft.println("Hello World!");  
}

void loop() {
  //tft.println("beginLoop");
  
  //Spannung
  //lcd.setCursor(0, 0);
  //testText();
    
  float spannung= readSpannung(100);
  //lcd.print(spannung);
  if(spannung<10){
    //lcd.print(" V");
  }else{
    //lcd.print("V");
  }
  tft.fillScreen(ILI9341_BLACK);
  tft.setCursor(1, 1);  
  tft.println(spannung);
  

  //Strom
  float strom=ACS712read5V(100);
  tft.println(strom);
    
  strom = (ACS712read5V(100));
  //lcd.setCursor(9, 0);
  //lcd.print("1:");
  //lcd.print((float)strom);
  //Console.print("1:");
  //Console.print(strom);
  
  strom = (ACS712read33V(100));
  //lcd.setCursor(0, 1);
  //lcd.print("2:");
  //lcd.print((float)strom);
  //Console.print(" 2:");
  //Console.print(strom); 

  strom = (readACS714());
  //lcd.setCursor(9, 1);
  //lcd.print("3:");
  //lcd.print((float)strom);
  //Console.print(" 3:");
  //Console.println(strom);   
  
  
  delay(1000);
  
}

float ACS712read5V(int mitteln) {
// Den ACS712 Stromsensor auslesen
// Sens ist im Datenblatt auf Seite 2 mit 185 angegeben.
// Für meinen Sensor habe ich 186 ermittelt bei 5.0V Vcc.
// Sens nimmt mit ca. 38 pro Volt VCC ab.
//
// 3,3V muss zu Analog Eingang 5 gebrückt werden.
// Der Sensoreingang ist Analog 1
//
// Parameter mitteln : die Anzahl der Mittlungen
//
// Matthias Busse 9.5.2014 Version 1.0
 
float sense=66.0;           // mV/A Datenblatt Seite 2
float sensdiff=80.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
float vcc, vsensor, amp, ampmittel=0;
int i;
  
  for(i=0;i< mitteln;i++) {
    //vcc = (float) 3.30 / analogRead(5) * 1023.0;    // Versorgungsspannung ermitteln
    vcc = (float)readVccArduino()/1000;
    vsensor = (float) analogRead(1) * vcc / 1023.0; // Messwert auslesen
    vsensor = (float) vsensor - (vcc/2);            // Nulldurchgang (vcc/2) abziehen
    sense = (float) 66.0 - ((5.00-vcc)*sensdiff);  // sense für Vcc korrigieren
    amp = (float) vsensor /sense *1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }
  //Console.println();
  //Console.print(vcc,5);
  //Console.print(" ");
  //Console.println(analogRead(5));
  //Console.println (analogRead(1));
  
  return ampmittel/mitteln;
}

float ACS712read33V(int mitteln) {
// Den ACS712 Stromsensor auslesen
// Sens ist im Datenblatt auf Seite 2 mit 185 angegeben.
// Für meinen Sensor habe ich 186 ermittelt bei 5.0V Vcc.
// Sens nimmt mit ca. 38 pro Volt VCC ab.
//
// 3,3V muss zu Analog Eingang 5 gebrückt werden.
// Der Sensoreingang ist Analog 1
//
// Parameter mitteln : die Anzahl der Mittlungen
//
// Matthias Busse 9.5.2014 Version 1.0
 
float sense=66.0;           // mV/A Datenblatt Seite 2
float sensdiff=40.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
float vcc, vsensor, amp, ampmittel=0;
int i;
  
  for(i=0;i< mitteln;i++) {
    vcc = (float) 3.30 / analogRead(5) * 1023.0;    // Versorgungsspannung ermitteln
    vsensor = (float) analogRead(1) * vcc / 1023.0; // Messwert auslesen
    vsensor = (float) vsensor - (vcc/2);            // Nulldurchgang (vcc/2) abziehen
    sense = (float) 66.0 - ((5.00-vcc)*sensdiff);  // sense für Vcc korrigieren
    amp = (float) vsensor /sense *1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }
  //Console.println();
  //Console.print(vcc,5);
  //Console.print(" ");
  //Console.println(analogRead(5));
  
  return ampmittel/mitteln;
}

// anderer Versuch
float readACS714(){
  double AnalogIn = analogRead(1);  // Read  analog value
  int Sensitivity    = 66; // mV/A
 
  // If you use a "normal" Arduino
  long   InternalVcc    = readVccArduino();
  //Console.println(InternalVcc);
  // If you use a Arduino Mega
  //long   InternalVcc    = readVccArduinoMega();
 
  double ZeroCurrentVcc = InternalVcc / 2;
  //Console.println(ZeroCurrentVcc);
  float SensedVoltage  = (AnalogIn * InternalVcc) / 1024;
  //Console.println(SensedVoltage);
  float Difference     = SensedVoltage - ZeroCurrentVcc;
  //Console.println(Difference);
  float SensedCurrent  = Difference / Sensitivity;
 
  //Console.print("Current: ");
  //Console.println(SensedCurrent);
  return SensedCurrent;

}

long readVccArduino() {
  long result;
  // Read 1.1V reference against AVcc
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Convert
  while (bit_is_set(ADCSRA,ADSC));
  result = ADCL;
  result |= ADCH<<8;
  result = 1126400L / result; // Back-calculate AVcc in mV
  return result;
}

float readSpannung(int mitteln){
  float Batt_div, volt, voltmittel = 0, sensdiff;
  int i;
  sensdiff = 0.6;  //korrekturwert WoMo
  // sensdiff = 0.2; /Korrekturwert Labor
  for(i=0;i< mitteln;i++) {
    Batt_div = 1023.0 / 5.00 / (10000+5000) * 5000.0;
    //Batt_div = 1024.0 / 5.00 / (10+5) * 5.0;
    //Batt_div = 1023.0 / ((float) readVccArduino() /1000) / (10000+5000) * 5000.0;
    Batt_div = Batt_div + sensdiff;
    volt = analogRead(0)/Batt_div;
    voltmittel += volt;
  }
  
  if(debugPWR){Console.print(analogRead(pinSpannung));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(voltmittel / mitteln);}
  //if(debugPWR){Console.print(volt);}
  if(debugPWR){Console.print("V ");}
  //if(debugPWR){Console.print((float)readVccArduino());}
  if(debugPWR){Console.println("V ");}
  return voltmittel / mitteln;
  //return volt;
}

unsigned long testText() {
  tft.fillScreen(ILI9341_BLACK);
  unsigned long start = micros();
  tft.setCursor(0, 0);
  tft.setTextColor(ILI9341_WHITE);  tft.setTextSize(1);
  tft.println("Hello World!");
  tft.setTextColor(ILI9341_YELLOW); tft.setTextSize(2);
  tft.println(1234.56);
  tft.setTextColor(ILI9341_RED);    tft.setTextSize(3);
  tft.println(0xDEADBEEF, HEX);
  tft.println();
  tft.setTextColor(ILI9341_GREEN);
  tft.setTextSize(5);
  tft.println("Groop");
  tft.setTextSize(2);
  tft.println("I implore thee,");
  tft.setTextSize(1);
  tft.println("my foonting turlingdromes.");
  tft.println("And hooptiously drangle me");
  tft.println("with crinkly bindlewurdles,");
  tft.println("Or I will rend thee");
  tft.println("in the gobberwarts");
  tft.println("with my blurglecruncheon,");
  tft.println("see if I don't!");
  return micros() - start;
}

