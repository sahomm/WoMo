//Nur fuer den YUN
//testGitHUB
//test2
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
int pin_volt_bat = A0; //AnalogIn Spannung Batterie
int pin_amp_bat = A1; //AnalogIn Strom Batterie
int pin_amp_solar = A2; //AnalogIn Strom Solar
float volt_bat, amp_bat, amp_solar, amp_bat_current, amphr_bat, amphr_solar, watthr_bat, watthr_solar, totalCharge, totalUsage;
long sample; //Abtastung zur Berechnung Ah

//Variablen TFT
int color_text = ILI9341_WHITE;
int color_background = ILI9341_BLACK;
int size_text = 2;
int pos_volt_bat_x = 200;
int pos_volt_bat_y = 50;
int pos_amp_bat_x = 200;
int pos_amp_bat_y = 70;
int pos_amp_solar_x = 200;
int pos_amp_solar_y = 90;
int pos_amp_bat_current_x = 200;
int pos_amp_bat_current_y = 110;
int pos_amphr_bat_x = 200;
int pos_amphr_bat_y = 130;
int pos_watthr_bat_x = 200;
int pos_watthr_bat_y = 150;
int pos_amphr_solar_x = 200;
int pos_amphr_solar_y = 170;
int pos_watthr_solar_x = 200;
int pos_watthr_solar_y = 190;

void setup() {
  //Nur fuer den YUN
  // initialize Console communication:
  Bridge.begin();
  Console.begin();
  Console.println("You're connected to the Console!!!!");
 
  tft.begin();
  
  //Refenzspannung
  analogReference(EXTERNAL);

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
  
  // TFT
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextWrap(true);
  
  // TFT write default Text
  tft.setCursor(5,10);
  tft.setTextColor(ILI9341_RED);
  tft.setTextSize(3);
  tft.print("MobileStatus v0.1");
  for(int i=0;i< tft.width();i++){
    tft.drawFastHLine(i, 42, 3, ILI9341_RED);
  }
  tft.setTextColor(color_text);
  tft.setTextSize(size_text);
  tft.setCursor(10,pos_volt_bat_y);
  tft.print("Batterie Volt: ");
  tft.setCursor(10,pos_amp_bat_y);
  tft.print("Batterie Amp.: ");
  tft.setCursor(10,pos_amp_solar_y);
  tft.print("Solar IN: ");
  tft.setCursor(10,pos_amp_bat_current_y);
  tft.print("IN/OUT Akt.: ");
  tft.setCursor(10,pos_amphr_bat_y);
  tft.print("IN/OUT Ah: ");
  tft.setCursor(10,pos_watthr_bat_y);
  tft.print("IN/OUT Wh: ");
  tft.setCursor(10,pos_amphr_solar_y);
  tft.print("SolarIn Ah: ");
  tft.setCursor(10,pos_watthr_solar_y);
  tft.print("SolarIn Wh: ");
  
}

void loop() {
  //tft.println("beginLoop");
  //Spannung
  float temp= readVoltBat(100);
  if((int)(temp*100)!=(int)(volt_bat*100)){
    // Alten Werten mit color_background überschreiben (vom Display löschen)
    show_value(volt_bat,color_background, pos_volt_bat_x, pos_volt_bat_y);
    // Neuen Wert anzeigen
    show_value(temp,color_text, pos_volt_bat_x, pos_volt_bat_y);
    volt_bat = temp; 
  }  
  
  //Amp Batterie
  temp =  readAmpBat(200);
  if((int)(temp*100)!=(int)(amp_bat*100)){
    show_value(amp_bat,color_background, pos_amp_bat_x, pos_amp_bat_y);
    // Neuen Wert anzeigen
    show_value(temp,color_text, pos_amp_bat_x, pos_amp_bat_y);
    amp_bat = temp;
  }
  
  //Amp Solar
  //temp = analogRead(A2);
  temp =  readAmpSolar(100);
  if((int)(temp*100)!=(int)(amp_solar*100)){
    show_value(amp_solar,color_background, pos_amp_solar_x, pos_amp_solar_y);
    // Neuen Wert anzeigen
    show_value(temp,color_text, pos_amp_solar_x, pos_amp_solar_y);
    amp_solar = temp;
  }
  
  //aktuelle Entnahme Battrie
  temp =  (float)amp_bat - (float)amp_solar;
  if((int)(temp*100)!=(int)(amp_bat_current*100)){
      show_value(amp_bat_current,color_background, pos_amp_bat_current_x, pos_amp_bat_current_y);
    // Neuen Wert anzeigen
    show_value(temp,color_text, pos_amp_bat_current_x, pos_amp_bat_current_y);
    amp_bat_current = temp;
  }
  

  //Ah Entnahme Battrie
  temp =  calcAhBat(volt_bat, amp_bat_current);
  if((int)(temp*100)!=(int)(amphr_bat*100)){
    show_value(amphr_bat,color_background, pos_amphr_bat_x, pos_amphr_bat_y);
    show_value(watthr_bat,color_background, pos_watthr_bat_x, pos_watthr_bat_y);
    // Neuen Wert anzeigen
    watthr_bat = temp*volt_bat;
    show_value(temp,color_text, pos_amphr_bat_x, pos_amphr_bat_y);
    show_value(watthr_bat,color_text, pos_watthr_bat_x, pos_watthr_bat_y);
    amphr_bat = temp;
  }
  
  //Ah Ertrag Solar
  temp =  calcAhSolar(volt_bat, amp_solar);
  if((int)(temp*100)!=(int)(amphr_solar*100)){
    show_value(amphr_solar,color_background, pos_amphr_solar_x, pos_amphr_solar_y);
    show_value(watthr_solar,color_background, pos_watthr_solar_x, pos_watthr_solar_y);
    // Neuen Wert anzeigen
    watthr_solar = temp*volt_bat;
    show_value(temp,color_text, pos_amphr_solar_x, pos_amphr_solar_y);
    show_value(watthr_solar,color_text, pos_watthr_solar_x, pos_watthr_solar_y);
    amphr_solar = temp;
  } 
  
  delay(1000);
  
}

float readAmpBat(int mitteln) {
 
  float sense=66.0;           // mV/A Datenblatt Seite 2
  //float sensdiff=0.01;         // sense nimmt mit ca. sensdiff/V Vcc ab.
  int sensdiff=15;         // sense nimmt mit ca. sensdiff/V Vcc ab.
  float vcc, vsensor, amp, ampmittel=0;
  int i;
  
  for(i=0;i< mitteln;i++) {
    vcc = (float) 3.30 / analogRead(5) * 1024.0;    // Versorgungsspannung ermitteln
    vsensor = (float) analogRead(1) * vcc / 1024.0; // Messwert auslesen
    vsensor = (float) vsensor - (vcc/2);            // Nulldurchgang (vcc/2) abziehen
    sense = (float) 66.0 - ((5.00-vcc)*sensdiff);  // sense für Vcc korrigieren
    amp = (float) vsensor /sense *1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }
  //Console.println();
  //Console.print(vcc,5);
  //Console.print(" ");
  //Console.println(analogRead(5));
  //Console.println();
  if(debugPWR){Console.print("AmpBat: ");}
  if(debugPWR){Console.print(vcc,5);}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(analogRead(pin_amp_bat));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(ampmittel/mitteln);}
  if(debugPWR){Console.println(" A");}
  return ampmittel/mitteln;
}


float readAmpSolar(int mitteln){
  
  float vcc, vsensor, amp, ampmittel=0;
  int i;
  
  for(i=0;i< mitteln;i++) {
    vcc = (float) 3300 / analogRead(5) * 1024;    // Versorgungsspannung ermitteln
    //vsensor = (((long)analogRead(A2) * 5000 / 1024) - 500 ) * 1000 / 133; // Messwert auslesen
    vsensor = (((long)analogRead(A2) * vcc / 1024) - (vcc/10) ) * 1000 / 133; // Messwert auslesen
    //vsensor = (((long)analogRead(A2) * vcc / 1024) - 420 ) * 1000 / 133; // Messwert auslesen
    amp = (float) vsensor /1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }
  if(debugPWR){Console.print("AmpSolar: ");}
  if(debugPWR){Console.print(vcc);}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(analogRead(pin_amp_solar));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(ampmittel/mitteln);}
  if(debugPWR){Console.println(" A");}
  
  return ampmittel/mitteln;
}

float readVoltBat(int mitteln){
  
  /*info
    for(i=0;i< mitteln;i++) {
    vcc = (float) 3.30 / analogRead(5) * 1024.0;    // Versorgungsspannung ermitteln
    vsensor = (float) analogRead(1) * vcc / 1024.0; // Messwert auslesen
    vsensor = (float) vsensor - (vcc/2);            // Nulldurchgang (vcc/2) abziehen
    sense = (float) 66.0 - ((5.00-vcc)*sensdiff);  // sense für Vcc korrigieren
    amp = (float) vsensor /sense *1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }*/
  float Batt_div, volt, voltmittel=0, vcc, sensdiff;
  int i;
  // sensdiff = 0.6;  //korrekturwert WoMo
  sensdiff = -0.5; //Korrekturwert Labor
  for(i=0;i< mitteln;i++) {
    vcc = (float) 3.30 / analogRead(5) * 1024.0;    // Versorgungsspannung ermitteln
    Batt_div = 1024.0 / vcc / (10000+4700) * 4700.0;
    //Batt_div = 1023.0 / 5.0 / (10000+5000) * 5000.0;
    //Batt_div = 1024.0 / 5.00 / (10+5) * 5.0;
    //Batt_div = 1023.0 / ((float) readVccArduino() /1000) / (10000+5000) * 5000.0;
    Batt_div = Batt_div + sensdiff;
    volt = analogRead(0)/Batt_div;
    voltmittel += volt;
  }
  if(debugPWR){Console.println("_____________________________________");}   
  if(debugPWR){Console.print("Spannung: ");}  
  if(debugPWR){Console.print(vcc,5);}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(analogRead(pin_volt_bat));}
  if(debugPWR){Console.print(" ");}
  if(analogRead(0)==1023){
    if(debugPWR){Console.print("99,99");}
  }else{
    if(debugPWR){Console.print(voltmittel / mitteln);}
  }
  //if(debugPWR){Console.print(voltmittel / mitteln);}
  //if(debugPWR){Console.print(volt);}
  //if(debugPWR){Console.print("V ");}
  //if(debugPWR){Console.print((float)readVccArduino());}
  if(debugPWR){Console.println("V ");}
  
  if(analogRead(0)==1023){
    return 99,99;
  }else{
    return voltmittel / mitteln;
  }
  //return volt;
}

float readVoltBatBACKUP(int mitteln){
  float Batt_div, volt, voltmittel = 0, sensdiff;
  int i;
  // sensdiff = 0.6;  //korrekturwert WoMo
  sensdiff = 0.2; //Korrekturwert Labor
  for(i=0;i< mitteln;i++) {
    Batt_div = 1023.0 / 5.0 / (10000+5000) * 5000.0;
    //Batt_div = 1024.0 / 5.00 / (10+5) * 5.0;
    //Batt_div = 1023.0 / ((float) readVccArduino() /1000) / (10000+5000) * 5000.0;
    Batt_div = Batt_div + sensdiff;
    volt = analogRead(0)/Batt_div;
    voltmittel += volt;
  }
  
  if(debugPWR){Console.print(analogRead(pin_volt_bat));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(voltmittel / mitteln);}
  //if(debugPWR){Console.print(volt);}
  if(debugPWR){Console.print("V ");}
  //if(debugPWR){Console.print((float)readVccArduino());}
  if(debugPWR){Console.println("V ");}
  return voltmittel / mitteln;
  //return volt;
}

float calcAhBat (float voltbat, float amp){
  float watts = (float)voltbat * (float)amp;
  sample = sample + 1;
  float msec = millis();
  float time = (float)msec /1000;
  totalUsage = totalUsage + amp;
  float averageAmp = totalUsage/sample;
  float ampSec = averageAmp*time;
  return (float)ampSec/3600;  
}

float calcAhSolar (float voltbat, float amp){
  float watts = (float)voltbat * (float)amp;
  sample = sample + 1;
  float msec = millis();
  float time = (float)msec /1000;
  totalCharge = totalCharge + amp;
  float averageAmp = totalCharge/sample;
  float ampSec = averageAmp*time;
  return (float)ampSec/3600;  
}

void show_value (float value,int color, int x, int y){
  tft.setCursor(x,y);
  tft.setTextColor(color);
  if(value!=0){
    tft.print(value);
  }
  else tft.print("----");
}

