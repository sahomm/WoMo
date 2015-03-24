
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

//GPS
#include <SoftwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;
SoftwareSerial serialgps(0,1); //GPS_TX Pin 2, GPS_RX Pin 3
int year;
byte month, day, hour, minute, second, hundredths;
unsigned long chars;
unsigned short sentences, failed_checksum;

//debug on/off
boolean debugPWR = true; //debug Anteil Strom- Spannungsmessung ON OFF
boolean debugDIM = false; //debug Dimmer Display ON OFF

//Eingangspin
int pinSpannung = A0; //AnalogIn Spannung
int pinStrom = A1; //AnalogIn Strom     
int pinLCDDim = 6; //DigitalOut DimmLCD
int btnLCDWakeUp = 7; //DigitalIn zum einschalten LCD
//float aryStrom[10];
float strom;
int count = 0; // Counter zu zaehlen der Durchlaefe


//LCD-Helligkeit
int dimDefault = 150; //Default Helligkeit 0-255
int dim = dimDefault; //Variable zum setzen des Dim-Werts
long previousMillis = 0; // speichert die Zeit wann die Helligkeit das letzte mal geändert wurde
long LCDDimInterval = 2; // Zeit in Minuten wann das Display abgedunkelt werden soll

/*
// Funktion zum anpassen der helligkeit.
// @param int change   -  relative Aenderung des aktuellen Wertes
//
void set_backlight(int change) {
  if(debugDIM){Serial.print(" dim= ")};
  if(debugDIM){Serial.println(dim)};
  //dim = dim + change;
  dim = change;
  //pruefen auf erlaubten wertebereich 0 - 255
  if(debugDIM){Serial.print(" dim + chnage= ")};
  if(debugDIM){Serial.println(dim)};
  if ( dim < 0 ) dim =0;
  if (dim > 255 ) dim = 255;
  if (debugDIM){Serial.print("setting dim = ")};
  if(debugDIM){Serial.println(dim)};
  //schreiben der Helligkeit
  analogWrite(pinLCDDim,dim); 
}
*/

void setup() {

  
  //analogReference(EXTERNAL);
  
  //LCD initialiesieren
  lcd.begin(16, 2);
  analogWrite(pinLCDDim, dimDefault); //Displayhelligkeit auf Defaultwert setzen
  
  pinMode(btnLCDWakeUp, INPUT);
  
  //Serielle Schnitstelle für Ausgabe öffnen
  if(debugPWR || debugDIM){Serial.begin(115200);}    
  if(debugPWR || debugDIM){Serial.println("Get Strom & Spannung");} 
  //Serial.begin(9660);
  
  //GPS
  serialgps.begin(4800);
  Serial.println("");
  Serial.println("GPS Shield QuickStart Example Sketch v12");
  Serial.println(" ...waiting for lock... ");
  Serial.println("");
}

void loop() {
  //Counter zur Auswertung wieviele Durchlaeufe berets erfogt sind
  //count=count++;
  
  //aktuelle Laufzeit fuer Dimmer LCD
  unsigned long currentMillis = millis()/1000/60; //aktuelle Laufzeit
  
  //Button LCDWakeUp abfragen Displaybelaechtung ON
  if (digitalRead(btnLCDWakeUp)) {     
    // turn LCD on:    
    analogWrite(pinLCDDim, dimDefault);
    dim = dimDefault; // aktuellen Dimmwert setzen
    if(debugDIM){Serial.print("Wakeupbutton pressed > ");}
    if(debugDIM){Serial.print("Set Dim to:");}
    if(debugDIM){Serial.println(dimDefault);}
    previousMillis = currentMillis; // Timer bis fade off neu setzen
  } 
  
  //Spannung
  lcd.setCursor(0, 0);
  
/*
  const float Batt_div = 1024.0 / 5.00 / (10000+5000) * 5000.0;
  if(debugPWR){Serial.print(analogRead(pinSpannung));}
  if(debugPWR){Serial.print(" ");}
  if(debugPWR){Serial.print(analogRead(pinSpannung)/Batt_div);}
  if(debugPWR){Serial.print("V ");}*/
  
  float spannung= readSpannung(100);
  lcd.print(spannung);
  if(spannung<10){
    lcd.print(" V");
  }else{
    lcd.print("V");
  }
  //Strom
  

  //NEW
  //richtige Ausgabe
  /*
  strom = (ACS712read(100));
  lcd.setCursor(10, 0);
  lcd.print((float)strom);
  if(debugPWR){Serial.print(analogRead(pinStrom));}
  if(debugPWR){Serial.print(" ");}
  if(debugPWR){Serial.println(strom);}
  Serial.println(readVccArduino());
  //Serial.println(readACS714());
  //endNEW
  //LCD-Zelle neu schreiben wenn kein Vorzeichen Minus
  if (strom >=0){
    lcd.setCursor(14,0);
    lcd.print(" ");
  }
  
  lcd.setCursor(15,0);
  lcd.print("A");
  */
  //test Ausgabe
  
  strom = (ACS712read5V(100));
  lcd.setCursor(9, 0);
  lcd.print("1:");
  lcd.print((float)strom);
  Serial.print("1:");
  Serial.print(strom);
  
  strom = (ACS712read33V(100));
  lcd.setCursor(0, 1);
  lcd.print("2:");
  lcd.print((float)strom);
  Serial.print(" 2:");
  Serial.print(strom); 

  strom = (readACS714());
  lcd.setCursor(9, 1);
  lcd.print("3:");
  lcd.print((float)strom);
  Serial.print(" 3:");
  Serial.println(strom);   
  
  
  //delay(1000);
  
  //Dimmen der Displayhelligkeit nach LCDDimInterval in Millisekunden
  if(debugDIM){Serial.print(millis());}
  if(debugDIM){Serial.print(" currentMillis:");}
  if(debugDIM){Serial.print(currentMillis);}
  if(debugDIM){Serial.print(" previousMillis:");}
  if(debugDIM){Serial.print(previousMillis);}
  if(debugDIM){Serial.print(" LCDDimInterval:");}
  if(debugDIM){Serial.print(LCDDimInterval);}
  if(debugDIM){Serial.print(" dim=:");}
  if(debugDIM){Serial.println(dim);}
  if(currentMillis - previousMillis >= LCDDimInterval && dim>0) {
    previousMillis = currentMillis;
    do{
      dim --;
      if(debugDIM){Serial.print("Set DIM to:");}
      if(debugDIM){Serial.println(dim);}
      analogWrite(pinLCDDim,dim);
      delay(10);
    }while(dim>0);
  }
  //GPS

  while(serialgps.available()) {
    int c = serialgps.read();
        
    while(gps.encode(c)) {
      float latitude, longitude;
      gps.f_get_position(&latitude, &longitude);
      Serial.print("Lat/Long: "); 
      Serial.print(latitude,5); 
      Serial.print(", "); 
      Serial.println(longitude,5);
      gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths);
      Serial.print("Date: "); Serial.print(month, DEC); Serial.print("/"); 
      Serial.print(day, DEC); Serial.print("/"); Serial.print(year);
      Serial.print(" Time: "); Serial.print(hour, DEC); Serial.print(":"); 
      Serial.print(minute, DEC); Serial.print(":"); Serial.print(second, DEC); 
      Serial.print("."); Serial.println(hundredths, DEC);
      Serial.print("Altitude (meters): "); Serial.println(gps.f_altitude()); 
      Serial.print("Course (degrees): "); Serial.println(gps.f_course()); 
      Serial.print("Speed(kmph): "); Serial.println(gps.f_speed_kmph());
      Serial.print("Satellites: "); Serial.println(gps.satellites());
      Serial.println();
      gps.stats(&chars, &sentences, &failed_checksum);
    }
  }
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
float sensdiff=90.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
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
  //Serial.println();
  //Serial.print(vcc,5);
  //Serial.print(" ");
  //Serial.println(analogRead(5));
  
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
float sensdiff=90.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
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
  //Serial.println();
  //Serial.print(vcc,5);
  //Serial.print(" ");
  //Serial.println(analogRead(5));
  
  return ampmittel/mitteln;
}

// anderer Versuch
float readACS714(){
  double AnalogIn = analogRead(1);  // Read  analog value
  int Sensitivity    = 66; // mV/A
 
  // If you use a "normal" Arduino
  long   InternalVcc    = readVccArduino();
  //Serial.println(InternalVcc);
  // If you use a Arduino Mega
  //long   InternalVcc    = readVccArduinoMega();
 
  double ZeroCurrentVcc = InternalVcc / 2;
  //Serial.println(ZeroCurrentVcc);
  float SensedVoltage  = (AnalogIn * InternalVcc) / 1024;
  //Serial.println(SensedVoltage);
  float Difference     = SensedVoltage - ZeroCurrentVcc;
  //Serial.println(Difference);
  float SensedCurrent  = Difference / Sensitivity;
 
  //Serial.print("Current: ");
  //Serial.println(SensedCurrent);
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
  float Batt_div, volt, voltmittel = 0;
  int i; 
  for(i=0;i< mitteln;i++) {
    //Batt_div = 1024.0 / 5.00 / (10000+5000) * 5000.0;
    //Batt_div = 1024.0 / 5.00 / (10+5) * 5000.0;
    Batt_div = 1024.0 / ((float) readVccArduino() /1000) / (10000+5000) * 5000.0;

    volt = analogRead(0)/Batt_div;
    voltmittel += volt;
  }
  
  if(debugPWR){Serial.print(analogRead(pinSpannung));}
  if(debugPWR){Serial.print(" ");}
  if(debugPWR){Serial.print(voltmittel / mitteln);}
  if(debugPWR){Serial.print("V ");}
  if(debugPWR){Serial.print(readVccArduino());}
  if(debugPWR){Serial.print("V ");}
  return voltmittel / mitteln;
}
