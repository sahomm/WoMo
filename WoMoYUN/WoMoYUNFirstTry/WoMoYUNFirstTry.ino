//Nur fuer den YUN
#include <Console.h>
 
#include <LiquidCrystal.h>
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);

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
long LCDDimInterval = 10000; // Zeit in Minuten wann das Display abgedunkelt werden soll

/*
// Funktion zum anpassen der helligkeit.
// @param int change   -  relative Aenderung des aktuellen Wertes
//
void set_backlight(int change) {
  if(debugDIM){Console.print(" dim= ")};
  if(debugDIM){Console.println(dim)};
  //dim = dim + change;
  dim = change;
  //pruefen auf erlaubten wertebereich 0 - 255
  if(debugDIM){Console.print(" dim + chnage= ")};
  if(debugDIM){Console.println(dim)};
  if ( dim < 0 ) dim =0;
  if (dim > 255 ) dim = 255;
  if (debugDIM){Console.print("setting dim = ")};
  if(debugDIM){Console.println(dim)};
  //schreiben der Helligkeit
  analogWrite(pinLCDDim,dim); 
}
*/

void setup() {
  //Nur fuer den YUN
  // initialize Console communication:
  Bridge.begin();
  Console.begin();
  Console.println("You're connected to the Console!!!!");
  
  //analogReference(EXTERNAL);
  
  //LCD initialiesieren
  lcd.begin(16, 2);
  analogWrite(pinLCDDim, dimDefault); //Displayhelligkeit auf Defaultwert setzen
  
  pinMode(btnLCDWakeUp, INPUT);
  
  //Serielle Schnitstelle für Ausgabe öffnen
  //if(debugPWR || debugDIM){Console.begin(9600);}    
  if(debugPWR || debugDIM){Console.println("Get Strom & Spannung");} 
  //Console.begin(9660);
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
    if(debugDIM){Console.print("Wakeupbutton pressed > ");}
    if(debugDIM){Console.print("Set Dim to:");}
    if(debugDIM){Console.println(dimDefault);}
    previousMillis = currentMillis; // Timer bis fade off neu setzen
  } 

  //Spannung
  lcd.setCursor(0, 0);
  const float Batt_div = 1024.0 / 5.00 / (10000+5000) * 5000.0;
  if(debugPWR){Console.print(analogRead(pinSpannung));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.print(analogRead(pinSpannung)/Batt_div);}
  if(debugPWR){Console.print("V ");}
  lcd.print(analogRead(pinSpannung)/Batt_div);
  if(analogRead(pinSpannung)/Batt_div<10){
    lcd.print(" V");
  }else{
    lcd.print("V");
  }
  //Strom
  
  //OLD
  /*
  float mitStrom = 0;
  for(byte i = 0; i < 10; i++){
    mitStrom += aryStrom[i];
    Console.println(aryStrom[i]);
  }
  
  if(mitStrom!=0){
    mitStrom= mitStrom/10;
  }
  if(debugPWR){Console.print(analogRead(pinStrom));}
  if(debugPWR){Console.print(" ");}
  //if(debugPWR){Console.print((float)0.066*(analogRead(pinStrom)-515));}
  if(debugPWR){Console.print((float)mitStrom);}
  if(debugPWR){Console.println("A");}
  lcd.setCursor(10, 0);  
  //lcd.print(float(30/512*(analogRead(pinStrom)-512)));
  //lcd.print(float(0.066*(analogRead(pinStrom)-512)));
  lcd.print(mitStrom);
  
  if (count < (10-1)){
    //Console.println(count);
    count = ++count;
  }
  else{
    //Console.println(count);
    count = 0;
    //Console.println(count);
  }
  //Console.println(count);
  //aryStrom[count]=(float)0.066*(analogRead(pinStrom)-512);
  //aryStrom[count]=(float)30/512*(analogRead(pinStrom)-512);
  */
  //endOLD

  //NEW
  //richtige Ausgabe
  /*
  strom = (ACS712read(100));
  lcd.setCursor(10, 0);
  lcd.print((float)strom);
  if(debugPWR){Console.print(analogRead(pinStrom));}
  if(debugPWR){Console.print(" ");}
  if(debugPWR){Console.println(strom);}
  Console.println(readVccArduino());
  //Console.println(readACS714());
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
  Console.print("1:");
  Console.print(strom);
  
  /*
  strom = (ACS712read33V(100));
  lcd.setCursor(0, 1);
  lcd.print("2:");
  lcd.print((float)strom);
  Console.print(" 2:");
  Console.print(strom); 

  strom = (readACS714());
  lcd.setCursor(9, 1);
  lcd.print("3:");
  lcd.print((float)strom);
  Console.print(" 3:");
  Console.println(strom);   
  */
  
  delay(1000);
  
  //Dimmen der Displayhelligkeit nach LCDDimInterval in Millisekunden
  if(debugDIM){Console.print(millis());}
  if(debugDIM){Console.print(" currentMillis:");}
  if(debugDIM){Console.print(currentMillis);}
  if(debugDIM){Console.print(" previousMillis:");}
  if(debugDIM){Console.print(previousMillis);}
  if(debugDIM){Console.print(" LCDDimInterval:");}
  if(debugDIM){Console.print(LCDDimInterval);}
  if(debugDIM){Console.print(" dim=:");}
  if(debugDIM){Console.println(dim);}
  if(currentMillis - previousMillis >= LCDDimInterval && dim>0) {
    previousMillis = currentMillis;
    do{
      dim --;
      if(debugDIM){Console.print("Set DIM to:");}
      if(debugDIM){Console.println(dim);}
      analogWrite(pinLCDDim,dim);
      delay(10);
    }while(dim>0);
  }
  //Nur fuer den YUN
  // Ensure the last bit of data is sent.
  Console.flush();

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
float sensdiff=0.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
float vcc, vsensor, amp, ampmittel=0;
int i;
  
  for(i=0;i< mitteln;i++) {
    //vcc = (float) 3.30 / analogRead(5) * 1023.0;    // Versorgungsspannung ermitteln
    //vcc = (float)readVccArduino()/1000;
    //vcc = 5.0;
    vsensor = (float) analogRead(1) * vcc / 1023.0; // Messwert auslesen
    vsensor = (float) vsensor - (vcc/2);            // Nulldurchgang (vcc/2) abziehen
    sense = (float) 66.0 - ((5.00-vcc)*sensdiff);  // sense für Vcc korrigieren
    amp = (float) vsensor /sense *1000 ;            // Ampere berechnen
    ampmittel += amp;                               // Summieren
  }
  Console.println();
  Console.print("5V vcc:");
  Console.print(vcc,5);
  Console.print(" Analog 1:");
  Console.println(analogRead(1));
  
  return ampmittel/mitteln;
}
/*
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
float sensdiff=0.0;         // sense nimmt mit ca. sensdiff/V Vcc ab.
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
  Console.println();
  Console.print("3,3V vcc:");
  Console.print(vcc,5);
  Console.print(" Analog 1:");
  Console.println(analogRead(1));
  Console.print(" Analog 5:");
  Console.println(analogRead(5));
  
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

*/
