
#include <Adafruit_MAX31865.h>  //Bibliothek um MAX31865 zu verwenden

// Verbindung zwischen Arduino und Max31865 über SPI-Bus
#define MAX_SCK 13    // Pin 13 (Arduino) ist die Schnittstelle mit SCK (MAX31865)
#define MAX_MISO 12   // Pin 12 (Arduino) ist die Schnittstelle mit MISO (MAX31865)
#define MAX_MOSI 11   // Pin 11 (Arduino) ist die Schnittstelle mit MOSI (MAX31865)
#define MAX_CS 10     // Pin 10 (Arduino) ist die Schnittstelle mit CS (MAX31865)

// Klasse Adafruit_Max31865 ermöglicht Kommunikation mit dem Modul MAX31865 über definierte Pins
Adafruit_MAX31865 max = Adafruit_MAX31865(MAX_CS, MAX_MOSI, MAX_MISO, MAX_SCK);

//Zeitvariablen
unsigned long zMillis = 0;          // Zwischenspeicher Variable für Millis-Funktion
const long interval = 10000;        // Intervall Variable für Millis-Funktion (10 sekunden)
unsigned long aktuelle_Zeit;        // Speicher-Variable für die aktuelle Zeit 
unsigned long vor_Micros = 0;       // Vorherige Zeit in Mikrosekunden

// Variablen für Temperatursteuerung
float temperatur;                   // Temperatur Variable in Datentyp float
const int ssr = 9;                  // Halbleiterrelais ist am Pin 9 festgelegt
float y;                            // Stellgröße Ausgangsignal für SSR
float w = 85.0;                     // Führungsgröße (Sollwert-Temperatur) 
float x;                            // Regelgröse (Istwert-Temperatur)
float e;                            // Regeldifferenzgröße e = w - x
float e_alt;                        // Regeldifferenzsgröße von Vorlauf
float integral = 0;                 // Summe der Fehler im Zeitverlauf (Startwert)
float integral_max;                 // Deklaration der Maximale Grenze für Integralanteil
float integral_min;                 // Deklaration der Minimale Grenze für Integralanteil 
float ableitung;                    // Differenzialanteil 

float yP, yI, yD;

// PID-Regler Parameter
  float Kp = 7.203;                 // Proportional Konstante 
  float Ki;                         // Integralanteil 
  float Kd;                         // Ableitungsteil
  float Tn = 723.805;               // Nachstellzeit
  float Tv = 41.498;                // Verstärkungszeit
  float Ts ;                        // Abtastzeit 

void setup() {
  
  Serial.begin(9600);               // Serielle Kommunikation mit einer Baudrahte 9600
  max.begin(MAX31865_3WIRE);        // Kommunikation mit 3-Draht PT100 Sensor über MAX31865
  pinMode(ssr, OUTPUT);             // Konfiguriere Pin 9 als Ausgang
  vor_Micros = micros();            // Initialize previousMicroseviousMicro
}

void loop() { 
  // Berechnen die Zeitdifferenz
  unsigned long aktuellMicros = micros();          // Speichert die aktuelle Zeit in Mikrosekunden seit dem Start des Arduino
  unsigned long zeitDifferenz = aktuellMicros - vor_Micros; // Berechnet die Zeitdifferenz zwischen der aktuellen und der vorherigen Zeit

  // Konvertirung der Zeit in sekunden 
  Ts = zeitDifferenz / 1e6;           // Konvertiert die Zeitdifferenz von Mikrosekunden in Sekunden 

  aktuelle_Zeit = millis();   // Aktuelle Zeit nach dem Start des Arduino 
  
  temperatur = max.temperature(100, 431); //Temperatureinlesung mit MAX31865 (Pt100), 431 Referenzwiderstand
  x = temperatur;

   // Auslesung und Ausgabe der Temperatur jeder  10 sek. 
  if ( aktuelle_Zeit - zMillis >= interval)  // Überprüft, ob 10 Sekunden vergangen sind
  {
     // Ausgabe der Temperatur auf Serielle Monitor
    Serial.print("Abtastzeit: "); Serial.print(Tvz);              // Gibt die Abtastzeit 'Tvz' auf dem seriellen Monitor aus
    Serial.print(" w(t): "); Serial.print(w);                     // Gibt den Sollwert 'w' (Führungsgröße) auf dem seriellen Monitor aus
    Serial.print(" x(t): "); Serial.print(x);                     // Gibt den Istwert 'x' (Regelgröße) auf dem seriellen Monitor aus
    Serial.print(" y(t): "); Serial.print(y);                     // Gibt die Stellgröße 'y' (Ausgangssignal für das SSR) auf dem seriellen Monitor aus
    Serial.print(" e(t): "); Serial.print(e);                     // Gibt die Regeldifferenz 'e' auf dem seriellen Monitor aus
    Serial.print(" Integral: "); Serial.print(integral);          // Gibt den Integralanteil des Fehlers auf dem seriellen Monitor aus
    Serial.print(" Ableitung "); Serial.print(ableitung);         // Gibt die Ableitung des Fehlers auf dem seriellen Monitor aus
    Serial.print(" yP: "); Serial.print(yP);                      // Gibt den Proportionalanteil 'yP' des Ausgangssignals auf dem seriellen Monitor aus
    Serial.print(" yI: "); Serial.print(yI);                      // Gibt den Integralanteil 'yI' des Ausgangssignals auf dem seriellen Monitor aus
    Serial.print(" yD: "); Serial.println(yD);                    // Gibt den Differenzialanteil 'yD' des Ausgangssignals auf dem seriellen Monitor aus 
    
    zMillis = aktuelle_Zeit;                                      // Zwischenvariable bekommt neuen Wert 
  }
  
  // PID Regelglied
  Ki = Kp / Tn;                                                   // Berechnung von Integralanteil Ki
  Kd = Kp * Tv;                                                   // Berechnung von Ableitungsanteil Kd
  integral_min = 0;                                               // Setzt die minimale Grenze für den Integralanteil
  integral_max = 10000;                                            // Setzt die maximale Grenze für den Integralanteil
  
 
  integral = constrain(integral, integral_min, integral_max);     // Begrenzt den Integralanteil zwischen den festgelegten Grenzen
  
  e = w - x ;                                                      // Berechnung von Regeldifferenzgröße
  ableitung = (e - e_alt) / Ts;                                    // Berechnung der Ableitung des Fehlers
  integral += e * Ts;                                                 // Aktuelle Fehler wird akkumuliert im Integralanteil

   yP = Kp * e;                                                   // Berechnung des Proportionalanteils
   yI = Ki * integral;                                            // Berechnung des Integralanteils
   yD = Kd * ableitung;                                           // Berechnung des Differenzialanteils

  // Berechnung der Stellgröße 
  y = yP + yI + yD;                                               // Gesamtausgangssignal des PID-Reglers
  y = constrain(y, 0, 255);                                       // Begrenzt die Stellgröße zwischen 0 und 255

  analogWrite(ssr, (int) y);                                      // Steuerung des SSR mit PWM
  vor_Micros = aktuellMicros;
  e_alt = e;
}
