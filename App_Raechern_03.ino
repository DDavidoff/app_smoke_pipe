#include <SoftwareSerial.h>
#include <Adafruit_MAX31865.h> //Bibliothek um MAX31865 zu verwenden

// Verbindung zwischen Arduino und Max31865 über SPI-Bus
#define MAX_SCK 13              // Pin 13 (Arduino) ist die Schnittstelle mit SCK (MAX31865)
#define MAX_MISO 12             // Pin 12 (Arduino) ist die Schnittstelle mit MISO (MAX31865)
#define MAX_MOSI 11             // Pin 11 (Arduino) ist die Schnittstelle mit MOSI (MAX31865)
#define MAX_CS 10               // Pin 10 (Arduino) ist die Schnittstelle mit CS (MAX31865)

// Klasse Adafruit_Max31865 ermöglicht Kommunikation mit dem Modul MAX31865 über definierte Pins
Adafruit_MAX31865 max = Adafruit_MAX31865(MAX_CS, MAX_MOSI, MAX_MISO, MAX_SCK);

float temperatur; // Temperatur Variable in Datentyp float

// Zeitvariablen
unsigned long vor_Millis = 0;   // Zwischenspeicher Variable für Millis-Funktion
const long interval = 10000;    // Intervall Variable für Millis-Funktion 5s
unsigned long aktuelle_Zeit;    // Speicher-Variable für die aktuelle Zeit

// Lüfter 12V
const int enA = 5;              // PWM Pin für die Geschwindigkeitssteuerung des Lüfters
const int in1 = 6;              // Eingangspin 1 für die Lüftersteuerung
const int in2 = 7;              // Eingangspin 2 für die Lüftersteuerung
int fanSpeed = 0;               // Geschwindigkeit des Lüfters als Wert zwischen 0 und 255
int fanSpeedInPercent;          // Geschwindigkeit des Lüfters in Prozent (0-100)


// Temperatursteuerung von Räucherofen
const int ssr = 9;              // Halbleiterrelais ist am Pin 9 festgelegt
float y;                        // Stellgröße Ausgangsignal für SSR
float w;                        // Führungsgröße w(t) (Sollwert-Temperatur)
float x;                        // Regelgröse x(t) (Istwert-Temperatur)
float e;                        // Regeldifferenzgröße e(t) = w(t) - x(t)
float e_alt;                    // Regel differenzsgröße von Vorlauf
float integral = 0;             // Integral des Fehlers über die Zeit
float integral_max;             // Maximale Grenze für Integralanteil
float integral_min;             // Minimale Grenze für Integralanteil
float ableitung;                // Ableitung des Fehlers

float yP, yI, yD;               // Proportional-, Integral- und Differentialanteil des Ausgangssignals

// PID-Regler Parameter
float Kp = 7.203;               // Proportionalitätskonstante
float Ki;                       // Integral-Konstante
float Kd;                       // Differenzial-Konstante
float Tn = 723.805;             // Nachstellzeit
float Tv = 41.498;              // Verstärkungszeit
float Ts;                       // Abtastzeit von 1 s

// App Kommunikation
String incomingData = "";       // String für eingehende Daten von der App
String program;

unsigned long vor_Micros = 0;   // Vorherige Zeit in Mikrosekunden

SoftwareSerial mySerialBT(3, 2);// Initialisiert SoftwareSerial für Bluetooth-Kommunikation an Pins 3 (RX) und 2 (TX)

void setup()
{

  Serial.begin(9600);           // Serielle Kommunikation mit einer Baudrahte 9600
  mySerialBT.begin(9600);
  max.begin(MAX31865_3WIRE);    // Kommunikation mit 3-Draht PT100 Sensor über MAX31865
  pinMode(ssr, OUTPUT);         // Konfiguriere Pin 9 als Ausgang

  pinMode(enA, OUTPUT);         // Konfiguriere Pin 9 als Ausgang
  pinMode(in1, OUTPUT);         // Konfiguriere Pin 9 als Ausgang
  pinMode(in2, OUTPUT);

  vor_Micros = micros();        // Initialize previousMicroseviousMicro
}

bool deviceIsOn = false;

// Funktion, um das Gerät einzuschalten
void setDeviceOn()
{
  deviceIsOn = true;            // Setzt den Zustand des Geräts auf "true"
}

// Funktion, um das Gerät auszuschalten
void setDeviceOff()
{
  deviceIsOn = false;            // Setzt den Zustand des Geräts auf "aus"
}

void loop()
{
  // Berechnen der Abtastrate
  unsigned long aktuellMicros = micros();      
  unsigned long zeitDifferenz = aktuellMicros - vor_Micros; 
  // Konvertirung der Zeit in sekunden 
  float Ts = zeitDifferenz / 1e6;           

  aktuelle_Zeit = millis();                // Speichert die aktuelle Zeit nach dem Start des Arduino

  temperatur = max.temperature(100, 431);  // Temperatureinlesung mit MAX31865 (Pt100), 431 Referenzwiderstand
  x = temperatur;                          // Speichert die Temperatur in der Variable x

  // Periodische Ausgabe der Temperatur
  if (aktuelle_Zeit - vor_Millis >= interval)      // Überprüft, ob 10 Sekunden vergangen sind
  {
    // Senden der Temperatur an die App
    mySerialBT.print("I:");                 // Sendet ein "I:" als Präfix
    mySerialBT.print(x);                    // Sendet die Temperatur
    mySerialBT.print("\n");                 // Sendet einen Zeilenumbruch
    
    // Sendet die Temperatur an den seriellen Monitor
    Serial.print("x(t): ");
    Serial.print(x);

    Serial.print(" yP: "); Serial.print(yP);
    Serial.print(" yI: "); Serial.print(yI);
    Serial.print(" yD: "); Serial.println(yD);

    vor_Millis = aktuelle_Zeit; // Zwischenvariable bekommt neuen Wert
  }

  // Übernahme einen Sollwert von der App!!!!!
  while (mySerialBT.available() > 0)                        // Überprüft, ob Daten über Bluetooth verfügbar sind
  {
    char receivedChar = mySerialBT.read();                  // Liest ein Zeichen aus dem Bluetooth-Serial-Buffer
    incomingData += receivedChar;                           // Fügt das gelesene Zeichen zur incomingData-String hinzu

    // Überprüfung, ob das Ende der Nachricht erreicht ist
    if (receivedChar == '\n')                               // Überprüft, ob das Zeichen ein Zeilenumbruch ist
    {
      Serial.print("Empfangen: ");
      Serial.println(incomingData);

      if (incomingData.startsWith("T:"))                    // Überprüft, ob der empfangene String mit "T:" beginnt
      {
        String tempString = incomingData.substring(2, incomingData.length() - 1); // Entfernt "T:" und "\n" aus dem String
        w = tempString.toFloat();                                                 // Konvertierung von String zu einem Float
      }

      // Lüfter Steuerung
      if (incomingData.startsWith("L:"))                    // Überprüft, ob der empfangene String mit "L:" beginnt
      {
        String incomingFanSpeed = incomingData.substring(2, incomingData.length()); // Entfernt "L:" und "\n" aus dem String
        fanSpeedInPercent = incomingFanSpeed.toInt();                               // Konvertiert von String zu einem Integer  
        fanSpeed = map(fanSpeedInPercent, 0, 100, 0, 255);                          // Ändert den Bereich von 0-100 zu 0-255                                                  
      }

      // Prüfkommunikation zum Anschalten
      if (incomingData.startsWith("S:"))
      {
        String state = incomingData.substring(2, incomingData.length() - 1);
        if (state == "1")                   // Überprüft, ob der verbleibende String "1" ist
        {
          setDeviceOn();                    // Schaltet das Gerät ein
        }
        else if (state == "0")              // Überprüft, ob der verbleibende String "0" ist
        {
          setDeviceOff();                   // Schaltet das Gerät aus
        }
      }

      Serial.println(incomingData);
      incomingData = "";                    // Setzt den incomingData-String zurück
    }
    receivedChar = (char)"";                // Setzt receivedChar zurück
  }
    // PID Regelglied
    Ki = Kp / Tn;                           // Berechnung von Integralanteil Ki
    Kd = Kp * Tv;                           // Berechnung von Ableitungsanteil Kd
    integral_min = 0;                       // Setzt die minimale Grenze für den Integralanteil
    integral_max = 4000;                    // Setzt die maximale Grenze für den Integralanteil

  // Begrenzt den Integralanteil zwischen den festgelegten Grenzen
    integral = constrain(integral, integral_min, integral_max);    

      e = w - x;                             // Berechnung von Regeldifferenzgröße
      integral += e * Ts ;                   // Aktuelle Fehler wird akkumuliert im Integralanteil
      ableitung = (e - e_alt) / Ts;          // Berechnung der Ableitung des Fehlers
      yP = Kp * e;                           // Berechnung des Proportionalanteils
      yI = Ki * integral;                    // Berechnung des Integralanteils
      yD = Kd * ableitung;                   // Berechnung des Differenzialanteils

      // Berechnung der Stellgröße (Ausgangssignal)
      y = yP + yI + yD;                      // Gesamtausgangssignal des PID-Reglers
      y = constrain(y, 0, 255);              // Begrenzt die Stellgröße zwischen 0 und 255

    digitalWrite(in1, HIGH);                 // Setzt in1 auf HIGH
    digitalWrite(in2, LOW);                  // Setzt in2 auf LOW
    
    if (deviceIsOn == true) // Überprüft, ob das Gerät eingeschaltet ist
    {
      analogWrite(ssr, int(y));              // Schreibt die Stellgröße auf das Halbleiterrelais (SSR)
      analogWrite(enA, fanSpeed);            // Schaltet den Lüfter mit der festgelegten Geschwindigkeit ein
    }
    else // Falls das Gerät ausgeschaltet ist
    {
      analogWrite(ssr, 0);                   // Schaltet das Heizelement aus
      analogWrite(enA, 0);                   // Schaltet den Lüfter aus
    }
  e_alt = e;
  vor_Micros = aktuellMicros;
}
