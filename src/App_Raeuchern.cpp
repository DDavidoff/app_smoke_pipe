#include <SoftwareSerial.h>
#include <Adafruit_MAX31865.h> //Bibliothek um MAX31865 zu verwenden

// Verbindung zwischen Arduino und Max31865 über SPI-Bus
#define MAX_SCK 13  // Pin 13 (Arduino) ist die Schnittstelle mit SCK (MAX31865)
#define MAX_MISO 12 // Pin 12 (Arduino) ist die Schnittstelle mit MISO (MAX31865)
#define MAX_MOSI 11 // Pin 11 (Arduino) ist die Schnittstelle mit MOSI (MAX31865)
#define MAX_CS 10   // Pin 10 (Arduino) ist die Schnittstelle mit CS (MAX31865)

// Klasse Adafruit_Max31865 ermöglicht Kommunikation mit dem Modul MAX31865 über definierte Pins
Adafruit_MAX31865 max = Adafruit_MAX31865(MAX_CS, MAX_MOSI, MAX_MISO, MAX_SCK);

float temperatur; // Temperatur Variable in Datentyp float

// Zeitvariablen
unsigned long zMillis = 0;   // Zwischenspeicher Variable für Millis-Funktion
const long interval = 10000; // Intervall Variable für Millis-Funktion (10 sekunden)
unsigned long aktuelle_Zeit; // Speicher-Variable für die aktuelle Zeit

// Lüfter 12V
const int enA = 6;
const int in1 = 5;
const int in2 = 4;
int wunschGeschwindigkeit = 0; // Geschwindigkeit des Lüfters als Wert zwischen 0 und 255
int fanSpeedInPercent;
int fanSpeedInBytes = 0;

// Temperatursteuerung von Räucherofen
const int ssr = 9;  // Halbleiterrelais ist am Pin 9 festgelegt
float y;            // Stellgröße Ausgangsignal für SSR
float w = 85.0;     // Führungsgröße w(t) (Sollwert-Temperatur)
float x;            // Regelgröse x(t) (Istwert-Temperatur)
float e;            // Regeldifferenzgröße e(t) = w(t) - x(t)
float e_alt;        // Regel differenzsgröße von Vorlauf
float integral = 0; // Summe der Fehler im Zeitverlauf (Startwert)
float integral_max; // Deklaration der Maximale Grenze für Integralanteil
float integral_min; // Deklaration der Minimale Grenze für Integralanteil
float ableitung;    // Differentialanteil

float yP, yI, yD;

// PID-Regler Parameter
float Kp = 7.203;   // Proportional Konstante
float Ki;           // Integralanteil
float Kd;           // Ableitungsteil
float Tn = 723.805; // Nachstellzeit
float Tv = 41.498;  // Verstärkungszeit
float Tvz;          // Abtastzeit

// App Kommunikation
long Resistor = 0;
String wStr = "";
byte byteBuffer[4];
String incomingData = "";
String program;
long countdown = 0;
long countdownStartTime;
long countdownCurrentTime = 0;

SoftwareSerial mySerialBT(3, 2);

void setup()
{

  Serial.begin(9600); // Serielle Kommunikation mit einer Baudrahte 9600
  mySerialBT.begin(9600);
  max.begin(MAX31865_3WIRE); // Kommunikation mit 3-Draht PT100 Sensor über MAX31865
  pinMode(ssr, OUTPUT);      // Konfiguriere Pin 9 als Ausgang

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
}

bool deviceIsOn = false;
void setDeviceOn()
{
  deviceIsOn = true;
  countdownStartTime = millis();
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  Serial.println("Device was set on.");
}

void setDeviceOff()
{
  deviceIsOn = false;
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);

  Serial.println("Device was set off.");
}

void heatOn(int pwmY)
{
  analogWrite(ssr, pwmY); // Aktiviert den SSR (Solid State Relay) mit PWM-Wert 'y'
}

void heatOff()
{
  analogWrite(ssr, 0); // Deaktiviert den SSR, indem der PWM-Wert auf 0 gesetzt wird
}

void fanOn(byte fanSpeed)
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, fanSpeed);
}

void fanOff()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

void loop()
{

  aktuelle_Zeit = millis(); // Aktuelle Zeit nach dem Start des Arduino

  temperatur = max.temperature(100, 431); // Temperatureinlesung mit MAX31865 (Pt100), 431 Referenzwiderstand
  x = temperatur;

  // Auslesung und Ausgabe der Temperatur jeder  10 sek.
  if (aktuelle_Zeit - zMillis >= interval) // Bedingung: Wenn >= 10000 dann wird Funktion für 10 sek gestartet
  {
    // Senden der Temperatur an die App
    mySerialBT.print("Istwert:");
    mySerialBT.print(x);
    mySerialBT.print("\n");
    // Sende  der Temperatur in die device logs
    Serial.print("Istwert:");
    Serial.println(x);

    Serial.print("aktuelle_Zeit");
    Serial.println(aktuelle_Zeit);
    Serial.print("countdownStartTime");
    Serial.println(countdownStartTime);
    Serial.print("countdownCurrentTime");
    Serial.println(countdownCurrentTime);
    Serial.print("countdown");
    Serial.println(countdown);

    zMillis = aktuelle_Zeit; // Zwischenvariable bekommt neuen Wert
  }

  // Übernahme einen Sollwert von der App!!!!!
  while (mySerialBT.available() > 0)
  {
    char receivedChar = mySerialBT.read();
    incomingData += receivedChar;

    // Überprüfung, ob das Ende der Nachricht erreicht ist
    if (receivedChar == '\n')
    {
      Serial.print("Empfangen: ");
      Serial.println(incomingData);

      if (incomingData.startsWith("T:")) // Aufnahme von Führungagröße (Heizelement)
      {
        String tempString = incomingData.substring(2, incomingData.length() - 1); // Entfernt "T:" und "\n"
        w = tempString.toFloat();                                                 // Konvertierung von string zu float

        Serial.print("Sollwert ist jetzt: ");
        Serial.println(w);
      }

      // Lüfter Steuerung
      if (incomingData.startsWith("L:")) // Aufnahme von Stellwert (Lüvter 12V)
      {
        String incomingFanSpeed = incomingData.substring(2, incomingData.length());
        fanSpeedInPercent = incomingFanSpeed.toInt();             // Entfernt "L:" und "\n" und
        fanSpeedInBytes = map(fanSpeedInPercent, 0, 100, 0, 255); // Ändert den Bereich von 0-100 zu 0-255                                                       // Serial.println(fanSpeedInBytes);
      }

      // Prüfkommunikation zum Anschalten
      if (incomingData.startsWith("S:"))
      {
        String state = incomingData.substring(2, incomingData.length() - 1);
        if (state == "1")
        {
          setDeviceOn();
        }
        else if (state == "0")
        {
          setDeviceOff();
        }
      }

      if (incomingData.startsWith("P:")) // Aufnahme der Werte von Festen Einstellungen
      {
        program = incomingData.substring(2, incomingData.length() - 1); // Entfernt "P:" und "\n"
        Serial.print("Program selected: ");
        Serial.println(program);
        if (program == "Aal") // Programm "Aal" mit fest eingestellten Werten
        {
          countdown = 5000; // Rückzählung ist auf 30 min
        }
        if (program == "Forelle") // Programm "Forelle" mit fest eingestellten Werten
        {
          countdown = 5000; // Rückzählung ist auf 30 min
        }
        Serial.print("Programmed heat: ");
        Serial.println(w);
      }
      Serial.println(incomingData);
      incomingData = "";
    }
    receivedChar = (char)"";

    // PID Regelglied
    Ki = Kp / Tn; // Berechnung von Integralanteil Ki
    Kd = Kp * Tv; // Berechnung von Ableitungsanteil Kd
    integral_min = 0;
    integral_max = 3000;
    e = w - x;     // Berechnung von Regeldifferenzgröße
    integral += e; // Aktuelle Fehler wird akkumuliert im Integralanteil
    integral = constrain(integral, integral_min, integral_max);
    ableitung = (e - e_alt) / 0.1;
    e_alt = e;

    yP = Kp * e;
    yI = Ki * integral;
    yD = Kd * ableitung;

    // Berechnung der Stellgröße
    y = Kp * e + Ki * integral + Kd * ableitung;
    y = constrain(y, 0, 255);

    Serial.print("aktuelle_Zeit");
    Serial.println(aktuelle_Zeit);
    Serial.print("countdownStartTime");
    Serial.println(countdownStartTime);
    Serial.print("countdownCurrentTime");
    Serial.println(countdownCurrentTime);
    Serial.print("countdown");
    Serial.println(countdown);
 

    if (deviceIsOn == true) // Überprüft, ob das Gerät eingeschaltet ist
    {
      if (countdown == 0)
      {
        heatOn((int)y);
        fanOn(fanSpeedInBytes);
      }
      else if (countdown != 0 && aktuelle_Zeit >= (countdownStartTime + countdown))
      {
        setDeviceOff();
        
        Serial.println("we are stopping heating and blowing");
      }
      else if (countdown != 0 && aktuelle_Zeit < (countdownStartTime + countdown))
      {
        Serial.println("we are heating and blowing");
        heatOn((int)y);
        fanOn(fanSpeedInBytes);
      }
    }
    else // Falls das Gerät ausgeschaltet ist
    {
      heatOff();
      fanOff();
    }
  }
}
