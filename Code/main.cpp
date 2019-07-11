#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>

#define PIXEL_PIN    D2  // Digital IO pin connected to the NeoPixels.
#define PIXEL_COUNT  64 // Number of pixels

#define ANALOG_PIN  A0  // Defines used analog Pin
#define BUTTON_PIN  D4  // Defines used Button Pin

// WiFi Settings
WiFiClient espClient;
PubSubClient client(espClient);

// Settings for NeoPixels
Adafruit_NeoPixel strip(PIXEL_COUNT, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

// Prototypes
void setLine(uint8_t line , uint32_t color);
void reconnect();
void callback(char* topic, byte* payload, unsigned int length);

// Further definitions
uint32_t color = strip.Color(255,0,0);    //red color

// Used variables
int analogValue = 0;
int buttonState = 0;
char text[40];


void setup() 
{

  pinMode(ANALOG_PIN, INPUT);         // Analoger Eingang an PIN A0
  pinMode(BUTTON_PIN,INPUT_PULLUP);   // Taster Eingang an PIN D4 mit PullUp

  Serial.begin(9600);                 // Baudrate 9600
  Serial.println("Hallo Welt");       // Begrüßungstext

  // NeoPixel Init
  strip.begin(); // Initialize NeoPixel strip object (REQUIRED)
  strip.show();  // Initialize all pixels to 'off'
  strip.setBrightness(50);

  // WiFi settings
  WiFi.mode(WIFI_STA);
  WiFi.hostname("Lokal");
  WiFi.begin("SSID", "password");

  // Warte auf Server Anbindung
  int index = 0;
  while (WiFi.status() != WL_CONNECTED) 
  {
    delay(500);
    Serial.print(".");

    // Zeigt grüne laufende LEDs an (Waiting for Connection)
    if(index >= 8)
    {
      index = 0;
    }
    strip.clear();
    strip.setPixelColor(index+32, strip.Color(0,255,0));         //  Set pixel's color (in RAM)
    strip.show();

    index++;
  }

  Serial.println(WiFi.localIP());

  client.setServer("192.168.0.42", 1883);
  client.setCallback(callback);
}

void loop() 
{
  // Erneuter Verbindungsaufbau bei Connection-Verlust
  if (!client.connected())
   reconnect();

  client.loop();
  
  // Lese Analog-Wert vom Joystick
  analogValue = analogRead(ANALOG_PIN);

  // Level Anzeige auf NeoPixel Matrix
  if (analogValue >= 0 && analogValue <= 124)
  {
    setLine(0,color);
  }
  else if (analogValue >= 132 && analogValue <= 252)
  {
    setLine(1,color);
  }
  else if (analogValue >= 260 && analogValue <= 380)
  {
    setLine(2,color);
  }
  else if (analogValue >= 388 && analogValue <= 508)
  {
    setLine(3,color);
  }
  else if (analogValue >= 516 && analogValue <= 636)
  {
    setLine(4,color);
  }
  else if (analogValue >= 644 && analogValue <= 764)
  {
    setLine(5,color);
  }
  else if (analogValue >= 772 && analogValue <= 892)
  {
    setLine(6,color);
  }
  else if (analogValue >= 900 && analogValue <= 1024)
  {
    setLine(7,color);
  }

  // Taster-Abfrage
  buttonState = digitalRead(BUTTON_PIN);

  // Wenn taster gedrückt
  if (buttonState == 0)
  {
    sprintf(text, "Value of Joystick is %d", analogValue);
    client.publish("/joystick/switch", text);
  
  }

  // Übertrage Joystick Daten
  sprintf(text, "%d", analogValue);
  client.publish("/joystick/achse", text);

  // Wait for it
  delay(20);
}


// Subroutine für NeoPixel um Linie anzuzeigen  
void setLine(uint8_t line , uint32_t color)
{
  strip.clear();

  for(int i=line*8; i<(line*8)+8; i++)
  {
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
  }
  strip.show();
}


void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (unsigned int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    if (client.connect("client-sfasdfasdf")) {
      Serial.println("connected");
      client.publish("/status/devices", "I'm here");
      //client.subscribe("/joystick");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      // Warte 5 Sekunden und zeige grüne laufende LEDs an
      for(int i=0; i<8; i++)
      {
        strip.clear();
        strip.setPixelColor(i+32, strip.Color(0,255,0));         //  Set pixel's color (in RAM)
        strip.show();

        delay(625);
      }
    }

    strip.clear();
  }
}