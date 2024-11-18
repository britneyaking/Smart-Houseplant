/*
 * Project Midterm_2_SmartHousePlant
 * Author: Britney A. King
 * Date: 11/14/24
 */

#include "Particle.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include "Adafruit_BME280.h"
#include "math.h"
#include "credentials.h"
#include "Grove_Air_quality_Sensor.h"

// Define OLED
#define OLED_RESET D4
Adafruit_SSD1306 display(OLED_RESET);
String DateTime, TimeOnly; // needed for getting the current time to send to the Adafruit dashboard

// BME
#define BME_ADDRESS 0x76
Adafruit_BME280 bme;

const int BMEPIN = D0;
float Pa;
float tempC;
float tempF;
float pressPA;
float humidRH;
int ccTemp;
int ccPressure;
int ccHumid;

int starttime;

TCPClient TheClient;
Adafruit_MQTT_SPARK mqtt(&TheClient, AIO_SERVER, AIO_SERVERPORT, AIO_USERNAME, AIO_KEY);

// Setup Feeds to publish or subscribe
Adafruit_MQTT_Publish mqtttempF = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempF");
Adafruit_MQTT_Publish mqttpressPA = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressPA");
Adafruit_MQTT_Publish mqttrhumidRH = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidRH");
Adafruit_MQTT_Publish mqttmoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moisture");
Adafruit_MQTT_Publish mqttAirQuality = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/AirQuality");

// Adafruit_MQTT_Publish mqtttime = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/time");  //time object for MQTT
// Adafruit_MQTT_Subscribe mqttSubPumpButton = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/pumpButton"); Updated Name
Adafruit_MQTT_Subscribe mqttwaterOnOff = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterOnOff");

// Set Sensor Pins
const int PUMP = D10;          // sets the pin to use for the pump
const int MOISTURESENSOR = A1; // pin to use for the moisture sensor
AirQualitySensor sensor(A2);   // pin to use for the AQ sensor

unsigned long last, lastTime;
int valueB;         // MQTT Button
int moistureValues; // var to store the moisture probe values
int lastTime2;

void MQTT_connect();
void showDisplayValues();

void setup()
{
  Serial.begin(9600);
  bme.begin(0x76);

  // OLED
  Time.zone(-7);
  Particle.syncTime();
  display.begin(SSD1306_SWITCHCAPVCC, 0x3c);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.display();

  pinMode(MOISTURESENSOR, INPUT); // sets the pinMode to moisture probe
  pinMode(PUMP, OUTPUT);          // sets the pinMode to motor pump

  WiFi.connect();
  while (WiFi.connecting())
  {
    Serial.printf(".");
  }

  mqtt.subscribe(&mqttwaterOnOff); // subscribe to pump button from Adafruit
  starttime = millis();            // stores current time to the starttime var
}

void loop()
{

// OLED
  DateTime = Time.timeStr();
  TimeOnly = DateTime.substring(11, 19);

// MQTT
  MQTT_connect();

  if ((millis() - last) > 120000)
  {
    Serial.printf("Pinging MQTT \n");
    if (!mqtt.ping())
    {
      Serial.printf("Disconnecting \n");
      mqtt.disconnect();
    }
    last = millis();
  }

  // MQTT Subscription
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100)))
  {
    if (subscription == &mqttwaterOnOff)
    {
      valueB = atoi((char *)mqttwaterOnOff.lastread);                          // takes last data and converts it char and converts it to a float
      Serial.printf("Received %i from Adafruit.io feed FeedNameB \n", valueB); // prints to screen
    }
  }

  // Pump
  if (valueB == 1)
  {                           // if the the soil is dry(less than value), pump water
    digitalWrite(PUMP, HIGH); // turns pump on
    Serial.printf("Pump is ON \n");
    delay(500);
    digitalWrite(PUMP, LOW); // turns pump off
    Serial.printf("Pump is OFF \n");
    delay(500);
  }
  if ((millis() - lastTime2) > (10000))
  {

    moistureValues = analogRead(MOISTURESENSOR);

    if (mqtt.Update())
    {
      mqttmoisture.publish(moistureValues);
      Serial.printf("Publishing moistureValues %0.2f \n", moistureValues);

      if (moistureValues > 2300)
      {                           // if the the soil is dry(less than value), pump water
        digitalWrite(PUMP, HIGH); // turns pump on
        Serial.printf("Pump is ON \n");
        delay(500);
        digitalWrite(PUMP, LOW); // turns pump off
        Serial.printf("Pump is OFF \n");
        delay(500);
      }
      lastTime2 = millis();
    }
          }
 lastTime2 = millis();
  }

  showDisplayValues(); // function to print sensor values to OLED
 
  if ((millis() - lastTime > 20000))
  {

    // Air Quality Check Function
    int quality = sensor.slope();
    Serial.print("Sensor value: ");
    Serial.println(sensor.getValue());

    if (quality == AirQualitySensor::FORCE_SIGNAL)
    {
      Serial.println("High pollution! Force signal active.");
    }
    else if (quality == AirQualitySensor::HIGH_POLLUTION)
    {
      Serial.println("High pollution!");
    }
    else if (quality == AirQualitySensor::LOW_POLLUTION)
    {
      Serial.println("Low pollution!");
    }
    else if (quality == AirQualitySensor::FRESH_AIR)
    {
      Serial.println("Fresh air.");
    }

    // BME
    tempC = bme.readTemperature();
    tempF = tempC * (9 / 5.0) + 32;
    // tempF = (bme.readTemperature()*1.8)+32;  //gets the temp values  and converts them to F
    pressPA = bme.readPressure() * 0.00030; // converts the press values to inches of murcury
    humidRH = bme.readHumidity();           // gets RH values from the BME sensor

    if (mqtt.Update())
    {
      mqtttempF.publish(tempF);
      Serial.printf("Publishing TempF %0.2f \n", tempF);
      mqttpressPA.publish(pressPA);
      Serial.printf("Publishing pressPA %0.2f \n", pressPA);
      mqttrhumidRH.publish(humidRH);
      Serial.printf("Publishing humidityRH %0.2f \n", humidRH);

      //mqttmoisture.publish(moistureValues);
      
      moistureValues = analogRead (MOISTURESENSOR);

      Serial.printf("Publishing Moisture %i \n", moistureValues);

      mqttAirQuality.publish(sensor.getValue());
      Serial.printf("Publishing AQ %i \n", sensor.getValue());
    }
    lastTime = millis();
  }
}

void MQTT_connect()
{
  int8_t ret;

  if (mqtt.connected())
  {
    return;
  }
  Serial.print("Connecting to MQTT... ");
  while ((ret = mqtt.connect()) != 0)
  {
    Serial.printf("%s\n", (char *)mqtt.connectErrorString(ret));
    Serial.printf("Retrying MQTT connection in 5 seconds..\n");
    mqtt.disconnect();
    delay(5000);
  }
  Serial.printf("MQTT Connected!\n");
}

// Print to OLED
void showDisplayValues()
{
  display.setCursor(0, 0);
  display.clearDisplay();                            // clears the previous values
  display.printf("Temp: %0.1f\n", tempF);            // shows the temp values
  display.printf("Pressure: %0.1f\n", pressPA);      // shows the pressure values
  display.printf("Humidity:  %0.1f\n", humidRH);     // shows the rel humidity values
  display.printf("AQ: %i \n", sensor.getValue());    // gets the air quality
  display.printf("Moisture: %i \n", moistureValues); // gets the moisture values

  display.printf("Time is %s\n", TimeOnly.c_str()); // gets time
  display.display();
}