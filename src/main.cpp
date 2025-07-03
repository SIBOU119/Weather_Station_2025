#include <Arduino.h>
#include "Adafruit_BMP3XX.h"
#include "RunningMedian.h"
#include <ESP8266WiFi.h>        // Include the Wi-Fi library
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

#define NB_AVG      50

#define DAY_HOUR      14
#define DAY_MINUTE    00
#define NIGHT_HOUR    02
#define NIGHT_MINUTE  00

#define DEBUG

//const char *ssid = "LE NOM DU WIFI";
//const char *password = "MOT DE PASSE DU WIFI";
const char *ssid = "TheAlienHub";
const char *password = "livelyship925";

const char* ntpServer = "north-america.pool.ntp.org";
const long  gmtOffset_sec = -18000;   // -5 * 60 * 60 = -18000
const int   daylightOffset_sec = 3600;
struct tm weather365Time;
bool notSaved = true;

// Windspeed sensor on GPIO26
const byte interruptPin = 26;
volatile int interruptCounter = 0;

unsigned long Time = 0;
unsigned long OldTime = 0;
float tourParSeconde = 0.0f;

unsigned long wifiSendData_time = 0;
unsigned long wifiSendData_OldTime = 0;

unsigned long updateData_time = 0;
unsigned long updateData_OldTime = 0;

float humidity    = 0.0f;
float temperature = 0.0f;
float pressure    = 0.0f;
float WindSpeed   = 0.0f;

Adafruit_BMP3XX bmp;

RunningMedian samples_Temperature = RunningMedian(NB_AVG);
RunningMedian samples_Pression = RunningMedian(NB_AVG);
RunningMedian samples_WindSpeed = RunningMedian(NB_AVG);
RunningMedian samples_Humidity = RunningMedian(NB_AVG);

float median_Temperature = 0;
float median_Pression = 0;
float median_Humidity = 0;
float highest_WindSpeed = 0;

void printLocalTime(void);

//portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
void IRAM_ATTR isr() {
  //portENTER_CRITICAL_ISR(&mux);
  interruptCounter++;
  Time = millis();
  //portEXIT_CRITICAL_ISR(&mux);
}

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("");
  Serial.println("WEATHER-MAN BASE STATION");

  pinMode(interruptPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(interruptPin), isr, FALLING);

  if(!bmp.begin_I2C())  // Temperature + Pressure sensor
  {
    Serial.println("Unable to connect to BMP388...");
    while(1);
  }
  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.performReading();

  delay(5000); // Delay needed before calling the WiFi.begin

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  { //Check for the connection
    Serial.println("Connecting to WiFi..");
    delay(2000);
  }
  Serial.println("Connected to the WiFi network");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  Serial.println();

  // Get the latest timing
  updateData_OldTime = millis();    // To get new sensors data and averaging
  wifiSendData_OldTime = millis();  // To upload new data to server
  // OldTime = millis();               // For the interrupt : Wind Speed sensor

}

void loop() {
  
  if(interruptCounter > 0)
  {
      //portENTER_CRITICAL(&mux);
      interruptCounter--;
      //portEXIT_CRITICAL(&mux);

      tourParSeconde = 1 / ((float)(Time - OldTime) / 1000.0f);
      WindSpeed = tourParSeconde * 2 * PI * 0.0775;

#ifdef DEBUG_WIND_SPEED
      Serial.print("Time between interrupts: ");
      Serial.println(Time - OldTime);

      Serial.print("Speed: ");
      Serial.println(tourParSeconde);
#endif
      OldTime = Time;
  }

  // Getting new data every 10 seconds
  updateData_time = millis();
  if((updateData_time - updateData_OldTime) > 10000)
  //if((updateData_time - updateData_OldTime) > 2000)
  {
    updateData_OldTime = updateData_time;
    
    bmp.performReading();
    temperature = bmp.temperature;
    pressure    = bmp.pressure / 1000.0;

    samples_Temperature.add(temperature);
    samples_Pression.add(pressure);
    //samples_WindSpeed.add(WindSpeed);

    median_Temperature = samples_Temperature.getMedian();
    median_Pression = samples_Pression.getMedian();
    median_Humidity = samples_Humidity.getMedian();
    highest_WindSpeed = samples_WindSpeed.getHighest();    // Get the highest value of wind speed

#ifdef DEBUG
    //weatherman.printLocalTime();

    Serial.println("->AvgTemp = " + String(median_Temperature)
                    + " AvgHum = " + String(median_Humidity)
                    + " AvgPress = " + String(median_Pression)
                    + " AvgWindSpeed = " + String(highest_WindSpeed));

    Serial.print("Temperature BMP = ");
    Serial.print(temperature);
    Serial.println(" °C");

    Serial.print("Humidity = ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial.print("Pressure = ");
    Serial.print(pressure);
    Serial.println(" kPa");

    Serial.print("Wind Speed = ");
    Serial.print(WindSpeed);
    Serial.println(" m/s");
#endif
  }
  Serial.println("Test...");
  delay(1000); // Delay needed before calling the WiFi.begin

   // Sending data to SERVER every 5 minutes
  wifiSendData_time = millis();
  if((wifiSendData_time - wifiSendData_OldTime) > (5 * 60000))   // Send a request every 5 minutes
  {
    wifiSendData_OldTime = wifiSendData_time;
    printLocalTime();
    Serial.println("Sending data to cloud....");
  
    if (WiFi.status() == WL_CONNECTED)  // Check WiFi connection status
    { 
      if((!isnan(median_Temperature)) && (!isnan(median_Humidity)) && (!isnan(median_Pression)) && (!isnan(highest_WindSpeed)))
      {
        WiFiClient client;
        HTTPClient http;
        
        String httpRequestData = "http://192.168.2.106:8030/addWeather?station_name=" + String("Le-QG") 
                                  + "&temperature=" + String(median_Temperature)
                                  + "&humidity=" + String(median_Humidity)
                                  + "&pressure=" + String(median_Pression)
                                  + "&wind_speed=" + String(highest_WindSpeed)
                                  + "&wind_direction=" + String(999.9) + "";

        http.begin(client, httpRequestData);

        int httpResponseCode = http.POST("POSTING from ESP32"); //Send the actual POST request

        if (httpResponseCode > 0)
        {
          String response = http.getString(); //Get the response to the request

          Serial.print(String(httpResponseCode) + " "); // Print return code
          Serial.println(response);             // Print request answer
        }
        else
        {
          Serial.print("Error on sending POST: ");
          Serial.println(httpResponseCode);
        }
        http.end(); //Free resources
      }
      else
      {
        Serial.println("There is a NAN value !!");
        
      }

      // Send all the temperature sensor values
      if(!isnan(median_Temperature))
      {
        WiFiClient client;
        HTTPClient http;
        
        String httpRequestData = "http://192.168.2.106:8030/addTemp?temperature1=" + String(temperature)
                                  + "&temperature2=" + String(0)
                                  + "&temperature3=" + String(0)
                                  + "&temperature4=" + String(0) + "";
                                
        http.begin(client, httpRequestData);

        int httpResponseCode = http.POST("POSTING from ESP32"); //Send the actual POST request

        if (httpResponseCode > 0)
        {
          String response = http.getString(); //Get the response to the request

          Serial.print(String(httpResponseCode) + " "); // Print return code
          Serial.println(response);             // Print request answer
        }
        else
        {
          Serial.print("Error on sending POST: ");
          Serial.println(httpResponseCode);
        }
        http.end(); //Free resources
      }
      else
      {
        Serial.println("There is a NAN value !!");
      }
    }
    else
    {
      Serial.println("======================================== Error in WiFi =============================================");
      Serial.println("Error in WiFi connection -- Trying to reconnect...");
      WiFi.begin(ssid, password);
      while (WiFi.status() != WL_CONNECTED)
      { //Check for the connection
        delay(2000);
        Serial.println("Re-connecting to WiFi..");
      }
    }
    Serial.println();
  }


  // Sending the daily data 
  // Sending data to SERVER 2 times a day
  getLocalTime(&weather365Time);
  // Remet le compte a zero pour l'enregistrement quotidient
  if((weather365Time.tm_min == (DAY_MINUTE + 5) || weather365Time.tm_min == (NIGHT_MINUTE + 5))) notSaved = true;

  if(notSaved == true)
  {
    if((((weather365Time.tm_hour == DAY_HOUR) && (weather365Time.tm_min == DAY_MINUTE)) || 
        ((weather365Time.tm_hour == NIGHT_HOUR) && (weather365Time.tm_min == NIGHT_MINUTE))))
    {
      Serial.println("Got it!");
      notSaved = false;
      // Send data to cloud
      printLocalTime();

      Serial.println("Sending DAILY data to cloud....");
    
      if (WiFi.status() == WL_CONNECTED)  // Check WiFi connection status
      { 
        if((!isnan(median_Temperature)) && (!isnan(median_Humidity)) && (!isnan(median_Pression)))
        {
          WiFiClient client;
          HTTPClient http;

          String httpRequestData = "http://192.168.2.106:8030/addWeather365?station_name=" + String("Le-QG") 
                                    + "&temperature=" + String(median_Temperature)
                                    + "&humidity=" + String(median_Humidity)
                                    + "&pressure=" + String(median_Pression) + "";

          //http.begin("http://192.168.2.106:8030/addWeather365?station_name=abc&temperature=23.68&humidity=75.05&pressure=101.325");
          http.begin(client, httpRequestData);

          int httpResponseCode = http.POST("POSTING from ESP32"); //Send the actual POST request

          if (httpResponseCode > 0)
          {
            String response = http.getString(); //Get the response to the request

            Serial.print(String(httpResponseCode) + " "); // Print return code
            Serial.println(response);             // Print request answer
          }
          else
          {
            Serial.print("Error on sending POST: ");
            Serial.println(httpResponseCode);
          }
          http.end(); //Free resources
        }
        else
        {
          Serial.println("There is a NAN value !!");
        }
      }
      else
      {
        Serial.println("======================================== Error in WiFi =============================================");
        Serial.println("Error in WiFi connection -- Trying to reconnect...");
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED)
        { //Check for the connection
          delay(2000);
          Serial.println("Re-connecting to WiFi..");
        }
      }
      Serial.println();
    }
    else notSaved = true;
  }
}

void printLocalTime(void)
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  // Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}