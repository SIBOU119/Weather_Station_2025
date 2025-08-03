#include "Arduino.h"
#include "bme68xLibrary.h"
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>
#include <RunningMedian.h>

// Mettre à jour les info suivante pour configurer le WiFi:
const char *ssid = "WIFI-NAME";
const char *password = "WIFI-PASSWORD";

const char *server_address = "weather.computatrum.cloud"; //"192.168.1.236";
const char *name_of_local_station = "St-Jean-Chrysostome";

// BE CARFUL, the following strings are using %s for data substitution
const char *POST_current_weather = "http://%s:8025/addWeather?station_name=%s&temperature=%s&humidity=%s&pressure=%s&wind_speed=%s&air_quality=%s";
const char *POST_daily_weather   = "http://%s:8025/addWeather365?station_name=%s&temperature=%s&humidity=%s&pressure=%s";

// Example: 
// http://weather.computatrum.cloud:8025/addWeather?station_name=St-Jean-Chrysostome&temperature=15&humidity=50&pressure=1005&wind_speed=0&air_quality=0

// =====> UPDATED to send windsensor data more often for debug purpose
const int SENDING_TO_SERVER_TIME_INTERVAL = 1;  // Time in minutes
const int NB_AVG = 50;

// Times at which the daily weather values will be recorded
const int DAY_HOUR      = 14;
const int DAY_MINUTE    = 00;
const int NIGHT_HOUR    = 02;
const int NIGHT_MINUTE  = 00;

//**********************************************************************************************
#define SDA_PIN 2
#define SCL_PIN 14
#define ADD_I2C 0x76  // 0x76 or 0x77

#define analogPin A0 /* ESP8266 Analog Pin ADC0 = A0 */

const char* ntpServer = "north-america.pool.ntp.org";
const long  gmtOffset_sec = -18000;   // -5 * 60 * 60 = -18000
const int   daylightOffset_sec = 3600;
struct tm weather365Time;
bool notSaved = true;

unsigned long currentTime = 0;
unsigned long wifiSendData_OldTime = 0;
unsigned long updateData_OldTime = 0;

struct weather_var
{
  float temperature = 0;
  float pressure    = 0;
  float humidity    = 0;
  float AirQuality  = 0;
  float WindSpeed   = 0;

  float median_Temperature = 0;
  float median_Pression = 0;
  float median_Humidity = 0;
  float median_AirQuality = 0;
  float median_Windspeed = 0;
}weather_var;

RunningMedian samples_Temperature = RunningMedian(NB_AVG);
RunningMedian samples_Pression = RunningMedian(NB_AVG);
RunningMedian samples_Humidity = RunningMedian(NB_AVG);
RunningMedian samples_WindSpeed = RunningMedian(NB_AVG);
RunningMedian samples_AirQuality = RunningMedian(NB_AVG);

Bme68x bme;
bme68xData data;

// Function declaration. Definitions are at the end of this file
void initializeSensor(void);
void getWeatherDataFromSensor(void);
float getWindSpeed(void);
void printLocalTime(void);
void try_reconnect_to_wifi(void);
void send_weather_data_to_server(char *buffer);

// *******************************************************************************************
// Setup the ESP8266, the sensor and connect to WiFi
// This function is executed only once at start-up
// *******************************************************************************************
void setup(void)
{
	Serial.begin(115200);
	while (!Serial);
  Serial.println("WEATHER-MAN BASE STATION is booting...");
		
	initializeSensor();

  delay(1000);  // Delay needed before calling the WiFi.begin
  
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Connecting to WiFi..");
    delay(2000);
  }
  Serial.println("Connected to the WiFi network: " + String(ssid));
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  printLocalTime();
  Serial.println();

  // Get the latest timing
  updateData_OldTime = millis();    // To get new sensors data and averaging
  wifiSendData_OldTime = millis();  // To upload new data to server

	Serial.println("TimeStamp(ms), Temperature(deg C), Pressure(Pa), Humidity(%), Gas resistance(ohm), Status");
}

// *******************************************************************************************
// Main program looping forever
// *******************************************************************************************
void loop(void)
{
  // Get current time
  currentTime = millis();

  // Getting new data every 10 seconds
  if((currentTime - updateData_OldTime) > 10000)
  {
    updateData_OldTime = currentTime;
    getWeatherDataFromSensor();    
  }

  // Sending data to SERVER every 5 minutes
  if((currentTime - wifiSendData_OldTime) > (SENDING_TO_SERVER_TIME_INTERVAL * 60000))   // Send a request every x minutes
  {
    wifiSendData_OldTime = currentTime;

    if((!isnan(weather_var.median_Temperature)) && (!isnan(weather_var.median_Humidity)) && (!isnan(weather_var.median_Pression)))
    {
      char buffer[200];
      sprintf(buffer, POST_current_weather, server_address, name_of_local_station , String(weather_var.median_Temperature), String(weather_var.median_Humidity), String(weather_var.median_Pression), String(weather_var.median_Windspeed), String(weather_var.median_AirQuality));
      send_weather_data_to_server(buffer);
    }
    else Serial.println("There is a NAN value !!");
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
      notSaved = false;
    
      if((!isnan(weather_var.median_Temperature)) && (!isnan(weather_var.median_Humidity)) && (!isnan(weather_var.median_Pression)))
      {
        char buffer[200];
        sprintf(buffer, POST_daily_weather, server_address, name_of_local_station, String(weather_var.median_Temperature), String(weather_var.median_Humidity), String(weather_var.median_Pression));
        send_weather_data_to_server(buffer);
      }
      else Serial.println("There is a NAN value !!");
    }
    else notSaved = true;
  }
}

// *******************************************************************************************
//
// Custom function definition
//
// *******************************************************************************************
// Initialize the Bosch BME sensor and ADC input for wind sensor
void initializeSensor(void)
{
  // Initialize the ADC module for the wind sensor
  Wire.begin(SDA_PIN, SCL_PIN);
  // initializes the sensor based on I2C library
  bme.begin(ADD_I2C, Wire);

	if(bme.checkStatus())
	{
		if (bme.checkStatus() == BME68X_ERROR)
		{
			Serial.println("Sensor error:" + bme.statusString());
			return;
		}
		else if (bme.checkStatus() == BME68X_WARNING) Serial.println("Sensor Warning:" + bme.statusString());
	}
	// Set the default configuration for temperature, pressure and humidity
	bme.setTPH(BME68X_OS_16X,BME68X_OS_16X,BME68X_OS_16X);
}

// Get new weather data from sensors (Bosch BME + Wind sensor)
void getWeatherDataFromSensor(void)
{
    bme.setOpMode(BME68X_SEQUENTIAL_MODE);
    // Get data from the Bosch sensor (Temp, Pressure, Humidity, Air Quality)
    if (bme.fetchData())
    {
      bme.getData(data);
      weather_var.temperature = data.temperature;
      weather_var.pressure    = data.pressure / 100;
      weather_var.humidity    = data.humidity;
      weather_var.AirQuality  = data.gas_resistance;

      samples_Temperature.add(weather_var.temperature);
      samples_Pression.add(weather_var.pressure);
      samples_Humidity.add(weather_var.humidity);
      samples_AirQuality.add(weather_var.AirQuality);

      weather_var.median_Temperature = samples_Temperature.getMedian();
      weather_var.median_Pression = samples_Pression.getMedian();
      weather_var.median_Humidity = samples_Humidity.getMedian();
      weather_var.median_AirQuality = samples_AirQuality.getMedian();
      //weather_var.median_AirQuality = samples_AirQuality.getHighest();    // Get the highest value of wind speed

      //Serial.println(data.temperature);
    }
    
    // Get wind speed from the ADC
    weather_var.WindSpeed = getWindSpeed();
    samples_WindSpeed.add(weather_var.WindSpeed);
    // weather_var.median_Windspeed = samples_WindSpeed.getMedian();
    //weather_var.median_Windspeed = samples_WindSpeed.getHighest();    // Get the highest value of wind speed
    weather_var.median_Windspeed = weather_var.WindSpeed;   // Use to debug windsensor. Will save the RAW sensor data
    //Serial.println("ADC Value = " + String(weather_var.WindSpeed));
    
    Serial.println("->AvgTemp = " + String(weather_var.median_Temperature)
                    + " AvgHum = " + String(weather_var.median_Humidity)
                    + " AvgPress = " + String(weather_var.median_Pression)
                    + " HighestWind = " + String(weather_var.median_Windspeed));
}

// Get the wind speed from the ADC
float getWindSpeed()
{
    int adc_windspeed = analogRead(analogPin); // Read the Analog Input value 
    adc_windspeed = adc_windspeed - 19;        // Remove offset when sensor not turning
    if(adc_windspeed <= 0) adc_windspeed = 0;

    return (float)adc_windspeed * 0.006 * 3.6;   // Compute windspeed in m/s
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

void try_reconnect_to_wifi()
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

// ******************************************************************************************************************************
// Generic function to send data on server.
// 
// ******************************************************************************************************************************
void send_weather_data_to_server(char *buffer)
{
  printLocalTime();
  Serial.println("Sending data to cloud....");
  if (WiFi.status() == WL_CONNECTED)  // Check WiFi connection status
  { 
    WiFiClient client;
    HTTPClient http;
    
    http.begin(client, String(buffer));

    int httpResponseCode = http.POST("POSTING from ESP8266"); //Send the actual POST request

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
    http.end(); // Free resources
  }
  else try_reconnect_to_wifi();
}
