#include <Arduino.h>
#include <Arduino_FreeRTOS.h>
#include <queue.h>
#include <Adafruit_Sensor.h>
#include <Wire.h> // include LCD library function
#include "DHT.h"
#include <Servo.h>

const int DHTPIN = 37;
const int DHTTYPE = DHT11;

DHT dht(DHTPIN, DHTTYPE);
QueueHandle_t intQueue;
QueueHandle_t floatQueue;
int visitors = 0;
int sensorOut = 31;
int sensorIn = 33;
int led = 35;
int ledState = 0;
int rainSensor = 39;
int servoPin = 41;

Servo servo;
void sensorTask(void *param);
void dhtTask(void *param);
void ledTask(void *param);
void pushDataToFirebaseTask(void *param);
void rainSensorTask(void *pvParameters);

void len()
{
  servo.write(0);
}
void xuong()
{
  servo.write(180);
}

void setup()
{
  Serial.begin(9600);
  dht.begin();
  servo.attach(servoPin);

  pinMode(led, OUTPUT);
  intQueue = xQueueCreate(5, sizeof(int));
  floatQueue = xQueueCreate(5, sizeof(float));

  if (intQueue != NULL)
  {
    xTaskCreate(sensorTask, "Sender", 100, NULL, 2, NULL);
    xTaskCreate(ledTask, "Receiver", 100, NULL, 2, NULL);
  }
  if (floatQueue != NULL)
  {
    xTaskCreate(dhtTask, "Dht", 5000, NULL, 2, NULL);
    xTaskCreate(pushDataToFirebaseTask, "Receiver", 100, NULL, 2, NULL);
  }
  xTaskCreate(rainSensorTask, "rain", 100, NULL, 2, NULL);
  vTaskStartScheduler();
}

void loop()
{
  // put your main code here, to run repeatedly:
}

void sensorTask(void *params)
{

  for (;;)
  {
    int sensorOutState = digitalRead(sensorOut);
    int sensorInState = digitalRead(sensorIn);
    if (sensorInState == LOW)
    {
      visitors++;
      delay(700);
    }
    else if (sensorOutState == LOW && visitors > 0)
    {
      visitors--;
      delay(700);
    }
    xQueueSend(intQueue, &visitors, portMAX_DELAY == pdPASS);
    Serial.print("Visitors = ");
    Serial.println(visitors);
    vTaskDelay(700 / portTICK_PERIOD_MS);
  }
}

void ledTask(void *params)
{
  int receiSignal = 0;
  for (;;)
  {

    if (xQueueReceive(intQueue, &receiSignal, portMAX_DELAY) == pdPASS)
    {

      Serial.print("Led State = ");
      Serial.println(receiSignal);
      if (receiSignal > 0)
      {
        digitalWrite(led, HIGH);
      }
      else
      {
        digitalWrite(led, LOW);
      }
    }
  }
}

void dhtTask(void *pvParameters)
{
  const TickType_t xTicksToWait = 2000 / portTICK_PERIOD_MS;

  for (;;)
  {
    Serial.println("\n --- Read temp ---");

    float temp = dht.readTemperature();
    // Serial.println(temp);
    xQueueSend(floatQueue, &temp, portMAX_DELAY == pdPASS);

    vTaskDelay(xTicksToWait);
    // taskYIELD();
  }
}

void pushDataToFirebaseTask(void *pvParameters)
{
  const TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS;
  bool xStatus;
  float Data;
  for (;;)
  {
    Serial.println("\n --- Send temp and hum ---");
    xStatus = xQueueReceive(floatQueue, &Data, portMAX_DELAY);

    if (xStatus == true)
    {
      Serial.println(Data);
    }
    vTaskDelay(xTicksToWait);
  }
}

void rainSensorTask(void *pvParameters)
{
  const TickType_t xTicksToWait = 1000 / portTICK_PERIOD_MS;
  int test = 0;
  for (;;)
  {
    test = digitalRead(rainSensor);
    Serial.println(test);
    if (test == 0)
    {
      len();
    }
    else
    {
      xuong();
    }
    vTaskDelay(xTicksToWait);
  }
}
