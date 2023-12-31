#define MAX_BRIGHTNESS 255
#define REPORTING_PERIOD_MS 1000
#define OXY_SAMPLE 60

#include <Arduino.h>
#include <Adafruit_MLX90614.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"

MAX30105 particleSensor;
LiquidCrystal_I2C lcd(0x27, 16, 2);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();

uint8_t HeartChar[8] = {
    0b00000,
    0b01010,
    0b11111,
    0b11111,
    0b01110,
    0b00100,
    0b00000,
    0b00000};

uint8_t DegreeChar[8] = {
    B00110,
    B01001,
    B01001,
    B00110,
    B00000,
    B00000,
    B00000,
    B00000};

// SENSOR MLX90614
float body_temp = 0.0;

// SENSOR MAX30102
uint32_t irBuffer[100];
uint32_t redBuffer[100];

int32_t bufferLength;
int32_t spo2;
int32_t heartRate;

int8_t validSPO2;
int8_t validHeartRate;

int32_t trueOxy;
int32_t trueHR;

byte pulseLED = 2; // onboard led on esp32 nodemcu
byte readLED = 19;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE];    // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Pengukuran
boolean isMeasurement = false;
boolean isAllowMeasureTemperature = false;
boolean isAllowMeasureHeartRate = false;
boolean isAllowMeasureOxy = false;
boolean isAllowToSendData = false;
boolean isAllowTolcdTemperature = false;
boolean isAllowTolcdHeartRate = false;
boolean isAllowTolcdOxy = false;

// Data
String SSID = "deya";
String PWD = "smbx3835";
String DEVICE_ID = "fa05d";
long temperatureDataCounter = 0;
long temperatureTotalData = 0;
long oxyDataCounter = 0;
long oxyTotalData = 0;
long heartDataCounter = 0;
long heartTotalData = 0;
unsigned long temperatureTimeStamp = 0;
unsigned long temperatureInterval = 5000;
unsigned long heartRateTimeStamp = 0;
unsigned long heartRateInterval = 30000;
unsigned long oxyTimeStamp = 0;
unsigned long oxyInterval = 30000;
const char *MQTT_SERVER = "103.139.192.253";

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();

  String topic_listening = "/session/start/" + DEVICE_ID;
  const char *topic_listening_char = topic_listening.c_str();
  if (topic_listening == topic)
  {
    Serial.println("Mulai pengukuran");
    isMeasurement = true;
    temperatureTimeStamp = millis();
    isAllowMeasureTemperature = true;
    isAllowTolcdTemperature = true;
  }
}

void reconnect()
{
  // Loop until we're reconnected
  // client.setKeepAlive(90); // needs be made before connecting
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("stress-monitoring-0das20", "skripsimqtt", "@YZ7rqrLIDJ^!Qrz"))
    {
      Serial.println("connected");
      // Once connected, publish an announcement...
      String topic_listening = "/session/start/" + DEVICE_ID;
      const char *topic_listening_char = topic_listening.c_str();
      client.subscribe(topic_listening_char);
      Serial.println("Subscribe to " + topic_listening);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  WiFi.begin("deya", "smbx3835");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(500);
  }

  Serial.println("Wifi Connected.");
  Serial.println(WiFi.localIP());
  Serial.println("WORK");
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  lcd.init();
  lcd.setBacklight(1);
  lcd.setContrast(2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DEVICE_ID);
  lcd.setCursor(0, 1);
  lcd.print("Device Standby");
  lcd.createChar(0, HeartChar);

  // SET UP MLX90614
  Serial.println("Adafruit MLX90614 test");
  mlx.begin();

  // SET UP MAX30102
  ledcSetup(0, 0, 8);         // PWM Channel = 0, Initial PWM Frequency = 0Hz, Resolution = 8 bits
  ledcAttachPin(pulseLED, 0); // attach pulseLED pin to PWM Channel 0
  ledcWrite(0, 255);

  Serial.println("Initializing Pulse Oximeter..");
  // while (!particleSensor.begin())
  // {
  //   Serial.println("MAX30102 was not found");
  //   delay(1000);
  // }
  particleSensor.begin(Wire, I2C_SPEED_FAST);                                                    // Use default I2C port, 400kHz speed
  byte ledBrightness = 50;                                                                       // Options: 0=Off to 255=50mA
  byte sampleAverage = 1;                                                                        // Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2;                                                                              // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100;                                                                         // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 69;                                                                           // Options: 69, 118, 215, 411
  int adcRange = 4096;                                                                           // Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A);                                                     // Turn Red LED to low to indicate sensor is running

  Wire.setClock(100000);
}

int interval = 5000;
int id = 1;
unsigned long sendTimeStamp = 0;

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  if (isMeasurement)
  {
    //---------1. SUHU-----------
    if (millis() - temperatureTimeStamp < temperatureInterval &&
        isAllowMeasureTemperature)
    {

      // lcd ONCE
      if (isAllowTolcdTemperature)
      {
        Serial.println("Lakukan pengukuran suhu");
        delay(3000);
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Ukur Suhu Tubuh");
        lcd.setCursor(0, 1);
        lcd.print("Suhu: ");
        isAllowTolcdTemperature = false;
      }
      body_temp = mlx.readObjectTempC() + 2.253;
      lcd.setCursor(6, 1);
      lcd.print(body_temp);
      lcd.print("*C");

      Serial.print("Emissivity = ");
      Serial.println(mlx.readEmissivity());
      Serial.print("Suhu tubuh = ");
      Serial.println(body_temp);
      temperatureDataCounter += 1;
      temperatureTotalData += body_temp;
    }

    if (millis() - temperatureTimeStamp > temperatureInterval &&
        isAllowMeasureTemperature)
    {

      isAllowMeasureTemperature = false;
      isAllowMeasureHeartRate = true;
      isAllowTolcdHeartRate = true;
      heartRateTimeStamp = millis();
      Serial.println("Pengukuran dengan MLX90614 dihentikan.");
      delay(3000);
    }

    //---------2. HR -----------
    if (millis() - heartRateTimeStamp < heartRateInterval &&
        isAllowMeasureHeartRate)
    {

      // lcd ONCE
      if (isAllowTolcdHeartRate)
      {
        Serial.println("Pengukuran HR");
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Ukur HR");
        // delay(5000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("HeartRate");
        isAllowTolcdHeartRate = false;
      }

      // sensorMax();
      // LAKUKAN PERHITUNGAN RATA-RATA MAX DI SINI
      long irValue = particleSensor.getIR();
      Serial.print("IR=");
      Serial.println(irValue);

      if (checkForBeat(irValue) == true)
      {
        // We sensed a beat!
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);
        Serial.print(", BPM=");
        Serial.println(beatsPerMinute);

        if (beatsPerMinute < 255 && beatsPerMinute > 20)
        {
          rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
          rateSpot %= RATE_SIZE;                    // Wrap variable

          // Take average of readings
          beatAvg = 0;
          for (byte x = 0; x < RATE_SIZE; x++)
            beatAvg += rates[x];
          beatAvg /= RATE_SIZE;

          Serial.print(", Avg BPM=");
          Serial.println(beatAvg);
          lcd.setCursor(5, 1);
          lcd.print(beatAvg);
          lcd.write(byte(0));
        }
      }
    }

    if (millis() - heartRateTimeStamp > heartRateInterval &&
        isAllowMeasureHeartRate)
    {
      isAllowMeasureHeartRate = false;
      Serial.println("Pengukuran HR dihentikan.");
      // isAllowToSendData = true;
      isAllowMeasureOxy = true;
      isAllowTolcdOxy = true;
      oxyTimeStamp = millis();
    }

    // 3 --- Oxygen
    if (isAllowMeasureOxy == true)
    {
      // lcd ONCE
      if (isAllowTolcdOxy)
      {
        Serial.println("Pengukuran SpO2");
        lcd.clear();
        lcd.setCursor(1, 0);
        lcd.print("Ukur SpO2 ");
        // delay(5000);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SpO2");
        lcd.print(" | ");
        lcd.print("Tahan..");
        isAllowTolcdOxy = false;
      }

      // Lakukan Pengukuran Oksigen
      bufferLength = 100; // buffer length of 100 stores 4 seconds of samples running at 25sps

      // read the first 100 samples, and determine the signal range
      for (byte i = 0; i < bufferLength; i++)
      {
        while (particleSensor.available() == false) // do we have new data?
          particleSensor.check();                   // Check the sensor for new data

        redBuffer[i] = particleSensor.getRed();
        irBuffer[i] = particleSensor.getIR();
        particleSensor.nextSample(); // We're finished with this sample so move to next sample

        Serial.print(F("red="));
        Serial.print(redBuffer[i], DEC);
        Serial.print(F(", ir="));
        Serial.println(irBuffer[i], DEC);
      }

      // calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
      maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

      // Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
      while (1 && particleSensor.getIR() >= 100000)
      {
        // dumping the first 30 sets of samples in the memory and shift the last 70 sets of samples to the top
        for (byte i = 30; i < 100; i++)
        {
          redBuffer[i - 30] = redBuffer[i];
          irBuffer[i - 30] = irBuffer[i];
        }

        // take 70 sets of samples before calculating the heart rate.
        for (byte i = 70; i < 100; i++)
        {
          while (particleSensor.available() == false) // do we have new data?
            particleSensor.check();                   // Check the sensor for new data

          digitalWrite(readLED, !digitalRead(readLED)); // Blink onboard LED with every data read

          redBuffer[i] = particleSensor.getRed();
          irBuffer[i] = particleSensor.getIR();
          particleSensor.nextSample(); // We're finished with this sample so move to next sample

          // send samples and calculation result to terminal program through UART

          // int32_t trueHR = heartRate - 85.2;
          // int32_t trueOxy = spo2 - 2.4;

          if (validSPO2 == 1 && spo2 >= 88)
          {
            Serial.print(F("red="));
            Serial.print(redBuffer[i], DEC);
            Serial.print(F(", ir="));
            Serial.print(irBuffer[i], DEC);

            Serial.print(F(", SPO2="));
            Serial.print(spo2, DEC);

            Serial.print(F(", SPO2Valid="));
            Serial.println(validSPO2, DEC);

            // lcd.print(" | ");
            lcd.setCursor(5, 1);
            lcd.print(spo2);
            lcd.print(" %");

            // heartDataCounter += 1;
            // heartTotalData += trueHR;
            oxyDataCounter += 1;
            oxyTotalData += spo2;

            if (oxyDataCounter == OXY_SAMPLE)
            {
              isAllowMeasureOxy = false;
              Serial.println("Pengukuran dengan MAX30102 SpO2 dihentikan.");
              isAllowToSendData = true;
              break;
            }
          }
        }

        if (oxyDataCounter == OXY_SAMPLE)
        {
          isAllowMeasureOxy = false;
          Serial.println("Pengukuran dengan MAX30102 SpO2 dihentikan.");
          isAllowToSendData = true;
          break;
        }

        // //After gathering 25 new samples recalculate HR and SP02
        maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
      }
    }

    if (isAllowToSendData)
    {

      // LAKUKAN PERHITUNGAN RATA-RATA MLX DI SINI
      long tempavg = temperatureTotalData / temperatureDataCounter;
      Serial.print("Rata-rata Suhu:");
      Serial.println(tempavg);

      String tempavg_str;
      tempavg_str = String(tempavg);

      long heartavg = beatAvg;
      Serial.print("Rata-rata HR:");
      Serial.println(heartavg);

      String heartavg_str;
      heartavg_str = String(heartavg);

      Serial.print("Total SpO2: ");
      Serial.println(oxyDataCounter);
      Serial.print("Measurement Taken: ");
      Serial.println(oxyTotalData);

      long oxyavg;
      if (oxyDataCounter > 0)
      {
        oxyavg = oxyTotalData / oxyDataCounter;
        Serial.print("Rata-rata SpO2:");
        Serial.println(oxyavg);
      }

      String oxyavg_str;
      oxyavg_str = String(oxyavg);

      // MASUKAN DATA HASIL PERHITUNGAN UNTUK DIKIRIM KE SERVER

      if (!client.connected())
      {
        reconnect();
      }
      client.loop();

      delay(250);

      String dataToSend =
          "{\"heartRate\" : \"" + heartavg_str + "\", \"spo2\" : \"" + oxyavg_str + "\", \"temperature\" : "
                                                                                    "\"" +
          tempavg_str + "\",\"deviceId\" : \"" +
          DEVICE_ID + "\",\"no\" : \"1\"}";
      const char *dataToSend_chr = dataToSend.c_str();
      String publishTopic = "/session/record/" + DEVICE_ID;
      const char *publishTopic_chr = publishTopic.c_str();
      client.publish(publishTopic_chr, dataToSend_chr);

      delay(2000);
      lcd.clear();
      lcd.print("Berhasil dikirim");
      delay(2000);

      String finishData = "{\"deviceId\" : \"" + DEVICE_ID + "\"}";
      const char *finishData_chr = finishData.c_str();
      String publishFinish = "/session/stop/" + DEVICE_ID;
      const char *publishFinish_chr = publishFinish.c_str();
      client.publish(publishFinish_chr, finishData_chr);
      Serial.println("Mengirim Data Ke Server");

      isAllowToSendData = false;
      isMeasurement = false;
      temperatureDataCounter = 0;
      temperatureTotalData = 0;
      // oxyDataCounter = 0;
      // oxyTotalData = 0;
      // heartDataCounter = 0;
      // heartTotalData = 0;
      delay(500);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(DEVICE_ID);
      lcd.setCursor(0, 1);
      lcd.print("Device Standby");
    }
  }
}
