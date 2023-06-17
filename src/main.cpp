#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include <PubSubClient.h>
#include <WiFi.h>

boolean isMeasurement = false;
boolean isAllowMeasureTemprature = false;
boolean isAllowMeasureMLX30102 = false;
boolean isAllowToSendData = false;
boolean isAllowToDisplayTemprature = false;
boolean isAllowToDisplayMLX30102 = false;

String SSID = "vivo T1 Pro 5G";
String PWD = "12345678";
String DEVICE_ID = "fa05d";
long temperatureDataCounter = 0;
long temperatureTotalData = 0;
unsigned long temperatureTimeStamp = 0;
unsigned long temperatureInterval = 3000;
unsigned long max30102TimeStamp = 0;
unsigned long max30102Interval = 10000;
const char *MQTT_SERVER = "192.168.183.130";

WiFiClient espClient;
PubSubClient client(espClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);

void callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.println();

  String topic_listening = "/session/start/" + DEVICE_ID;
  const char *topic_listening_char = topic_listening.c_str();
  if (topic_listening == topic) {
    Serial.println("Mulai pengukuran");
    isMeasurement = true;
    temperatureTimeStamp = millis();
    isAllowMeasureTemprature = true;
    isAllowToDisplayTemprature = true;
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("esp", "dimasaulia", "t4np454nd1")) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      String topic_listening = "/session/start/" + DEVICE_ID;
      const char *topic_listening_char = topic_listening.c_str();
      client.subscribe(topic_listening_char);
      Serial.println("Subscribe to " + topic_listening);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  WiFi.begin("vivo T1 Pro 5G", "12345678");
  lcd.init();

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("Wifi Connected.");
  Serial.println(WiFi.localIP());
  Serial.println("WORK");
  client.setServer(MQTT_SERVER, 1883);
  client.setCallback(callback);

  lcd.setBacklight(1);
  lcd.setContrast(2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(DEVICE_ID);
  lcd.setCursor(0, 1);
  lcd.print("Device Standby");
}

int interval = 5000;
int id = 1;
unsigned long sendTimeStamp = 0;
void loop() {
  if (!client.connected()) {
    reconnect();
  }

  if (isMeasurement) {

    if (millis() - temperatureTimeStamp < temperatureInterval &&
        isAllowMeasureTemprature) {
      Serial.println("Lakukan pengukuran suhu");

      // DISPLAY ONCE
      if (isAllowToDisplayTemprature) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Pengukuran Suhu");
        lcd.setCursor(0, 1);
        lcd.print("Suhu Anda: 30");
        isAllowToDisplayTemprature = false;
      }

      // LAKUKAN PENGUKURAN SUHU DISINI
    }

    if (millis() - temperatureTimeStamp > temperatureInterval &&
        isAllowMeasureTemprature) {
      isAllowMeasureTemprature = false;
      isAllowMeasureMLX30102 = true;
      isAllowToDisplayMLX30102 = true;
      max30102TimeStamp = millis();
      Serial.println("Pengukuran suhu dihentikan");
    }

    if (millis() - max30102TimeStamp < max30102Interval &&
        isAllowMeasureMLX30102) {
      Serial.println("Pengukuran MLX30102");

      // DISPLAY ONCE
      if (isAllowToDisplayMLX30102) {
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("SpO2-HeartRate");
        lcd.setCursor(0, 1);
        lcd.print("SpO2 Anda: 30");
        isAllowToDisplayMLX30102 = false;
      }

      // LAKUKAN PENGUKURAN MLX DISINI
    }

    if (millis() - max30102TimeStamp > max30102Interval &&
        isAllowMeasureMLX30102) {
      isAllowMeasureMLX30102 = false;
      Serial.println("Berhenti mengukur MLX40102");
      isAllowToSendData = true;
    }

    if (isAllowToSendData) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Mengirim data");

      // LAKUKAN PERHITUANGAN RATA-RATA DISINI

      // MASUKAN DATA HASIL PERHITUNGAN UNTUK DIKIRIM KE SERVER
      String dataToSend =
          "{\"heartreat\" : \"90\", \"spo2\" : \"99\", \"temprature\" : "
          "\"30\",\"deviceId\" : \"" +
          DEVICE_ID + "\",\"no\" : \"1\"}";
      const char *dataToSend_chr = dataToSend.c_str();
      String publishTopic = "/session/record/" + DEVICE_ID;
      const char *publishTopic_chr = publishTopic.c_str();
      client.publish(publishTopic_chr, dataToSend_chr);

      delay(2000);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Berhasil mengirim");

      String finishData = "{\"deviceId\" : \"" + DEVICE_ID + "\"}";
      const char *finishData_chr = finishData.c_str();
      String publishFinish = "/session/stop/" + DEVICE_ID;
      const char *publishFinish_chr = publishFinish.c_str();
      client.publish(publishFinish_chr, finishData_chr);

      isAllowToSendData = false;
      isMeasurement = false;
      delay(500);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(DEVICE_ID);
      lcd.setCursor(0, 1);
      lcd.print("Device Standby");
    }
  }

  client.loop();
}
