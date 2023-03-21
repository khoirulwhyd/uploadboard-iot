#include <Arduino.h>
#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ShiftRegister74HC595.h>
#include <SimpleDHT.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>
#include <SimpleTimer.h>

#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
DynamicJsonDocument JSON(1024);

#define pinBuzz 9
#define pinRelay 10

#define pinDHT 10
#define pinEcho 12
#define pinTrigger 14
#define pinLDR A0
#define pinFan 15
#define pinIR 13
#define pinData 16
#define pinClock 2
#define pinLatch 0
#define pubQos 2

// const char *wifiName = "Sugeng Pras";
// const char *wifiPass = "sugeng123";
// const char *wifiName = "JTI-POLINEMA";
// const char *wifiPass = "jtifast!";
const char *wifiName = "WARJO";
const char *wifiPass = "ojotakonaku";
const char *brokerUser = "guest";
const char *brokerPass = "guest";
const char *brokerHost = "broker.mqttdashboard.com"; // tefa
// const char *brokerUser = "guest";
// const char *brokerPass = "guest";
// const char *brokerHost = "168.138.160.59"; // vedc
const uint16_t brokerPort = 1883;
// const char *brokerHost = "168.168.74.32"; // MQTT
// const char *brokerHost = "168.138.182.140"; // RabbitMQ
String MAC;

const char *outTopicJSON = "sensor";

const char *outTopicIR = "remoteir";
const char *inTopicFAN = "fanpwm";
const char *inTopicRelay = "relay";
const char *inTopicLED = "ledanim";
const char *inTopicPiezo = "piezo";

Adafruit_SSD1306 display(128, 64, &Wire, -1);

ShiftRegister74HC595<2> srChannel(pinData, pinClock, pinLatch);
SimpleDHT11 dht11(pinDHT);
IRrecv PenerimaIR(pinIR);
decode_results hasil;
SimpleTimer TimerJSON;

WiFiClient espClient;
PubSubClient client(espClient);
long lastReconnectAttempt = 0;

typedef enum
{
  AnimKiriKanan,
  AnimKiriKananSendirian,
  AnimTengahSamping,
  AnimSampingTengah
} animLED;

byte humValid, tempValid;
float ligValid, disValid;
unsigned int KodeTombolRemote;

void KontrolKecepatanFan();
void SpeedFANSub(int speedFAN);
int SensorLDR();
int SensorJarakUltraSonic();
void SensorDHT();
String msgJSON();
void beepBuzz(unsigned char delayms);
void runningLED(animLED al, int tunda);
void BacaKodeRemoteIR();
void MematikanSemuaLED();
void HidupkanLEDDenganIR();
void KoneksiWIFI();
void reconnect();
void callback(char *topic, byte *payload, unsigned int length);
void updateOLED();

void setup()
{
  Serial.begin(115200);
  Wire.begin();

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.clearDisplay();
  MAC = WiFi.macAddress();
  Serial.println(MAC);
  KoneksiWIFI();

  client.setServer(brokerHost, brokerPort);
  client.setCallback(callback);

  PenerimaIR.enableIRIn();

  pinMode(pinTrigger, OUTPUT);
  pinMode(pinEcho, INPUT);
  pinMode(pinFan, OUTPUT);
  pinMode(pinDHT, INPUT);
  srChannel.setAllLow();

  TimerJSON.setInterval(1500);
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  BacaKodeRemoteIR();

  if (TimerJSON.isReady())
  {
    SensorDHT();
    SensorJarakUltraSonic();
    SensorLDR();

    JSON["suhu"] = tempValid;
    JSON["kelembapan"] = humValid;
    JSON["cahaya"] = ligValid;
    JSON["jarak"] = disValid;
    JSON["macAdd"] = WiFi.macAddress();

    char buffer[1000];
    size_t n = serializeJson(JSON, buffer);

    //    String payload = "{";
    //        payload += "\"Humidity\":"; payload += char(humValid); payload += ",";
    //        payload += "\"Temperature\":"; payload += char(tempValid); payload += ",";
    //        payload += "\"Light\":"; payload += char(ligValid); payload += ",";
    //        payload += "\"Distance\":"; payload += char(disValid); payload += ",";
    //        payload += "\"MAC-Add\":"; payload += String(WiFi.macAddress());
    //        payload += "}";
    //
    //    char attributes[100];

    //    payload.toCharArray( attributes, 100 );
    //    client.publish(outTopicJSON, attributes);

    //    snprintf(msg, MSG_BUFFER_SIZE, "%s", msgJSON());
    //    client.publish(outTopicJSON, msg);
    updateOLED();
    client.publish(outTopicJSON, buffer, n);
    //    client.publish(outTopicJSON, JSON, pubQos, true);
    //    tambah pramater poss publish
    // tambahin qos di pusbliser
    Serial.print("Publish message: ");
    //    Serial.print(payload);
    //    Serial.println(n);

    TimerJSON.reset();
    delay(1000);
  }
}

// ------------method------------------------

// ------------1.
void KontrolKecepatanFan()
{
  int speedfan = map(SensorJarakUltraSonic(), 30, 1, 0, 1023);
  analogWrite(pinFan, speedfan);
  Serial.println("Speed FAN: " + String(speedfan));
}

// ------------2.
void SpeedFANSub(int speedFAN)
{
  int speedfan = map(speedFAN, 1, 100, 0, 1023);
  analogWrite(pinFan, speedfan);
}

// ------------3.
int SensorLDR()
{
  int nilaiAnalogLDR = analogRead(pinLDR);
  double Vout = nilaiAnalogLDR * 0.0032258064516129;
  int lux = 330 / (10 * ((3.3 - Vout) / Vout));
  if (lux != 0)
  {
    ligValid = lux;
  }
  Serial.println("Lux Intensity= " + String(int(lux)));
  return lux;
}

// ------------4.
int SensorJarakUltraSonic()
{
  digitalWrite(pinTrigger, LOW);
  delayMicroseconds(2);
  digitalWrite(pinTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pinTrigger, LOW);
  long Durasi = pulseIn(pinEcho, HIGH);
  int JarakCM = Durasi * 0.034 / 2;    // Jarak dalam satuan CM
  int JarakInch = Durasi * 0.0133 / 2; // Jarak dalam satuan INCH

  if (JarakCM != 0)
  {
    disValid = JarakCM;
  }
  Serial.println("Jarak Cm=" + String(JarakCM) + " Inch=" + String(JarakInch));
  return JarakCM;
}

// ------------5.
void SensorDHT()
{
  byte suhu = 0;
  byte hum = 0;

  int err = SimpleDHTErrSuccess;
  if ((err = dht11.read(&suhu, &hum, NULL)) != SimpleDHTErrSuccess)
  {
    Serial.print("Read DHT11 failed, err=");
    Serial.println(err);
    delay(100);
    return;
  }

  if (suhu != 0 || hum != 0)
  {
    tempValid = suhu;
    humValid = hum;
  }

  Serial.println("Sample OK: ");
  Serial.print("Suhu: ");
  Serial.print((int)tempValid);
  Serial.print(" *C, ");
  Serial.print("Kelembapan: ");
  Serial.print((int)humValid);
  Serial.println(" H");
}

// -----------6.
void beepBuzz(unsigned char delayms)
{
  srChannel.set(pinBuzz, HIGH);
  delay(delayms);
  srChannel.set(pinBuzz, LOW);
  delay(delayms);
}

// -----------7.
void runningLED(animLED al, int tunda)
{
  MematikanSemuaLED();
  switch (al)
  {
  // LED hidup dari kiri ke kanan
  case AnimKiriKanan:
    for (uint8_t i = 0; i <= 8; i++)
    {
      srChannel.set(i, HIGH);
      delay(tunda);
    }
    break;

  // LED hidup dari kanan ke kiri
  case AnimKiriKananSendirian:
    for (uint8_t i = 0; i <= 8; i++)
    {
      if (i > 0)
        srChannel.set(i - 1, LOW);
      srChannel.set(i, HIGH);
      delay(tunda);
    }
    break;

  case AnimTengahSamping:
    srChannel.set(4, HIGH);
    delay(tunda);
    srChannel.set(3, HIGH);
    srChannel.set(5, HIGH);
    delay(tunda);
    srChannel.set(2, HIGH);
    srChannel.set(6, HIGH);
    delay(tunda);
    srChannel.set(1, HIGH);
    srChannel.set(7, HIGH);
    delay(tunda);
    srChannel.set(0, HIGH);
    srChannel.set(8, HIGH);
    delay(tunda);
    break;

  // LEH hidup dari samping ke tengah
  case AnimSampingTengah:
    srChannel.set(0, HIGH);
    srChannel.set(8, HIGH);
    delay(tunda);
    srChannel.set(1, HIGH);
    srChannel.set(7, HIGH);
    delay(tunda);
    srChannel.set(2, HIGH);
    srChannel.set(6, HIGH);
    delay(tunda);
    srChannel.set(3, HIGH);
    srChannel.set(5, HIGH);
    delay(tunda);
    srChannel.set(4, HIGH);
    delay(tunda);
    break;
  }

  MematikanSemuaLED();
}

// ------------8.

void BacaKodeRemoteIR()
{
  if (PenerimaIR.decode(&hasil))
  {
    String ngatasiDebounce = String((int)hasil.value, (unsigned char)DEC);
    if (ngatasiDebounce.length() == 8)
    {
      KodeTombolRemote = hasil.value;
      Serial.println("Kode remote: " + String(KodeTombolRemote));

      snprintf(msg, MSG_BUFFER_SIZE, "Anda menekan keypad dengan kode: %d", KodeTombolRemote);
      client.publish(outTopicIR, msg);
    }
    else
    {
      unsigned int kodeGagal = hasil.value;
      Serial.println("Length Decode: " + String(ngatasiDebounce.length()));
      Serial.println("Kode remote tidak diproses: " + String(kodeGagal));
    }
    PenerimaIR.resume();
  }
  delay(100);
}

// ------------9.
void MematikanSemuaLED()
{
  for (uint8_t i = 0; i <= 8; i++)
  {
    srChannel.set(i, LOW);
  }
}

// -------------10.
void HidupkanLEDDenganIR()
{
  MematikanSemuaLED();

  if (KodeTombolRemote == 1303529910)
  {
    // hidupkan LED 1
    // dengan tombol 1
    srChannel.set(0, HIGH);
  }
  else if (KodeTombolRemote == 1303562550)
  {
    // hidupkan LED 1, 2
    // dengan tombol 2
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
  }
  else if (KodeTombolRemote == 1303524300)
  {
    // hidupkan LED 1, 2, 3
    // dengan tombol 3
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
  }
  else if (KodeTombolRemote == 1303540110)
  {
    // hidupkan LED 1, 2, 3, 4
    // dengan tombol 4
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
  }
  else if (KodeTombolRemote == 1303572750)
  {
    // hidupkan LED 1, 2, 3, 4, 5
    // dengan tombol 5
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
    srChannel.set(4, HIGH);
  }
  else if (KodeTombolRemote == 1303516140)
  {
    // hidupkan LED 1, 2, 3, 4, 5, 6
    // dengan tombol 6
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
    srChannel.set(4, HIGH);
    srChannel.set(5, HIGH);
  }
  else if (KodeTombolRemote == 1303531950)
  {
    // hidupkan LED 1, 2, 3, 4, 5, 6, 7
    // dengan tombol 7
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
    srChannel.set(4, HIGH);
    srChannel.set(5, HIGH);
    srChannel.set(6, HIGH);
  }
  else if (KodeTombolRemote == 1303564590)
  {
    // hidupkan LED 1, 2, 3, 4, 5, 6, 7, 8
    // dengan tombol 8
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
    srChannel.set(4, HIGH);
    srChannel.set(5, HIGH);
    srChannel.set(6, HIGH);
    srChannel.set(7, HIGH);
  }
  else if (KodeTombolRemote == 1303520220)
  {
    // hidupkan LED 1, 2, 3, 4, 5, 6, 7, 8, 9
    // dengan tombol 9
    srChannel.set(0, HIGH);
    srChannel.set(1, HIGH);
    srChannel.set(2, HIGH);
    srChannel.set(3, HIGH);
    srChannel.set(4, HIGH);
    srChannel.set(5, HIGH);
    srChannel.set(6, HIGH);
    srChannel.set(7, HIGH);
    srChannel.set(8, HIGH);
  }
}

// -----------11.

void KoneksiWIFI()
{
  Serial.print("Connecting to ");
  Serial.println(wifiName);

  display.clearDisplay();

  display.setCursor(0, 0);
  display.println("Connecting to WiFi...");
  display.display();

  WiFi.mode(WIFI_STA);
  WiFi.begin(wifiName, wifiPass);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  display.setCursor(0, 12);
  display.println("WiFi connected");
  display.display();
  delay(1000);
  display.setCursor(0, 24);
  display.println(WiFi.localIP());
  display.display();
  delay(1000);
  display.setCursor(0, 36);
  display.println("IoT Smart Device");
  display.display();
  delay(1000);
  display.setCursor(0, 48);
  display.println("Development Board");
  display.display();
  delay(3000);
}

// ----------12.
void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    //    if (client.connect("ESP8266Client", brokerUser, brokerPass, 0, 2, 0, 0))//jenis QoS = 2
    if (client.connect(MAC.c_str(), brokerUser, brokerPass))
    {

      Serial.println("connected");

      // memastikan bahwa IoT Development Board
      // telah meng-subscribe semua aktuator
      //      client.subscribe("/led");
      client.subscribe(inTopicFAN);
      client.subscribe(inTopicRelay);
      client.subscribe(inTopicLED);
      client.subscribe(inTopicPiezo);
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");

      delay(5000);
    }
  }
}

// ----------13.
void callback(char *topic, byte *payload, unsigned int length)
{
  // variable StringPayload untuk menyimpan konten paket data yang diterima
  String StringPayload = "";

  // Menjadikan setiap character yang diterima menjadi string utuh
  // melalui proses penggabungan character
  for (int i = 0; i < length; i++)
  {
    StringPayload += (char)payload[i];
  }

  Serial.println("TOPIC: " + String(topic));
  Serial.println("PAYLOAD: " + String(StringPayload));

  // Mem-filter data berdasarkan nama topic nya masing-masing
  if (strcmp(topic, inTopicFAN) == 0)
  {
    // Topic: "/fanpwm"
    SpeedFANSub(StringPayload.toInt());
  }
  else if (strcmp(topic, inTopicRelay) == 0)
  {
    // Topic: "relay"
    if (String(StringPayload) == "ON")
    {
      // Mengaktifkan Relay jika StringPayload = "ON"
      srChannel.set(pinRelay, HIGH);
      Serial.println("Relay ON");
    }
    else
    {
      // Menonaktifkan Relay jika StringPayload = "OFF"
      srChannel.set(pinRelay, LOW);
      Serial.println("Relay OFF");
    }
  }
  else if (strcmp(topic, inTopicPiezo) == 0)
  {
    // Topic: "/piezo"
    if (String(StringPayload) == "ON")
    {
      // Mengaktifkan Buzz jika StringPayload = "ON"
      srChannel.set(pinBuzz, HIGH);
      Serial.println("Buzzer ON");
    }
    else
    {
      // Menonaktifkan Buzz jika StringPayload = "OFF"
      srChannel.set(pinBuzz, LOW);
      Serial.println("Buzzer OFF");
    }
  }
  else if (strcmp(topic, inTopicLED) == 0)
  {
    // Topic: "/ledanim"
    for (int i = 0; i <= 8; i++)
    {
      // Menset status channel shift register mulai dari 0-8 menjadi LOW
      srChannel.set(i, LOW);
    }

    for (int i = 1; i <= StringPayload.toInt(); i++)
    {
      // Menset status channel shift register mulai dari 0
      // sampai nilai maks yang diterima dari payload menjadi HIGH
      srChannel.set(i - 1, HIGH);
    }
  }
}

// ----------14.
void updateOLED()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);

  display.setCursor(0, 0);
  display.print("Light");
  display.setCursor(75, 0);
  display.print(String(SensorLDR()));
  display.setCursor(100, 0);
  display.print("Lux");

  display.setCursor(0, 14);
  display.print("Temperature");
  display.setCursor(75, 14);
  display.print(String(tempValid));
  display.drawCircle(100, 13, 2, SSD1306_WHITE);
  display.setCursor(105, 14);
  display.print("C");

  display.setCursor(0, 28);
  display.print("Humidity");
  display.setCursor(75, 28);
  display.print(String(humValid));
  display.setCursor(100, 28);
  display.print("H");

  display.setCursor(0, 42);
  display.print("Distance");
  display.setCursor(75, 42);
  display.print(String(SensorJarakUltraSonic()));
  display.setCursor(100, 42);
  display.print("CM");

  display.display();
}

// ------------15.
String msgJSON()
{
  String payload = "{";
  payload += "\"Humidity\":";
  payload += char(humValid);
  payload += ",";
  payload += "\"Temperature\":";
  payload += char(tempValid);
  payload += ",";
  payload += "\"Light\":";
  payload += char(SensorLDR());
  payload += ",";
  payload += "\"Distance\":";
  payload += char(SensorJarakUltraSonic());
  payload += ",";
  payload += "\"MAC-Add\":";
  payload += MAC;
  payload += "}";
  return payload;
}