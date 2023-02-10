#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include "DHT.h"
#include <WiFi.h>
#include <esp_now.h>

#define DHTPIN 14
#define vent 12
#define DHTTYPE DHT11   // DHT 11

DHT dht(DHTPIN, DHTTYPE);

float temperatura = 5.00;
int temperatura2=0;
int led = 0;

//parametros del envio
uint8_t broadcastAddress[] = {0xc0, 0x49, 0xef, 0xca, 0x38, 0xd0};

//estructura y variable del mensaje
typedef struct struct_message {
  int id;
  int a;
  int b;
} struct_message;

struct_message myData;

esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status){
  Serial.print("\r\nEstado del ultimo paquete enviado:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Enviado con exito" : "Fallo en el envio");
}

void setup() {
  Serial.begin(9600);

  pinMode(vent, OUTPUT);

  Serial.println(F("DHTxx test!"));
  dht.begin();

  WiFi.mode(WIFI_STA);

  if(esp_now_init() != ESP_OK){
    Serial.println("Error inicializando ESP-NOW");
    return;
  }

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if(esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Fallo al añadir peer");
    return;
  }
}

void loop() {
  
  delay(3000);

  float h = dht.readHumidity();
  
  float t = dht.readTemperature();
  
  float f = dht.readTemperature(true);

  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println(F("Failed to read from DHT sensor!"));
    return;
  }

  float hif = dht.computeHeatIndex(f, h);

  float hic = dht.computeHeatIndex(t, h, false);

  Serial.print(F("Humidity: "));
  Serial.print(h);
  Serial.print(F("%  Temperature: "));
  Serial.print(t);
  Serial.print(F("°C "));
  Serial.print(f);
  Serial.print(F("°F  Heat index: "));
  Serial.print(hic);
  Serial.print(F("°C "));
  Serial.print(hif);
  Serial.println(F("°F"));

  if (t>=23.00){
    digitalWrite(vent, HIGH);
    led=1;
  } else {
    digitalWrite(vent, LOW);
    led=0;
  }

  temperatura=t;
  temperatura2=(int)temperatura;
  myData.b = temperatura2;
  myData.a = led;
  myData.id = 2;

  esp_now_register_send_cb(OnDataSent);
  
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
  if(result == ESP_OK){
    Serial.println("enviado con exito");
  }
  else{
    Serial.println("fallo en el envio");
  }
}
