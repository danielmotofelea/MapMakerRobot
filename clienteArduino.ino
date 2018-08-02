#include <WiFiEsp.h>
#include <WiFiEspClient.h>
#include "SoftwareSerial.h"
#include <PubSubClient.h>
#include <NewPing.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MPU9250_asukiaaa.h>

#ifdef _ESP32_HAL_I2C_H_ //Se inicializa I2C para el IMU
#define SDA_PIN A4
#define SCL_PIN A5
#endif

#define numUltra     3 // Numero de sensores ultrasonidos

#define pinTemp 4     // Pin del sensor de temperatura y humedad
#define DHTTYPE DHT11 // Tipo de sensor que utilizamos

#define enableMotores 11 // Enable para los dos ejes del robot

#define ejeIzqPos 5      // Pin PWM para salida positiva del eje izquierdo del robot
#define ejeIzqNeg 9      // Pin PWM para salida negativa del eje izquierdo del robot

#define ejeDerPos 6     // Pin PWM para salida positiva del eje derecho del robot
#define ejeDerNeg 10    // Pin PWM para salida positiva del eje derecho del robot

int valorUltra[numUltra]; // Almacenamiento de distancias de los ultrasonidos

float temp;   // DHT11
float hum;    // DHT11
float aX, aY; // IMU

int velocidad;

MPU9250 mySensor;               // IMU
DHT dht(pinTemp, DHTTYPE);      // Creacion del objeto del sensor de temp/hum

NewPing sonar[numUltra] = { // Sensor object array.
  NewPing(13, 7),
  NewPing(13, 8),
  NewPing(13, 12)
};

IPAddress server(192, 168, 1, 131);     // Direccion IP del Broker
char ssid[] = "JAZZTEL_dbhm";           // Nombre del Punto de Acceso
char pass[] = "enkgws4ndjxu";           // Contrasena
int status = WL_IDLE_STATUS;            // Estado del WiFi

// Creacion del cliente ESP
WiFiEspClient espClient;
PubSubClient client(espClient);
SoftwareSerial soft(3, 2); // RX, TX

void setup() {

  while (!Serial);

#ifdef _ESP32_HAL_I2C_H_ // IMU
  Wire.begin(SDA_PIN, SCL_PIN); 
#else
  Wire.begin();
#endif

  mySensor.setWire(&Wire);
  mySensor.beginAccel(ACC_FULL_SCALE_2_G);

  velocidad = 0;

  pinMode(ejeIzqPos, OUTPUT); // Inicializamos los pines de los motores como salidas
  pinMode(ejeIzqNeg, OUTPUT);
  pinMode(ejeDerPos, OUTPUT);
  pinMode(ejeDerNeg, OUTPUT);
  pinMode(enableMotores, OUTPUT);

  dht.begin(); // Iniciamos el sensor de temperatura y humedad.

  soft.begin(9600);    // Inicializamos el serial con el ESP 
  WiFi.init(&soft);    // Inicializamos el modulo

  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    // don't continue
    while (true);
  }

  // Se intenta conectar al PA
  while ( status != WL_CONNECTED) {
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }

  // Conexion con el Broker
  client.setServer(server, 1883);
  client.setCallback(callback);

  client.subscribe("robot");

}

void loop()
{
  if (!client.connected()) {

    if (client.connect("Cliente Arduino")) {
      client.subscribe("robot");

    } else {
      delay(2000);
    }
  }
  // Cliente a la escucha
  //Serial.print("Empieza el loop del cliente MQTT")
  client.loop();
  //Serial.print("Termina el loop del cliente MQTT")
}

void recogerValoresUltrasonidos() {

  for (int i = 0; i < numUltra; i++) {
    valorUltra[i] = 0;
    delay(50);
    valorUltra[i] = sonar[i].ping_cm();
  }
}

void recogerValoresDHT() {

  temp = dht.readTemperature();
  hum = dht.readHumidity();

}

void recogerValoresMPU() {

  mySensor.accelUpdate();
  aX = mySensor.accelX();
  aY = mySensor.accelY();
}

void enviarDatos() {

  char buffer[10];

  client.publish("robot/distancia/u_izq", dtostrf(valorUltra[0], 0, 0, buffer));
  client.publish("robot/distancia/u_cen", dtostrf(valorUltra[1], 0, 0, buffer));
  client.publish("robot/distancia/u_der", dtostrf(valorUltra[2], 0, 0, buffer));

  client.publish("robot/dht/hum", dtostrf(hum, 0, 0, buffer));
  client.publish("robot/dht/temp", dtostrf(temp, 0, 0, buffer));

  client.publish("robot/posicion/x", dtostrf(aX, 0, 4, buffer));
  client.publish("robot/posicion/y", dtostrf(aY, 0, 4, buffer));
}

void callback(char* topic, byte* payload, unsigned int length) {

  payload[length] = '\0';
  String s = String((char*)payload);

  velocidad = s.toInt();

  ajustarVelocidad();
  recogerValoresUltrasonidos();
  recogerValoresDHT();

  enviarDatos();
}

void pararMapeo() {
  digitalWrite(enableMotores, LOW);

}

void iniciarMapeo() {
  digitalWrite(enableMotores, HIGH);
}
void ajustarVelocidad() {

  int velocidadEjeIzq = 0;
  int velocidadEjeDer = 0;
  int velocidadBase = 128;

  bool positivoDer = false;
  bool positivoIzq = false;

  velocidadEjeIzq = velocidadBase - (velocidad);
  velocidadEjeDer = velocidadBase + (velocidad);

  if (velocidadEjeIzq >= 0) {
    positivoIzq = true;
    if (velocidadEjeIzq > 255)
      velocidadEjeIzq = 255;
  }
  else {
    positivoIzq = false;
    if (velocidadEjeIzq < -255) {
      velocidadEjeIzq = 255;
    }
    else
      velocidadEjeIzq = velocidadEjeIzq - (2 * velocidadEjeIzq);
  }

  if (velocidadEjeDer >= 0) {
    positivoDer = true;
    if (velocidadEjeDer > 255)
      velocidadEjeDer = 255;
  }
  else {
    positivoDer = false;
    if (velocidadEjeDer < -255) {
      velocidadEjeDer = 255;
    }
    else
      velocidadEjeDer = velocidadEjeDer - (2 * velocidadEjeDer);
  }

  iniciarMapeo();

  if (positivoIzq) {
    analogWrite(ejeIzqPos, velocidadEjeIzq);
    analogWrite(ejeIzqNeg, 0);
  }
  else {
    analogWrite(ejeIzqPos, 0);
    analogWrite(ejeIzqNeg, velocidadEjeIzq);
  }

  if (positivoDer) {
    analogWrite(ejeDerPos, velocidadEjeDer);
    analogWrite(ejeDerNeg, 0);
  }
  else {
    analogWrite(ejeDerPos, 0);
    analogWrite(ejeDerNeg, velocidadEjeIzq);
  }

  delay (500);
  recogerValoresMPU();
  pararMapeo();

}

