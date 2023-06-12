/*****************
 * Program : Pengendalian Suhu dengan PID via IoT
 *           Menggunakan Kit iTCLab
 * Oleh    : Tim io-t.net
 * Surabaya, 25 April 2022
 *****************/

#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>

const char* ssid = "wifi"; // Masukkan nama WiFi Anda
const char* password =  "password_wifi"; // Masukkan kata sandi WiFi Anda

#define mqttServer "broker.hivemq.com"
#define mqttPort 1883

WiFiServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

String Topic;
String Payload;

// Konstanta
const int baud = 115200;       // Kecepatan baud serial

// Pin yang sesuai dengan sinyal pada iTCLab Shield
const int pinT1   = 34;         // T1
const int pinT2   = 35;         // T2
const int pinQ1   = 32;         // Q1
const int pinQ2   = 33;         // Q2
const int pinLED  = 26;         // LED
const int pinHeatSensor = 4;    // Pin untuk sensor suhu panas

// Pengaturan PWM
const int freq = 5000; //5000
const int ledChannel = 0;
const int Q1Channel = 1;
const int Q2Channel = 2;
const int resolutionLedChannel = 8; // Resolusi 8, 10, 12, 15
const int resolutionQ1Channel = 8;  // Resolusi 8, 10, 12, 15
const int resolutionQ2Channel = 8;  // Resolusi 8, 10, 12, 15

float cel, cel1, degC, degC1;
float P, I, D;
float KP, KI, KD, op0, ophi, oplo, error, dpv;

float sp = 0;      // Set point
float pv = 0;      // Suhu saat ini
float pv_last = 0; // Suhu sebelumnya
float ierr = 0;    // Error integral
float dt = 0;      // Waktu antara pengukuran
float op = 0;      // Output dari pengendali PID

int autoSet = 0;     // autoSet = 1 untuk pengaturan otomatis
float Kc = 10.0;     // K / %Pemanas
float tauI = 50.0;   // detik
float tauD = 1.0;    // detik

unsigned long ts = 0, new_ts = 0; // Penanda waktu
const float batas_suhu_atas = 58;

// Variabel global
float Q1 = 0;                 // Nilai yang ditulis ke pin Q1
float Q2 = 0;                 // Nilai yang ditulis ke pin Q2
int iwrite_value = 25;        // Nilai integer untuk penulisan
int iwrite_led = 255;         // Nilai integer untuk penulisan
int iwrite_min = 0;           // Nilai integer untuk penulisan
int heatSensorThreshold = 50; // Batas ambang sensor suhu