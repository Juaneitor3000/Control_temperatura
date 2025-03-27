
#include <Arduino.h>                      //libreria general arduino
#include <SPIFFS.h>                       //particiones SPIFFS
#include <max6675.h>                      //Manejo de la termocupla
#include <U8g2lib.h>                      //Manejo de la pantalla OLED
#include <Wire.h>
#include <PID_v1.h>                       //Algoritmo de control PID
#include <ESPAsyncWebServer.h>            //servidor WEB
#include <ESPAsyncWiFiManager.h>          //Wifi mannager https://github.com/tzapu/WiFiManager WiFi Configuration Magic
#include <ArduinoJson.h>                  //Manejo de datos Json
#include <Preferences.h>                  //permite guardar el estado de variables luego de un reinicio
#include <ESP32Encoder.h>                 //Encoder


#define SSR_PIN 2 
#define PERIOD_MS 5000
#define BOTON_PIN 17
double pulseTime=10;
int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
int sampleTime= 5000000;
bool onState = true;
double error;


//Encoder

ESP32Encoder encoder;

//Objeto que salva las preferencias

Preferences preferences;

// Tiempo de referencia para el pulso de salida
unsigned long previousMillis = 0;
double pulseDuration = 0;
unsigned long currentMillis=0;

//WIFI MANNAGER
AsyncWebServer server(80);
AsyncWebSocket ws("/ws"); // WebSocket en la ruta /ws
DNSServer dns;


// Variables PID
double tempSetpointCelcius = 100.0;   // Temperatura deseada (°C)
double tempReadCelcius = 0, output = 0;
//double Kp = 0.4 , Ki = 0.01, Kd = 2000.0;   funciona sin demasiado overshott pero muy lento
double Kp = 3 , Ki = 0, Kd = 0;   //100 grados
PID myPID(&tempReadCelcius, &output, &tempSetpointCelcius, Kp, Ki, Kd,DIRECT);


// Configurar e0-l temporizador del controlador
hw_timer_t *timer = NULL;
volatile bool pidFlag = false;

/* Constructor Termocupla*/
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);

/* Constructor  Pantalla */
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,/* SCL=*/22,/* SDA=*/21);


// put function declarations here:
void printScreen(double ,double);                       // PRINT SCREEN
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
void sendJson();
void sendJsonTemp();
void parseJsonString(const String &jsonString);
void restartPID();
void errorPID();


// Interrupción del Timer (Cada 500ms)
void IRAM_ATTR onTimer() {
  pidFlag = true;
}



void setup() {
// put your setup code here, to run once:
//****************************************************************** 
//SERIAL PORT
Serial.begin(115200);  // Serial port init
Serial.println("SERIAL PORT TEST:Temperature controller");
//****************************************************************** 
//RESTORE PREFERENCES
preferences.begin("config", false);
tempSetpointCelcius = preferences.getDouble("tempSetpoint", 50);
preferences.end();
//****************************************************************** 
//SCREEN
u8g2.begin();        //Screen Init
Serial.println("SCREEN STARTED");
//******************************************************************
// WEBSERVER and WIFI MANAGER
AsyncWiFiManager wifiManager(&server,&dns);
//first parameter is name of access point, second is the password
wifiManager.autoConnect("AutoConnectAP");
//****************************************************************** 
//FILESYSTEM
if (!SPIFFS.begin(true)) {
  Serial.println("Error al montar SPIFFS. ¿Subiste los archivos con uploadfs?");
  return;
}
Serial.println("SPIFFS MOUNTED.");
//*****************************************************************
// Servir archivos estáticos desde SPIFFS
server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/index.html", "text/html");
});
server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/style.css", "text/css");
});
server.on("/script.js", HTTP_GET, [](AsyncWebServerRequest *request){
  request->send(SPIFFS, "/script.js", "application/javascript");
});
//****************************************************************** 
// WEBSOCKET
ws.onEvent(onWebSocketEvent);
server.addHandler(&ws);
server.begin();

//****************************************************************** 
// SSR OUTPUT
pinMode(SSR_PIN, OUTPUT); //output pin
//******************************************************************
// ENCODER BUTTON
pinMode(BOTON_PIN, PULLUP);
//******************************************************************
// PID
myPID.SetMode(AUTOMATIC);
myPID.SetOutputLimits(0,1000); // Limita la salida entre 0 y 1000
//****************************************************************** 
// PID
Serial.println("Temperature controller");
// wait for MAX chip to stabilize
delay(500);
// Configurar temporizador (sampletime)
timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 → 1MHz
timerAttachInterrupt(timer, &onTimer, true);
timerAlarmWrite(timer, sampleTime, true); // 1000000us = 1000ms
timerAlarmEnable(timer);
//Encoder
encoder.attachHalfQuad(16, 4);
encoder.setCount(tempSetpointCelcius);


}

void loop() {
if (pidFlag) {
    pidFlag = false;  // Reset flag

    if (tempSetpointCelcius<75){
      Kp=1.35;//2.7;
      Ki=0.7;
    }
    if (tempSetpointCelcius>=75){
      Kp=1.5;//3;
      Ki=1;
    }
     // Leer temperatura y calcular PID
    tempReadCelcius =  thermocouple.readCelsius();
    errorPID();
    if ((abs(error))<=15)
      {
        myPID.SetTunings(Kp, Ki, Kd);           //se habilita el término integral
        Serial.println("Aplicando Ki" );
      }
    else
      {
        myPID.SetTunings(Kp, 0, Kd);            // muy alejado del setpoin, deshabilitamos el término integral
        Serial.println("Ki deshabilitado" );
      }
    
  myPID.Compute();

  pulseDuration = pulseTime * output;
  previousMillis=millis();  // Reiniciar ciclo
  if (output!=0){
  digitalWrite(SSR_PIN, HIGH);  // Encender SSR
  }
  printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN  
  sendJsonTemp();
  //Serial.println("Lectura C = "+ String(tempReadCelcius));
  Serial.println("Ki:"+ String(Ki));
  Serial.println("Setpoint = "+ String(tempSetpointCelcius));
  Serial.println("Kp = "+ String(Kp));
  Serial.println("Output = "+ String(output));
  Serial.println("PulseTime = "+ String(pulseTime));
  Serial.println("Error: "+String(error));
  Serial.println("PulseDuration = "+ String(pulseDuration));
  Serial.println("");
  //Serial.println("currentMillis - previousMillis = "+ String((currentMillis - previousMillis)));      
}

// Calcular duración del pulso (entre 0 y PERIOD_MS)

 // Generar el pulso con millis()
//if (currentMillis - previousMillis >= PERIOD_MS) {
  //pulseDuration = pulseTime * output;
  //previousMillis=millis();  // Reiniciar ciclo
  //if (output!=0){
  //digitalWrite(SSR_PIN, HIGH);  // Encender SSR
  //printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN
  
//  }
  //Serial.println("PulseDuration = "+ String(pulseDuration));
  

currentMillis=millis();
if ((currentMillis - previousMillis) >= pulseDuration) {
  digitalWrite(SSR_PIN, LOW);  // Apagar SSR
  printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN
}


// ENCODER

if (encoder.getCount()<0){
  encoder.setCount(0);
}

if (encoder.getCount()>300){
  encoder.setCount(300);
}

if(encoder.getCount()!=tempSetpointCelcius){
  tempSetpointCelcius=encoder.getCount();
  printScreen(tempReadCelcius,tempSetpointCelcius);
  sendJsonTemp(); 
}

}

// put function definitions here:

void printScreen(double tempRead,double tempSetpoint){

  static const unsigned char image_output_state[] = {0x38,0x00,0x44,0x40,0xd4,0xa0,0x54,0x40,0xd4,0x1c,0x54,0x06,0xd4,0x02,0x54,0x02,0x54,0x06,0x92,0x1c,0x39,0x01,0x75,0x01,0x7d,0x01,0x39,0x01,0x82,0x00,0x7c,0x00};

  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  u8g2.setFont(u8g2_font_6x12_tr);
  u8g2.drawStr(1, 8, "objetivo:");
  u8g2.drawStr(2, 41, "Temperatura:");
  u8g2.setFont(u8g2_font_profont29_tr);
  char buffer[10];
  sprintf(buffer,"%.2f C",tempRead);  // Formateamos la cadena, mostrando 1 decimal
  u8g2.drawStr(0, 63, buffer);
  sprintf(buffer,"%.0f C",tempSetpoint);  // Formateamos la cadena, mostrando 1 decimal
  u8g2.drawStr(0, 30, buffer);


  //dibuja el simbolo de "calentando" , estado de la salida
  if (digitalRead(SSR_PIN)){
  static const unsigned char image_output_state[] = {0x38,0x00,0x44,0x40,0xd4,0xa0,0x54,0x40,0xd4,0x1c,0x54,0x06,0xd4,0x02,0x54,0x02,0x54,0x06,0x92,0x1c,0x39,0x01,0x75,0x01,0x7d,0x01,0x39,0x01,0x82,0x00,0x7c,0x00};
  u8g2.drawXBM(112, 0, 16, 16, image_output_state);
  }
  //dibuja el simbolo de "encendido" , si el PID esta operatico
  if (onState){
  static const unsigned char image_device_power_button_bits[] = {0x80,0x00,0x80,0x00,0x98,0x0c,0xa4,0x12,0x92,0x24,0x8a,0x28,0x85,0x50,0x05,0x50,0x05,0x50,0x05,0x50,0x05,0x50,0x0a,0x28,0x12,0x24,0xe4,0x13,0x18,0x0c,0xe0,0x03};
  u8g2.drawXBM(92, 0, 15, 16, image_device_power_button_bits);
  } 

  u8g2.sendBuffer();

}


void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, 
  AwsEventType type, void *arg, uint8_t *data, size_t len) {
if (type == WS_EVT_DATA) {
// Convertir datos a cadena JSON
String message = String((char *)data).substring(0, len);
Serial.println("JSON recibido: " + message);

parseJsonString(message);
sendJson();
}
}


void sendJson() {
  JsonDocument doc;
  doc["temp"] = tempReadCelcius; // lectura de temperatura
  doc["setpoint"] = tempSetpointCelcius; // lectura de temperatura
  doc["onState"]=onState;
  
  String jsonStr;
  serializeJson(doc, jsonStr);
  ws.textAll(jsonStr); // Enviar a todos los clientes conectados
  printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN 
}


void sendJsonTemp() {
  JsonDocument doc;
  doc["temp"] = tempReadCelcius; // lectura de temperatura
  doc["setpoint"]=tempSetpointCelcius;  //lectura de setpoint
  String jsonStr;
  serializeJson(doc, jsonStr);
  ws.textAll(jsonStr); // Enviar a todos los clientes conectados
  printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN 
}


void parseJsonString(const String &jsonString) {
  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, jsonString);

  if (error) {
      Serial.print("Error al parsear JSON: ");
      Serial.println(error.f_str());
      return;
  }

  if (doc.containsKey("temp")) {
    tempReadCelcius = doc["temp"];
  }

  if (doc.containsKey("setpoint")) {
    restartPID();
    tempSetpointCelcius = doc["setpoint"];
    //****************************** */
    //Guardar SETPOINT
    preferences.begin("config", false);
    preferences.putDouble("tempSetpointCelcius",tempSetpointCelcius);
    preferences.end();
    //******************************* */
  }

  if (doc.containsKey("onState")) {
    onState = doc["onState"];
  }
}


void apagarPID() {
  onState = true;
  myPID.SetMode(MANUAL);
  //Output = 0;
}

void encenderPID() {
  onState =false;
  myPID.SetMode(AUTOMATIC);
}
void restartPID(){
  myPID.SetMode(MANUAL);  // Desactiva el PID temporalmente
  output=0;     // Opcional: Reinicia la salida
  myPID.SetMode(AUTOMATIC);  // Reactiva el PID con el nuevo setpoint
}

void errorPID(){
  error=tempSetpointCelcius-tempReadCelcius;
} 
