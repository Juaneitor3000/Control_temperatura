#include <Arduino.h>
#include <max6675.h>
#include <U8g2lib.h>
#include <Wire.h>



int thermoDO = 19;
int thermoCS = 23;
int thermoCLK = 5;
int sampleTime= 1000000;



// Configurar el temporizador del controlador
hw_timer_t *timer = NULL;
volatile bool pidFlag = false;
/* Constructor */
MAX6675 thermocouple(thermoCLK, thermoCS, thermoDO);
/* Constructor */
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0,U8X8_PIN_NONE,/* SCL=*/22,/* SDA=*/21);


// put function declarations here:
void printScreen(float ,float);                       // PRINT SCREEN



// Interrupción del Timer (Cada 500ms)
void IRAM_ATTR onTimer() {
  pidFlag = true;
}



void setup() {
// put your setup code here, to run once:
  
Serial.begin(9600);  // Serial port init
u8g2.begin();        //Screen Init


Serial.println("Temperature controller");
// wait for MAX chip to stabilize
delay(500);


// Configurar temporizador (sampletime)
   timer = timerBegin(0, 80, true);  // Timer 0, prescaler 80 → 1MHz
   timerAttachInterrupt(timer, &onTimer, true);
   timerAlarmWrite(timer, sampleTime, true); // 1000000us = 1000ms
   timerAlarmEnable(timer);


}

void loop() {



if (pidFlag) {
    pidFlag = false;  // Reset flag
    float tempSetpointCelcius=60;
    float tempReadFarenheit = thermocouple.readFahrenheit();
    float tempReadCelcius =  thermocouple.readCelsius();
    Serial.print("C = "+ String(tempReadCelcius) +" F= "+ String(tempReadFarenheit));
    printScreen(tempReadCelcius,tempSetpointCelcius);     //   PRINT SCREEN  
}
  // put your main code here, to run repeatedly:

}

// put function definitions here:

void printScreen(float tempRead,float tempSetpoint){

  static const unsigned char image_download_bits[] = {0x38,0x00,0x44,0x40,0xd4,0xa0,0x54,0x40,0xd4,0x1c,0x54,0x06,0xd4,0x02,0x54,0x02,0x54,0x06,0x92,0x1c,0x39,0x01,0x75,0x01,0x7d,0x01,0x39,0x01,0x82,0x00,0x7c,0x00};

  u8g2.clearBuffer();
  u8g2.setFontMode(1);
  u8g2.setBitmapMode(1);
  u8g2.setFont(u8g2_font_4x6_tr);
  u8g2.drawStr(0, 5, "Setpoint:");
  u8g2.drawStr(0, 48, "Temp:");
  u8g2.setFont(u8g2_font_t0_22b_tr);
  char buffer[10];
  sprintf(buffer,"%.1f C",tempRead);  // Formateamos la cadena, mostrando 1 decimal
  u8g2.drawStr(0, 64, buffer);
  u8g2.drawXBM(112, 0, 16, 16, image_download_bits);
  sprintf(buffer,"%.1f C",tempSetpoint);  // Formateamos la cadena, mostrando 1 decimal
  u8g2.drawStr(0, 21, buffer);
  u8g2.sendBuffer();
}