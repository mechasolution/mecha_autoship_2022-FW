#include <Adafruit_NeoPixel.h>
#include <Arduino.h>
#include <Arduino_LSM6DS3.h>
#include <FreeRTOS_SAMD21.h>
#include <Servo.h>
#include <Wire.h>
#include <queue.h>
#include <semphr.h>
#include <stdio.h>
#include <string.h>

#include "HMC5883L_Registers.h"

#define INCLUDE_uxTaskGetStackHighWaterMark 1 // 스택 밴치마크 활성화

#define STACK_SIZE (150*2) // Task 스택 크기
#define UART_BUFF_SIZE (10) // HW UART 버퍼 크기
#define COMMAND_QUEUE_SIZE (10) // 데이터 요청 리스트 큐 크기

#define RGBW_RING_SIZE 8 // 네오픽셀링 네오픽셀 개수

QueueHandle_t hwSerialCommandQueue;   // 시리얼 데이터 요청 리스트

Servo pwmServo;
Servo pwmBLDC;

Adafruit_NeoPixel pixelRing(RGBW_RING_SIZE, 12, NEO_GRBW + NEO_KHZ800);

struct SensorData_t {
  // IMU
  float IMU_ACC_X; // 가속도
  float IMU_ACC_Y;
  float IMU_ACC_Z;
  float IMU_GYR_X; // 자이로
  float IMU_GYR_Y;
  float IMU_GYR_Z;
  float IMU_MAG_X; // 지자기
  float IMU_MAG_Y;
  float IMU_MAG_Z;
  int16_t IMU_TEMP; // 내장 온도센서

  float IMU_GYR_OFFSET_X; // 자이로 센서 오프셋
  float IMU_GYR_OFFSET_Y;
  float IMU_GYR_OFFSET_Z;

  // GPS
  uint8_t GPS_LATITUDE[10];  // 위도
  uint8_t GPS_LONGITUDE[12]; // 경도
  uint8_t GPS_IS_VALID; // 유효 여부. 0=false, 1=true

  // 서보모터 각도
  int16_t SERVO_DEGREE = 90;

  // BLDC 강도
  int16_t BLDC_POWER = 0;
} SensorData;


void taskHwSerial(void *par);
void taskGetIMU(void *par);
void taskGetGPS(void *par);
void taskGetMAG(void *par);
void taskHwSerialSendData(void *par);

void setOffset();
void setRingColor(uint32_t color);

void initSerial() {
  Serial.begin(115200);
  Serial1.begin(9600);
}

void initIMU() {
  IMU.begin();
}

void initPWM() {
  pwmServo.attach(2);
  pwmBLDC.attach(16);

  pwmServo.write(90);
  pwmBLDC.write(90);
}

void initMAG() {
  Wire.beginTransmission(i2c_addr);
  Wire.write(mode);           //write to mode register
  Wire.write(continuous);           //set measurement mode
  Wire.endTransmission();
  delay(10);

  Wire.beginTransmission(i2c_addr);
  Wire.write(configA);        //write to config A register
  Wire.write(0x70);           //8 samples averaged, 15Hz output rate, normal measurement
  Wire.endTransmission();
  delay(10);
}

void initRing() {
  pixelRing.begin();
  pixelRing.show();
  pixelRing.setBrightness(255);
}


void setup() {
  initSerial();
  initIMU();
  initMAG();
  initPWM();
  initRing();

  setRingColor(pixelRing.Color(255, 0, 0, 0));
  setOffset();
  setRingColor(pixelRing.Color(0, 0, 0, 255));

  strcpy((char*)SensorData.GPS_LONGITUDE, "00000.0000");
  strcpy((char*)SensorData.GPS_LATITUDE, "0000.0000");

  hwSerialCommandQueue = xQueueCreate(COMMAND_QUEUE_SIZE, sizeof(uint8_t));

  xTaskCreate(taskHwSerial, "taskHwSerial", STACK_SIZE, NULL, 2, NULL); // 우선순위 숫자 비례

  xTaskCreate(taskGetIMU, "taskGetIMU", STACK_SIZE*2, NULL, 1, NULL);
  xTaskCreate(taskGetMAG, "taskGetMAG", STACK_SIZE*2, NULL, 0, NULL);
  xTaskCreate(taskGetGPS, "taskGetGPS", STACK_SIZE*3, NULL, 0, NULL);
  xTaskCreate(taskHwSerialSendData, "taskHwSerialSendData", STACK_SIZE, NULL, 2, NULL);
  vTaskStartScheduler();

  for(;;) {}
}

void loop() {}


// 기능
char *strsep(char **stringp, const char *delim) { // 문자열 파싱 함수
  char *ptr = *stringp;

  if (ptr == NULL) {
    return NULL;
  }

  while (**stringp) {
    if (strchr(delim, **stringp) != NULL) {
      **stringp = 0x00;
      (*stringp)++;
      return ptr;
    }
    (*stringp)++;
  }
  *stringp = NULL;

  return ptr;
}

void setRingColor(uint32_t color) {
  for (int i = 0; i < RGBW_RING_SIZE; i++) {
    pixelRing.setPixelColor(i, color);
  }
  pixelRing.show();
}

void setOffset() {
  // gyroscope
  bool update_gX = true;
  bool update_gY = true;
  bool update_gZ = true;

  int cnt = 500;
  while (1) { // offset adjusting loop
    if(IMU.gyroscopeAvailable()) {
      IMU.readGyroscope(SensorData.IMU_GYR_X, SensorData.IMU_GYR_Y, SensorData.IMU_GYR_Z);
      SensorData.IMU_GYR_X -= SensorData.IMU_GYR_OFFSET_X;
      SensorData.IMU_GYR_Y -= SensorData.IMU_GYR_OFFSET_Y;
      SensorData.IMU_GYR_Z -= SensorData.IMU_GYR_OFFSET_Z;
    }

    if(SensorData.IMU_GYR_X > 0) {
      SensorData.IMU_GYR_OFFSET_X+=0.01;
    } else if(SensorData.IMU_GYR_X < 0) {
      SensorData.IMU_GYR_OFFSET_X-=0.01;
    }
    if(SensorData.IMU_GYR_Y > 0) {
      SensorData.IMU_GYR_OFFSET_Y+=0.01;
    } else if(SensorData.IMU_GYR_Y < 0) {
      SensorData.IMU_GYR_OFFSET_Y-=0.01;
    }
    if(SensorData.IMU_GYR_Z > 0) {
      SensorData.IMU_GYR_OFFSET_Z+=0.01;
    } else if(SensorData.IMU_GYR_Z < 0) {
      SensorData.IMU_GYR_OFFSET_Z-=0.01;
    }

    float maximum_error = 0.01;
    if(abs(SensorData.IMU_GYR_X) <= maximum_error)
      update_gX = false;
    if(abs(SensorData.IMU_GYR_Y) <= maximum_error)
      update_gY = false;
    if(abs(SensorData.IMU_GYR_Z) <= maximum_error)
      update_gZ = false;

    if (update_gX == false && update_gY == false && update_gZ == false) {
      if(--cnt <= 0) {
        break;
      } else {
        // Serial.println(cnt);
      }
    }
    // print data
    // Serial.print("AX: ");
    // Serial.print(SensorData.IMU_ACC_X);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_ACC_OFFSET_X);
    // Serial.print(" ) ");
    // Serial.print(" AY: ");
    // Serial.print(SensorData.IMU_ACC_Y);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_ACC_OFFSET_Y);
    // Serial.print(" ) ");
    // Serial.print(" AZ: ");
    // Serial.print(SensorData.IMU_ACC_Z);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_ACC_OFFSET_Z);
    // Serial.print(" ) ");
    // Serial.print("    GX: ");
    // Serial.print(SensorData.IMU_GYR_X);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_GYR_OFFSET_X);
    // Serial.print(" ) ");
    // Serial.print(" GY: ");
    // Serial.print(SensorData.IMU_GYR_Y);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_GYR_OFFSET_Y);
    // Serial.print(" ) ");
    // Serial.print(" GZ: ");
    // Serial.print(SensorData.IMU_GYR_Z);
    // Serial.print(" (Offset: ");
    // Serial.print(SensorData.IMU_GYR_OFFSET_X);
    // Serial.println(" ) ");

    delay(10);
  }
}

void setPWM(int _bldc, int _servo) {
  pwmServo.write(_servo);
  pwmBLDC.write(_bldc);

  SensorData.SERVO_DEGREE = _servo;
  SensorData.BLDC_POWER = _bldc;
}

void queryOrder(uint8_t *_data, uint8_t _dataLen) {
  _data[_dataLen] = 0;
  if(_data[0] != '$') return;
  // $Q1\n
  switch(_data[1]) {
    case 'Q' :
      switch(_data[2]) {
        case 'B' : // 배터리 전압
          // TODO: 배터리 전압 읽어서 보내야함
          Serial.println("#QB\n");
          break;
        case 'I' : {// IMU 보정값 요청
          uint8_t _queue = 'I';
          xQueueSend(hwSerialCommandQueue, &_queue, portMAX_DELAY);
          }
          break;

        case 'O' : // IMU 오프셋 설정
          setOffset();
          break;

        case 'G' : // GPS Data req
          uint8_t _queue = 'G';
          xQueueSend(hwSerialCommandQueue, &_queue, portMAX_DELAY);
          break;
      }
      break;

    case 'C' :
      switch(_data[2]) {
        case 'D' : // 모터 설정 $CD,(BLDC),(서보)\n
          int _bldc, _servo;
          strtok((char*)_data, ","); // $CD
          _bldc = atoi(strtok(NULL, ",")); // (BLDC)
          _servo = atoi(strtok(NULL, "\n")); // (서보)

          setPWM(_bldc, _servo);
          break;

        case 'L' :
          int _r, _g, _b, _w = 0;
          strtok((char *)_data,  ",");
          _r = atoi(strtok(NULL, ","));
          _g = atoi(strtok(NULL, ","));
          _b = atoi(strtok(NULL, ","));

          if(_r == _g && _g == _b && _r == _b) {
            _w = _r;
            _r = _g = _b = 0;
          }
          setRingColor(pixelRing.Color(_r, _g, _b, _w));
          break;
      }
      break;
  }
}

void queryGPS(uint8_t *_data, uint8_t _dataLen) {
  // $GPGGA,092725.00,4717.11399,N,00833.91590,E,1,8,1.01,499.6,M,48.0,M,,0*5B
  if(_data[1] == 'G'
  && _data[2] == 'P'
  && _data[3] == 'G'
  && _data[4] == 'G'
  && _data[5] == 'A') {
    char *tmp = (char*)_data;
    char *ptr = NULL;
    uint8_t i = 1;
    while((ptr = strsep(&tmp, ",")) != NULL) {
      switch(i) {
        case 2: // 시각
          break;
        case 3: // 위도
          if(ptr[0] == 0)
            strcpy((char*)SensorData.GPS_LATITUDE, "0000.0000");
          else
            strcpy((char*)SensorData.GPS_LATITUDE, ptr);
          break;
        case 5: // 경도
          if(ptr[0] == 0)
            strcpy((char*)SensorData.GPS_LONGITUDE, "00000.0000");
          else
            strcpy((char*)SensorData.GPS_LONGITUDE, ptr);
          break;
        case 7: // 정확도
          if(ptr[0] == '0')
            SensorData.GPS_IS_VALID = 0;
          else
            SensorData.GPS_IS_VALID = 1;
          break;
      }
      i++;
    }
  }
  // GPGGA == GNGGA
  if(_data[1] == 'G'
  && _data[2] == 'N'
  && _data[3] == 'G'
  && _data[4] == 'G'
  && _data[5] == 'A') {
    char *tmp = (char*)_data;
    char *ptr = NULL;
    uint8_t i = 1;
    while((ptr = strsep(&tmp, ",")) != NULL) {
      switch(i) {
        case 2: // 시각
          break;
        case 3: // 위도
          if(ptr[0] == 0)
            strcpy((char*)SensorData.GPS_LATITUDE, "0000.0000");
          else
            strcpy((char*)SensorData.GPS_LATITUDE, ptr);
          break;
        case 5: // 경도
          if(ptr[0] == 0)
            strcpy((char*)SensorData.GPS_LONGITUDE, "00000.0000");
          else
            strcpy((char*)SensorData.GPS_LONGITUDE, ptr);
          break;
        case 7: // 정확도
          if(ptr[0] == '0')
            SensorData.GPS_IS_VALID = 0;
          else
            SensorData.GPS_IS_VALID = 1;
          break;
      }
      i++;
    }
  }
}

// Task
void taskHwSerial(void *par) {
  uint8_t arrBuffUART[20], len = 0;
  for(;;) {
    while(Serial.available()) {
      arrBuffUART[len++] = Serial.read();
      if(arrBuffUART[len - 1] == '\n') {
        queryOrder(arrBuffUART, len);
        len = 0;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskGetGPS(void *par) {
  uint8_t arrBuffUART[100];
  uint8_t len = 0;
  for(;;) {
    while(Serial1.available()) {
      arrBuffUART[len++] = Serial1.read();
      if(arrBuffUART[len - 1] == '\n') {
        arrBuffUART[len] = '\0';
        queryGPS(arrBuffUART, len);
        len = 0;
      }
    }
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

void taskGetMAG(void *par) {
  for(;;) {
    Wire.beginTransmission(i2c_addr);
    Wire.write(0x03);
    Wire.endTransmission();
    Wire.requestFrom(i2c_addr, 6);
    if (Wire.available() >= 6) {
      int16_t temp = Wire.read();
      int16_t x = temp << 8;
      temp = Wire.read();
      x = x | temp;
      temp = Wire.read();
      int16_t z = temp << 8;
      temp = Wire.read();
      z = z | temp;
      temp = Wire.read();
      int16_t y = temp << 8;
      temp = Wire.read();
      y = y | temp;

      SensorData.IMU_MAG_X = (float)x / 1090;
      SensorData.IMU_MAG_Y = (float)y / 1090;
      SensorData.IMU_MAG_Z = (float)z / 1090;
    }
    vTaskDelay(135 / portTICK_PERIOD_MS);
  }
}

void taskGetIMU(void *par) {
  for(;;) {
    if(IMU.accelerationAvailable() &&IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(SensorData.IMU_ACC_X, SensorData.IMU_ACC_Y, SensorData.IMU_ACC_Z);
      IMU.readGyroscope(SensorData.IMU_GYR_X, SensorData.IMU_GYR_Y, SensorData.IMU_GYR_Z);

      SensorData.IMU_GYR_X -= SensorData.IMU_GYR_OFFSET_X;
      SensorData.IMU_GYR_Y -= SensorData.IMU_GYR_OFFSET_Y;
      SensorData.IMU_GYR_Z -= SensorData.IMU_GYR_OFFSET_Z;
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
}

void taskHwSerialSendData(void *par) {
  uint8_t eventBuff;
  // uint16_t queue_1 = 1;
  for(;;) {
    if(uxQueueMessagesWaiting(hwSerialCommandQueue) != 0) {
      xQueueReceive(hwSerialCommandQueue, &eventBuff, portMAX_DELAY);
      if(eventBuff == 'G') {
        Serial.print("#RG,");
        Serial.print((char*)SensorData.GPS_LATITUDE);
        Serial.print(',');
        Serial.print((char*)SensorData.GPS_LONGITUDE);
        Serial.print(',');
        Serial.print((SensorData.GPS_IS_VALID == 0) ? 0 : 1);
        Serial.println();
        continue;
      }
    }
    Serial.print("#R0,");
    // 자이로
    Serial.print(SensorData.IMU_GYR_X);
    Serial.print(',');
    Serial.print(SensorData.IMU_GYR_Y);
    Serial.print(',');
    Serial.print(SensorData.IMU_GYR_Z);
    Serial.print(',');

    // 가속도
    Serial.print(SensorData.IMU_ACC_X);
    Serial.print(',');
    Serial.print(SensorData.IMU_ACC_Y);
    Serial.print(',');
    Serial.print(SensorData.IMU_ACC_Z);
    Serial.print(',');

    // 지자기
    Serial.print(SensorData.IMU_MAG_X);
    Serial.print(',');
    Serial.print(SensorData.IMU_MAG_Y);
    Serial.print(',');
    Serial.print(SensorData.IMU_MAG_Z);
    Serial.print(',');

    // 서보모터 각도
    Serial.print(SensorData.SERVO_DEGREE);
    Serial.print(',');

    // BLDC 각도
    Serial.print(SensorData.BLDC_POWER);
    Serial.println();

    vTaskDelay(20 / portTICK_PERIOD_MS);
    // vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}
