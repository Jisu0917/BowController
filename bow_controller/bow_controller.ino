#include <ArduinoBLE.h>

#include <PluggableUSBHID.h>
#include <USBHID_Types.h>
#include <USBHID.h>
#include <USBMouse.h>
#include <mbed.h>

#include <Arduino_LSM9DS1.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

USBMouse bleMouse;

// 블루투스 설정
BLEService pointerService("6B718D41-BE15-E156-BB8D-7F17F5BC90E6");//("19B10000-E8F2-537E-4F6C-D104768A1214");
BLEStringCharacteristic pointerChar("6B718D41-BE15-E156-BB8D-7F17F5BC90E6", BLEWrite|BLERead | BLENotify, 20 * sizeof(char));//("19B10001-E8F2-537E-4F6C-D104768A1214",BLEWrite|BLERead | BLENotify, 9);

//03000200-0400-0500-0006-000700080009 : 내 데스크탑 uuid
//6B718D41-BE15-E156-BB8D-7F17F5BC90E6 : 내 ble uuid

void setup() {

  Serial.begin(9600, SERIAL_8N1);

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");

    while (1);
  }

  BLE.setLocalName("BowController");
  BLE.setAdvertisedService(pointerService); 
  pointerService.addCharacteristic(pointerChar);
  BLE.addService(pointerService);

  BLE.advertise();
  
  Serial.println("Bluetooth device active, waiting for connections...");

  //-----------------------------------//

  if (!IMU.begin()) {
    while (1);
  }
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600, SERIAL_8N1); // 디버깅을 위한 시리얼 모니터 시작 
  Serial.println("setup success...");
}

void loop() {

  BLEDevice central = BLE.central();

  if (central) {
    Serial.print("Connected to central: ");
    Serial.println(central.address());
      
      while(central.connected()){
      /*쓰기*/
        move_and_click();
      /*읽기*/
        if(pointerChar.written()) {
            if (pointerChar.value()) {
                int val = atoi(pointerChar.value().c_str());
                Serial.println(val);
            }//value
        }//written
    }//connected
  }//central
}

char* makeString(int n1, int n2, int n3) {
    // 충분한 크기로 동적 메모리 할당
    char* str1 = (char*)malloc(20 * sizeof(char));

    // 정수를 문자열로 변환하여 연결
    sprintf(str1, "%03d,%03d,%d", n1, n2, n3);

    // 출력하여 확인
    printf("%s\n", str1);

    // 문자열 포인터를 반환
    return str1;
}


void move_and_click() {
  
  float ax, ay, az;
  float prevxValue = 0, prevyValue = 0, prevzValue = 0;
  float xAcc = 0, yAcc = 0, zAcc = 0;
  int maxPosAcc = 10, maxNegAcc = -10;
  int state = 1;
  while(1) {

    digitalWrite(LED_BUILTIN, HIGH);

  
    if (IMU.accelerationAvailable  ()) {// 가속도 센서
      IMU.readAcceleration(ax, ay, az);
  
      if(ax > 0.3){// y-Axis up
        if(prevxValue > 0.3){
          if(yAcc < maxPosAcc){
            yAcc += ax;//maybe use ax ay for percision (done but needs ironing out, also put xy movement back)
          }
        }
        else {
          prevxValue = ax;
          yAcc = 0.1;
        }
        //bleMouse.move(0,yAcc); // y-Axis down
        char* result = makeString(0, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }

      else if(ax < -0.3){// y-axis down
        if(prevxValue < -0.3){
          if(yAcc > maxNegAcc){
            yAcc -= -1*ax;
          }
        }
        else {
          prevxValue = ax;
          yAcc = -0.1;
        }
        //bleMouse.move(0,yAcc); // y-axis up
        char* result = makeString(0, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }
        
      else if(ay < -0.3){// x-axis right
        if(prevyValue < -0.3){
          if(xAcc > maxNegAcc){
          xAcc -= -1*ay;
          }
        }
        else {
          prevyValue = ay;
          xAcc = -0.1;
        }
        //bleMouse.move(xAcc,0); // x-axis right
        char* result = makeString(xAcc, 0, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }
      
      else if(ay > 0.3){// x-axis left
        if(prevyValue > 0.3){
          if(xAcc < maxPosAcc){
          xAcc += ay;
          }
        }
        else {
          prevyValue = ay;
          xAcc = 0.1;
        }
        //bleMouse.move(xAcc,0); // x-axis left
        char* result = makeString(xAcc, 0, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }
      
      if(ay > 0.3 and ax < -0.3){// x,y-axis left, up 
        if(prevyValue > 0.3 and prevxValue < -0.3){
          if(xAcc < maxPosAcc){ //if X is tilted
            xAcc += ay;
          }
          if(yAcc > maxNegAcc){ //if y is tilted
            yAcc -= -1*ax;
          }
        }
        else {
          if(xAcc > maxNegAcc){//no idea what that does
            prevyValue = ay;
            xAcc = 1;
          }
          if (yAcc > maxNegAcc){
            prevxValue = ax;
            yAcc = -0.1;
          }   
          else {
            prevxValue = ax;
            yAcc = -0.1;
            prevyValue = ay;
            xAcc = 0.1;
          }
        }
        //bleMouse.move(xAcc,yAcc); // x,y-axis left, up
        char* result = makeString(xAcc, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }  
      else if(ay < -0.3 and ax < -0.3){// x,y-axis right, up
        if(prevyValue < -0.3 and prevxValue < -0.3){
          if (xAcc > maxNegAcc){ //if x is tilted
            xAcc -= -1*ay;
          }
          if(yAcc > maxNegAcc){ //if y is tilted
            yAcc -= -1*ax;
          }  
        }
        else {
          if (xAcc > maxNegAcc){//no idea what that does
            prevyValue = ay;
            xAcc = -0.1;
          }
          if (yAcc > maxNegAcc){
            prevxValue = ax;
            yAcc = -0.1;
          }   
        }
        //bleMouse.move(xAcc,yAcc); // x,y-axis right, up
        char* result = makeString(xAcc, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }

      else if(ay > 0.3 and ax > 0.3){// x,y-axis left, down
        if(prevyValue > 0.3 and prevxValue > 0.3){
          if (xAcc < maxPosAcc){ //if x is tilted
            xAcc += ay;
          }
          if(yAcc < maxPosAcc){ //if y is tilted
            yAcc += ax;
          }  
        }
        else {
          if (xAcc < maxPosAcc){//no idea what that does
            prevxValue = ax;
            yAcc = 0.1;
          }
          if (yAcc < maxPosAcc){
            prevyValue = ay;
            xAcc = 0.1;
          }   
        }
        //bleMouse.move(xAcc,yAcc); // x,y-axis left, down
        char* result = makeString(xAcc, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }

      else if(ay < -0.3 and ax > 0.3){// x,y-axis right, down
        if(prevyValue < -0.3 and prevxValue > 0.3){
          if (xAcc > maxNegAcc){ //if x is tilted
            xAcc -= -1*ay;
          }
          if(yAcc < maxPosAcc){ //if y is tilted
            yAcc += ax;
          }  
        }
        else {
          if (xAcc > maxNegAcc){//no idea what that does
            prevyValue = ay;
            xAcc = -0.1;
          }
          if (yAcc < maxPosAcc){
            prevxValue = ax;
            yAcc = 0.1;
          }   
        }
        //bleMouse.move(xAcc,yAcc); // x,y-axis right, down
        char* result = makeString(xAcc, yAcc, 0);
        pointerChar.writeValue(result);
        Serial.println(result);
        free(result);
      }
      

      // //if the button is pressed, send a left mouse click
      // if (digitalRead(4) == LOW) {
      //   //bleMouse.click(1);
      //   pointerChar.writeValue((String)makeString(0, 0, 1));
      //   if (digitalRead(4)==HIGH)
      //   { bleMouse.release(1);}
      //   delay(100);
      // }
      
      // if (digitalRead(3) == LOW) {
      //   //bleMouse.click(4);
      //   pointerChar.writeValue((String)makeString(0, 0, 4));
      //   if (digitalRead(3)==HIGH)
      //   { bleMouse.release(4); }
      //   delay(100);
      // }
      //   //if the button is pressed, send a right mouse click
      // if (digitalRead(2) == LOW) {
      //   //bleMouse.click(2);
      //   pointerChar.writeValue((String)makeString(0, 0, 2));
      //   if (digitalRead(2)==HIGH)
      //   { bleMouse.release(2); }
      //   delay(100);
      // }
    
      digitalWrite(LED_BUILTIN, LOW);
    }//end of if 가속도 센서

  }//end of while loop
  
}//end of this function
