#include <PluggableUSBHID.h>
#include <USBHID_Types.h>
#include <USBHID.h>
//#include <USBMouse.h>
#include <mbed.h>

#include <Arduino_LSM9DS1.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//USBMouse bleMouse;

#include <ArduinoBLE.h>

 // Bluetooth® Low Energy Pointer Service
BLEService pointerService("0000AAAA-0000-1000-8000-00805F9B34FB");

// Bluetooth® Low Energy Pointer Characteristic
BLEFloatCharacteristic XpointerChar("0000AAAA-0001-1000-8000-00805F9B34FB",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

// Bluetooth® Low Energy Battery Level Characteristic
BLEFloatCharacteristic YpointerChar("0000AAAA-0002-1000-8000-00805F9B34FB",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes

BLEIntCharacteristic onClickChar("0000AAAA-0003-1000-8000-00805F9B34FB",  // standard 16-bit characteristic UUID
    BLERead | BLENotify); // remote clients will be able to get notifications if this characteristic changes


float prevXpoint = 0.0;  // last x point reading from analog input
long previousMillis = 0;  // last time the battery level was checked, in ms

void setup() {
  //Serial.begin(9600);    // initialize serial communication
  //while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT); // initialize the built-in LED pin to indicate when a central is connected

  // begin initialization
  if (!BLE.begin()) {
    //Serial.println("starting BLE failed!");

    while (1);
  }

  /* Set a local name for the Bluetooth® Low Energy device
     This name will appear in advertising packets
     and can be used by remote devices to identify this Bluetooth® Low Energy device
     The name can be changed but maybe be truncated based on space left in advertisement packet
  */
  BLE.setLocalName("PointerControl");
  BLE.setAdvertisedService(pointerService); // add the service UUID
  pointerService.addCharacteristic(XpointerChar); // add the battery level characteristic
  pointerService.addCharacteristic(YpointerChar);
  pointerService.addCharacteristic(onClickChar);
  BLE.addService(pointerService); // Add the battery service
  XpointerChar.writeValue(prevXpoint); // set initial value for this characteristic

  /* Start advertising Bluetooth® Low Energy.  It will start continuously transmitting Bluetooth® Low Energy
     advertising packets and will be visible to remote Bluetooth® Low Energy central devices
     until it receives a new connection */

  // start advertising
  BLE.advertise();

  //Serial.println("Bluetooth® device active, waiting for connections...");
}

void loop() {
  // wait for a Bluetooth® Low Energy central
  BLEDevice central = BLE.central();

  // if a central is connected to the peripheral:
  if (central) {
    //Serial.print("Connected to central: ");
    // print the central's BT address:
    //Serial.println(central.address());
    // turn on the LED to indicate the connection:
    digitalWrite(LED_BUILTIN, HIGH);

    // check the battery level every 200ms
    // while the central is connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the battery level:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updatePointer();
      }
    }
    // when the central disconnects, turn off the LED:
    digitalWrite(LED_BUILTIN, LOW);
    //Serial.print("Disconnected from central: ");
    //Serial.println(central.address());
  }
}

void updatePointer() {
  /* Read the current voltage level on the A0 analog input pin.
     This is used here to simulate the charge level of a battery.
  */
  float ax, ay, az;
  float prevxValue = 0, prevyValue = 0, prevzValue = 0;
  float xAcc = 0, yAcc = 0, zAcc = 0;
  int maxPosAcc = 10, maxNegAcc = -10;
  int state = 1;
  while(1) {
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
        YpointerChar.writeValue(yAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
        YpointerChar.writeValue(yAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
        XpointerChar.writeValue(xAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
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
        XpointerChar.writeValue(xAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
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
        XpointerChar.writeValue(xAcc);
        YpointerChar.writeValue(yAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
        XpointerChar.writeValue(xAcc);
        YpointerChar.writeValue(yAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
        XpointerChar.writeValue(xAcc);
        YpointerChar.writeValue(yAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
        XpointerChar.writeValue(xAcc);
        YpointerChar.writeValue(yAcc);
        //Serial.print("XpointerChar update: ");
        //Serial.println(xAcc);
        //Serial.print("YpointerChar update: ");
        //Serial.println(yAcc);
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
    }//end of if 가속도 센서

  }//end of while loop
}
