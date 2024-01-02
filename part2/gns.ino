#include <ArduinoBLE.h> // Arduino Standard BLE Library
#include <Arduino_LSM9DS1.h>

// Field structure of "Intermediate Hit Characteristic"
typedef struct IntermediateHit {
  byte flag = 0b00000000; // Refer to the "GATT Specification Supplement" for more detail
  byte cur[4];
  byte acc[4];
} IntermediateHit;
IntermediateHit ih;

BLEService HTService("1809"); // UUID of "Health Temperature Service" is 1809

BLECharacteristic intermediateHitChar("2A1E", BLENotify, sizeof(ih), true); // UUID of "Temperature Measurement Characteristic" is 2A1C.
byte cccd_value_2[2]= {0b00000000, 0b00000001};
BLEDescriptor cccd_2("2902", cccd_value_2, 2);

// Simple handler for the connect event
void blePeripheralConnectHandler(BLEDevice central) {
}

// Simple handler for the disconnect event
void blePeripheralDisconnectHandler(BLEDevice central) {
}

void setup() {
  pinMode(22, OUTPUT); // RED
  pinMode(23, OUTPUT); // GREEN
  pinMode(24, OUTPUT); // BLUE
  pinMode(25, OUTPUT);

  BLE.begin();
  IMU.begin();

  BLE.setEventHandler(BLEConnected, blePeripheralConnectHandler);
  BLE.setEventHandler(BLEDisconnected, blePeripheralDisconnectHandler);
  BLE.setLocalName("EE595");
  BLE.setDeviceName("MyDevice"); // UUID 2A00
  BLE.setAppearance(0x0543); // UUID 2A01
  intermediateHitChar.descriptor("2902") = cccd_2;
  HTService.addCharacteristic(intermediateHitChar); // Optional. Refer to the "Health Thermometer Service" document for more detail
  BLE.addService(HTService);
  BLE.setAdvertisedService(HTService);
  BLE.advertise();
}

void loop() {
  // This device will be a "peripheral"(or a GATT Server), and a "central"(or a GATT Client) will be connected
  BLEDevice central = BLE.central();
  float acc_thres = 5;
  float x, y, z, acc, max_acc = acc_thres;
  int state = 0;
  uint32_t current, old, debounce = 50000;
  // Central will be visible if connected
  if (central) {
    ih.flag = 0b00000000;
    while (central.connected()) {
      digitalWrite(23, LOW);
      digitalWrite(24, LOW);
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        acc = sqrt(x*x*6.25*6.25 + y*y*6.25*6.25 + z*z*6.25*6.25);
        current = micros();
        // Serial.println(acc); // for debug
        if (state == 0) {
          if (acc > acc_thres) {
            max_acc = acc;
            *(uint32_t*)&ih.cur = current;
            *(float*)&ih.acc = acc;
            state = 1;
          }
        }
        else if (state == 1) {
          if (acc > max_acc) {
            max_acc = acc;
            *(uint32_t*)&ih.cur = current;
            *(float*)&ih.acc = acc;
          }
          else {
            Serial.println(*(float*)&ih.acc);
            intermediateHitChar.writeValue((uint8_t*)&ih, sizeof(ih));
            old = current;
            max_acc = acc_thres;
            state = 2;
          }
        }
        else if (state == 2) {
          if (old + debounce > current)
            continue;
          else state = 0;
        }
      }
    }
    return;
  }
  else {
    digitalWrite(22, LOW);
    digitalWrite(23, HIGH);
    digitalWrite(24, HIGH);
  }
}
