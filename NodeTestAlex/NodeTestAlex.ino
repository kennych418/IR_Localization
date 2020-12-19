#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define BLE_UUID_IDANDENABLE_SERVICE        "59920589-d782-492d-8beb-0d211d66312f"
#define BLE_UUID_DATA_SERVICE               "19B10000-E8F2-537E-4F6C-D104768A1214"
#define BLE_UUID_SELF_SERVICE               "19A75601-A3C1-937D-8B5E-A483929A1832"

#define BLE_UUID_ENABLE                     "2134"

#define BLE_UUID_IR0                        "271C" 
#define BLE_UUID_IR120                      "2F21" 
#define BLE_UUID_IR240                      "2A65" 
#define BLE_UUID_COMP                       "2245" 
#define BLE_UUID_TIME                       "2703" 

#define BLE_UUID_SELFIR0                        "351F" 
#define BLE_UUID_SELFIR120                      "3F76" 
#define BLE_UUID_SELFIR240                      "7A98" 
#define BLE_UUID_SELFCOMP                       "5678" 
#define BLE_UUID_SELFTIME                       "9023" 

#define FLOATING_PIN1 A7
#define FLOATING_PIN2 A6

#define IR_EMITTER 3
#define IR_REC_0 A4
#define IR_REC_120 A0
#define IR_REC_240 A3
#define IR_READ_AVE_N 40

float mx, my, mz;
float mx_off, my_off, mz_off;

unsigned long remote_ir[3] = {0, 0, 0};
int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
int max_ir[3] = {0, 0, 0}; //This stores the filtered version.
const int deg_ir[3] = {0, 120, 240};
const int pin_location[3] = {IR_REC_0, IR_REC_120, IR_REC_240};

unsigned long local_compass = 0;
unsigned long remote_compass = 0;

BLEService IDandEnableService( BLE_UUID_IDANDENABLE_SERVICE );
BLEService DataService( BLE_UUID_DATA_SERVICE );
BLEService SelfService( BLE_UUID_SELF_SERVICE );

BLEUnsignedLongCharacteristic enableCharacteristic(BLE_UUID_ENABLE, BLEWrite|BLERead);

BLEUnsignedLongCharacteristic timeCharacteristic( BLE_UUID_TIME, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic ir0Characteristic( BLE_UUID_IR0, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic ir120Characteristic( BLE_UUID_IR120, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic ir240Characteristic( BLE_UUID_IR240, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic compCharacteristic( BLE_UUID_COMP, BLEWrite|BLERead);

BLEUnsignedLongCharacteristic selftimeCharacteristic( BLE_UUID_SELFTIME, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic selfir0Characteristic( BLE_UUID_SELFIR0, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic selfir120Characteristic( BLE_UUID_SELFIR120, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic selfir240Characteristic( BLE_UUID_SELFIR240, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic selfcompCharacteristic( BLE_UUID_SELFCOMP, BLEWrite|BLERead);

void read_IR() {
  for (int i = 0; i < 3; ++i) {
    raw_ir[i] = 0;
  }
  for (int n = 0; n < IR_READ_AVE_N; ++n) {
    for (int i = 0; i < 3; ++i) {
      raw_ir[i] += analogRead(pin_location[i]);
    }
  }
  for (int i = 0; i < 3; ++i) {
    raw_ir[i] = raw_ir[i] / IR_READ_AVE_N;
  }
}

float getCompassHeading() {
  float y = (my - my_off);
  float x = (mx - mx_off);
  if (x > 0 && y > 0) return 180+180*atan(y/x)/3.1415926535;
  else if (x > 0 && y < 0) return 180+180*atan(y/x)/3.1415926535;
  else if (x < 0 && y > 0) return 360+180*atan(y/x)/3.1415926535;
  else if (x < 0 && y < 0) return 180*atan(y/x)/3.1415926535;
  else return 0;
}

bool calibrateOffestsIMU() {
//  double gx_total = 0;
//  double gy_total = 0;
//  double gz_total = 0;
  float mx_max = 0;
  float my_max = 0;
  float mz_max = 0;
  float mx_min = 0;
  float my_min = 0;
  float mz_min = 0;
//  Serial.println("Rotate device to calibate magnetometer offsets. Press d when the motion is complete.");
  while (Serial.read() != 'd')
  {
    Serial.println(".");
    IMU.readMagneticField(mx, my, mz);
    if (mx > mx_max) mx_max = mx;
    if (mx < mx_min) mx_min = mx;
    if (my > my_max) my_max = my;
    if (my < my_min) my_min = my;
    if (mz > mz_max) mz_max = mz;
    if (mz < mz_min) mz_min = mz;
  }
  mx_off = (mx_max+mx_min)/2;
  my_off = (my_max+my_min)/2;
  mz_off = (mz_max+mz_min)/2;
  Serial.println("Magnetometer calibrated.");
//  Serial.println("Place device flat on the table and do not move it. Press s to start calibration.");
//  while (Serial.read() != 's') {delay(1);}
//  for (int i = 0; i < 1000; ++i)
//  {
//    IMU.readGyroscope(gx, gy, gz);
//    gx_total += gx;
//    gy_total += gy;
//    gz_total += gz;
//    delay(1);
//  }
//  gx_off = gx_total/1000;
//  gy_off = gy_total/1000;
//  gz_off = gz_total/1000;
//  Serial.println("Gyroscope calibrated.");
  
  return true;
}

void setup() {
  pinMode(IR_EMITTER, OUTPUT);
  pinMode(IR_REC_0, INPUT);
  pinMode(IR_REC_120, INPUT);
  pinMode(IR_REC_240, INPUT);
  
  Serial.begin(9600);
  while (!Serial);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  
  calibrateOffestsIMU();

  pinMode(LED_BUILTIN, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
}

int count = 0;

void loop() {
  BLE.setDeviceName("IRNode");
  BLE.setLocalName("IRNode");
  BLE.scanForUuid( BLE_UUID_DATA_SERVICE );
  
  //Rand Delay and ID
  unsigned long start = millis();
  unsigned long rand1 = analogRead( FLOATING_PIN1 )/6;
  unsigned long rand2 = analogRead( FLOATING_PIN2 )/4;
  unsigned long randDelay = rand1*rand2+1;
  Serial.print("Random delay: "); Serial.println(randDelay);
  while((millis() - start) < randDelay);

  start = millis();




  /////////////////Central/////////////////
  int trials = 6000;
  unsigned long enable = 0;
  for(int i = 0; i < trials; i++){
    BLEDevice peripheral = BLE.available();
    if(peripheral){ //Central
      if (peripheral.localName() != "IRNode") {
        Serial.println("peripheral local name not IRNode");
        return;
      }
      BLE.stopScan();
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Entering Central Mode");
      while(1){  
        if(peripheral.connect()){
          while(!peripheral.discoverAttributes());
          Serial.println("Connected");
          BLECharacteristic ENABLE = peripheral.characteristic(BLE_UUID_ENABLE);
          BLECharacteristic IR_0 = peripheral.characteristic(BLE_UUID_IR0);
          BLECharacteristic IR_120 = peripheral.characteristic(BLE_UUID_IR120);
          BLECharacteristic IR_240 = peripheral.characteristic(BLE_UUID_IR240);
          BLECharacteristic TIME = peripheral.characteristic(BLE_UUID_TIME);
          BLECharacteristic COMP = peripheral.characteristic(BLE_UUID_COMP);
          int state = 0;
          char c;// = Serial.read();
          while(peripheral.connected()){
//            Serial.println(state);
            if (millis() != start) {
              if (state > 0) --state;
              start = millis();
            }
            if (state == 0) {
              c = Serial.read();
              if (c == 'd' || c == 't') {
                state = 100;
              }
            }
            else if (state == 1) {
              if (c == 'd') ++count;
              IR_0.readValue(remote_ir[0]); // <----------------------------Write values in here
              IR_120.readValue(remote_ir[1]);
              IR_240.readValue(remote_ir[2]);
              ENABLE.readValue(enable); // <----------------------Read values here
              COMP.readValue(remote_compass);
              Serial.print("c = ");
              Serial.println(count);
              Serial.print("en = ");
              Serial.println(enable);
              Serial.print("L comp = ");
              Serial.println(local_compass);
              Serial.print("R comp = ");
              Serial.println(remote_compass);
              Serial.print("L = ");
              Serial.print(raw_ir[0]);
              Serial.print(",");
              Serial.print(raw_ir[1]);
              Serial.print(",");
              Serial.println(raw_ir[2]);
              Serial.print("R = ");
              Serial.print(remote_ir[0]);
              Serial.print(",");
              Serial.print(remote_ir[1]);
              Serial.print(",");
              Serial.println(remote_ir[2]);
//              --state;
            }
            else if (state > 1 && state <= 50) {
              enable = 1;
              ENABLE.writeValue(enable);
//              Serial.print(state);
//              Serial.println(" lower");
//              --state;
            }
            else if (state > 50 && state < 100) {
              enable = 0;
              ENABLE.writeValue(enable);
//              Serial.print(state);
//              Serial.println(" upper");
              read_IR();
              for (int i = 0; i < 3; ++i) {
                if (raw_ir[i] > max_ir[i]) {
                    max_ir[i] = raw_ir[i];
                }
              }
//              --state;
            }
            if (state == 100) {
//              Serial.print(state);
//              Serial.println(" start");
              for (int i = 0; i < 3; ++i) {
                raw_ir[i] = 0;
                max_ir[i] = 0;
              }
//              --state;
            }
//            if(millis()%2000 > 1000){ 
//              enable = 0;
//              ENABLE.writeValue(enable); //<----------------------------Alternate your emitter + Read/Write Stuff
//            //      start = millis();
//            }
//            else{
//              enable = 1;
//              ENABLE.writeValue(enable);      //<----------------------------Alternate your emitter + Read/Write Stuff
//            }
            if (IMU.magneticFieldAvailable()) {
              IMU.readMagneticField(mx,my,mz);
              local_compass = getCompassHeading();
            }
//            unsigned long measurement = 0;
//            unsigned long timemeasurement = 0;
          }
        }
        else{
          Serial.println("Failed to connect!");
          continue;
        }
      }
    }
  }




  
  /////////////////Peripheral///////////////////
  BLE.stopScan();
  digitalWrite(LED_BUILTIN, HIGH);
  Serial.println("Entering Peripheral Mode");

  BLE.setAdvertisedService( DataService );
  BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );

  IDandEnableService.addCharacteristic( enableCharacteristic );

  DataService.addCharacteristic( timeCharacteristic );
  DataService.addCharacteristic( ir0Characteristic );
  DataService.addCharacteristic( ir120Characteristic );
  DataService.addCharacteristic( ir240Characteristic );
  DataService.addCharacteristic( compCharacteristic );

  SelfService.addCharacteristic( selftimeCharacteristic );
  SelfService.addCharacteristic( selfir0Characteristic );
  SelfService.addCharacteristic( selfir120Characteristic );
  SelfService.addCharacteristic( selfir240Characteristic );
  SelfService.addCharacteristic( selfcompCharacteristic );

  BLE.addService( IDandEnableService );
  BLE.addService( DataService );
  BLE.addService( SelfService );

  enableCharacteristic.writeValue( 0 );
  timeCharacteristic.writeValue( 0 );
  ir0Characteristic.writeValue( 0 );
  ir120Characteristic.writeValue( 0 );
  ir240Characteristic.writeValue( 0 );
  selfcompCharacteristic.writeValue( 0 );
  BLE.advertise();
  
  start = millis();    
  unsigned long prev_enable = 0;
  while(1){
    BLE.poll();
    prev_enable = enable;
    enableCharacteristic.readValue(enable);

    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx,my,mz);
      local_compass = getCompassHeading();
      compCharacteristic.writeValue(local_compass);
    }

    if (prev_enable == 0 && enable == 1) {
      for (int i = 0; i < 3; ++i) {
        raw_ir[i] = 0;
        max_ir[i] = 0;
      }
    }

    if (millis() != start) {
      start = millis();
      if (enable) {
        digitalWrite(IR_EMITTER, LOW);
        read_IR();
        for (int i = 0; i < 3; ++i) {
          if (raw_ir[i] > max_ir[i]) {
            max_ir[i] = raw_ir[i];
          }
        }
        ir0Characteristic.writeValue(max_ir[0]);
        ir120Characteristic.writeValue(max_ir[1]);
        ir240Characteristic.writeValue(max_ir[2]);
      }
      else {
        if (millis()%20 >= 10) {
          digitalWrite(IR_EMITTER, HIGH);
        }
        else {
          digitalWrite(IR_EMITTER, LOW);
        }
      }
    }

//    if(millis()%2000 > 1000){    //<----------------------------Set timer
//      enableCharacteristic.writeValue(0); //<----------------------------Alternate your emitter + Read/Write Stuff
////      start = millis();
//    }
//    else{
//      enableCharacteristic.writeValue(1);      //<----------------------------Alternate your emitter + Read/Write Stuff
//    }

    BLE.setDeviceName("IRNode");
    BLE.setLocalName("IRNode");
    BLE.setAdvertisedService( DataService );
    BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );
    BLE.advertise();

    Serial.print("Local compass heading = ");
    Serial.println(local_compass);
  }
}
