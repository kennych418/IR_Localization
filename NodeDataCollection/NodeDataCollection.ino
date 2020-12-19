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

int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
int max_ir[3] = {0, 0, 0}; //This stores the filtered version.
const int deg_ir[3] = {0, 120, 240};
const int pin_location[3] = {IR_REC_0, IR_REC_120, IR_REC_240};

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




  /////////////////Central/////////////////
  int trials = 6000;
  unsigned long enable = 0;
  unsigned long compass_remote = 0;
  for(int i = 0; i < trials; i++){
    BLEDevice peripheral = BLE.available();
    if(peripheral){ //Central
      if (peripheral.localName() != "IRNode") {
        return;
      }
      BLE.stopScan();
      digitalWrite(LED_BUILTIN, LOW);
      Serial.println("Entering Central Mode");
      while(1){  
        if (IMU.magneticFieldAvailable()) {
          IMU.readMagneticField(mx,my,mz);
        }
        if(peripheral.connect() || peripheral.connected()){
          while(!peripheral.discoverAttributes());
          Serial.println("Connected");
          Serial.println(peripheral.localName());
          BLECharacteristic ENABLE = peripheral.characteristic(BLE_UUID_ENABLE);
          BLECharacteristic IR_0 = peripheral.characteristic(BLE_UUID_IR0);
          BLECharacteristic IR_120 = peripheral.characteristic(BLE_UUID_IR120);
          BLECharacteristic IR_240 = peripheral.characteristic(BLE_UUID_IR240);
          BLECharacteristic TIME = peripheral.characteristic(BLE_UUID_TIME);
          BLECharacteristic COMP = peripheral.characteristic(BLE_UUID_SELFCOMP);
          start = millis();
          int measurement_iterator = 0;
          bool data_good = false;
          bool take_reading = false;
          while(peripheral.connected()){
            if (take_reading){
              take_reading = false;
              //Do something here to change enable at the correct times.
              measurement_iterator = 50;
              for (int i = 0; i < 3; ++i) {
                raw_ir[i] = 0;
                max_ir[i] = 0;
              }
            }
            
            if (measurement_iterator = 1) {
              data_good = true;
              --measurement_iterator;
            }
            if (measurement_iterator > 1) {
              if (measurement_iterator > 25) enable = 1;
              else enable = 0;
              if (millis() != start) {
                if (!enable) {
                  if (millis()%20 >= 10) {
                    digitalWrite(IR_EMITTER, HIGH);
                  }
                  else {
                    digitalWrite(IR_EMITTER, LOW);
                  }
                }
                else {
                  digitalWrite(IR_EMITTER, LOW);
                  read_IR();
                  for (int i = 0; i < 3; ++i) {
                    if (raw_ir[i] > max_ir[i]) {
                      max_ir[i] = raw_ir[i];
                    }
                  }
                }
                start = millis();
                --measurement_iterator;
              }
            }
            if (IMU.magneticFieldAvailable()) {
              IMU.readMagneticField(mx,my,mz);
            }
            unsigned long measurement = 0;
            unsigned long timemeasurement = 0;
            //IR.writeValue(measurement); // <----------------------------Write values in here
            TIME.writeValue(timemeasurement); // <----------------------------Write values in here
            ENABLE.writeValue(enable); // <----------------------Read values here
            COMP.readValue(compass_remote);
//            IR0.readValue();
//            IR120.writeValue();
//            IR240.writeValue();
//            if (Serial.read() == 'e') {
//              ENABLE.writeValue(!enable);
//              Serial.print("Enable value: ");
//              Serial.println(enable);
//            }
            if (Serial.read() == 'd') {
//              ENABLE.writeValue(0);
              take_reading = true;
            }
            
            Serial.println();
            Serial.println(measurement_iterator);
            Serial.println(data_good);
            Serial.println(enable);
            if (data_good) {
//              Serial.println(enable);
              Serial.print(max_ir[0]);
              Serial.print(",");
              Serial.print(max_ir[1]);
              Serial.print(",");
              Serial.print(max_ir[2]);
              Serial.print(",");
              unsigned long val = 0;
              IR_0.readValue(val);
              Serial.print(val);
              Serial.print(",");
              IR_120.readValue(val);
              Serial.print(val);
              Serial.print(",");
              IR_240.readValue(val);
              Serial.print(val);
              Serial.print(",");
              Serial.print(compass_remote);
              Serial.print(",");
              Serial.print((unsigned long)getCompassHeading());
              Serial.print(",");
              long comp_diff_1 = (360 + compass_remote-(unsigned long)getCompassHeading())%360; //compass_remote-(unsigned long)getCompassHeading();
//              long comp_diff_2 = (360 + compass_remote-(unsigned long)getCompassHeading())%360;
              Serial.print(comp_diff_1);
              Serial.print(",");
//              if (abs(comp_diff_1) < abs(comp_diff_2)) {
//                Serial.print(comp_diff_1);
//                Serial.print(",");
//              }
//              else {
//                Serial.print(comp_diff_2);
//                Serial.print(",");
//              }
              Serial.println();
              data_good = false;
            }
          }
        }
        else{
          Serial.println("Failed to connect!");
          Serial.println(peripheral.localName());
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

  enableCharacteristic.readValue(enable);
  timeCharacteristic.writeValue( 0 );
  ir0Characteristic.writeValue( 0 );
  selfcompCharacteristic.writeValue(getCompassHeading());
  BLE.advertise();

  start = millis();    
  int prev_enable = 0;
  while(1){
    BLE.poll();
    enableCharacteristic.readValue(enable);
    if (prev_enable != enable) {
      for (int i = 0; i < 3; ++i) {
        raw_ir[i] = 0;
        max_ir[i] = 0;
      }
      prev_enable = enable;
    }
    if (millis() != start) {
      if (enable) {
        if (millis()%20 >= 10) {
          digitalWrite(IR_EMITTER, HIGH);
        }
        else {
          digitalWrite(IR_EMITTER, LOW);
        }
      }
      else {
        digitalWrite(IR_EMITTER, LOW);
        read_IR();
        for (int i = 0; i < 3; ++i) {
          if (raw_ir[i] > max_ir[i]) {
            max_ir[i] = raw_ir[i];
          }
        }
        selfir0Characteristic.writeValue(max_ir[0]);
        selfir120Characteristic.writeValue(max_ir[1]);
        selfir240Characteristic.writeValue(max_ir[2]);
      }
      start = millis();
    }
    

    if (IMU.magneticFieldAvailable()) {
      IMU.readMagneticField(mx,my,mz);
      selfcompCharacteristic.writeValue((unsigned long)getCompassHeading());
    }

//    if((millis()-start) > 1000){    //<----------------------------Set timer
//      enableCharacteristic.writeValue(0); //<----------------------------Alternate your emitter + Read/Write Stuff
//      start = millis();
//    }
//    else{
//      enableCharacteristic.writeValue(1);      //<----------------------------Alternate your emitter + Read/Write Stuff
//    }

    BLE.setDeviceName("IRNode");
    BLE.setLocalName("IRNode");
    BLE.setAdvertisedService( DataService );
    BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );
    BLE.advertise();
  }
}
