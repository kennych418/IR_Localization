#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h>

#define BLE_UUID_KEY        "59920589-d782-492d-8beb-0d211d66312f"

#define FLOATING_PIN1 A7
#define FLOATING_PIN2 A6

#define IR_EMITTER 3
#define IR_REC_0 A4
#define IR_REC_120 A0
#define IR_REC_240 A3
#define IR_READ_AVE_N 40

unsigned long NODE_NAME = 1;

///////// SET HOW LONG YOU WANT TO SCAN/BROADCAST DATA IN MILLISECONDS///////////////
unsigned long SCAN_TIME = 1000;//1300;    
unsigned long BROADCAST_TIME = 1000;//2000; 
////////////////////////////////////////////////////////////////////////////////////

float mx, my, mz;
float mx_off, my_off, mz_off;

int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
int max_ir[3] = {0, 0, 0}; //This stores the filtered version.
const int deg_ir[3] = {0, 120, 240};
const int pin_location[3] = {IR_REC_0, IR_REC_120, IR_REC_240};

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

int IRLED0 = 0;
int IRLED120 = 0;
int IRLED240 = 0;
int compass = 0;

BLEService KEY( BLE_UUID_KEY ); 
bool scanning = true;
unsigned long start = 0;

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

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  //Generate randomness for scan time and Node ID
  start = millis();
  unsigned long rand1 = analogRead( FLOATING_PIN1 )/6;
  unsigned long rand2 = analogRead( FLOATING_PIN2 )/4;
  unsigned long randDelay = rand1*rand2+1;
  Serial.print("Random delay: "); Serial.println(randDelay);
  SCAN_TIME = SCAN_TIME + randDelay;
  NODE_NAME = NODE_NAME + randDelay;
  while((millis() - start) < randDelay); 

  //Initialize temporary/initial values
  BLE.setLocalName("0");
  BLE.setAdvertisedService( KEY );
  BLE.setAdvertisedServiceUuid( BLE_UUID_KEY );
  BLE.addService( KEY );
}

void loop() {

  ////////INSERT YOUR CODE TO SAMPLE SENSORS HERE///////////
  if (IMU.magneticFieldAvailable()) {
    IMU.readMagneticField(mx,my,mz);
  }


  compass = (int)getCompassHeading();
  
  //////////////////////////////////////////////////////////

  //Switch from broadcasting to scanning
  if(((millis() - start) > SCAN_TIME) && !scanning){  
    Serial.println("Scanning");
    BLE.stopAdvertise();
    BLE.scanForUuid(BLE_UUID_KEY);
    scanning = true;
    start = millis();
    for (int i = 0; i < 3; ++i) {
      max_ir[i] = 0;
    }
  }

  //Switch from scanning to broadcasting
  if(((millis() - start) > BROADCAST_TIME) && scanning){ 
    digitalWrite(IR_EMITTER, LOW); 
    for (int k = 0; k < 20; ++k) {
      read_IR();
      for (int i = 0; i < 3; ++i) {
        if (raw_ir[i] > max_ir[i]) max_ir[i] = raw_ir[i];
      }
    }
    IRLED0 = max_ir[0];
    IRLED120 = max_ir[1];
    IRLED240 = max_ir[2];
    Serial.println("Broadcasting");
    BLE.stopScan();
    String nodename = String(NODE_NAME);  //Start with node name
    String embeddeddata = "," + String(IRLED0) + "," + String(IRLED120) + "," + String(IRLED240) + "," + String(compass); //Add data
    String localname = nodename + embeddeddata; //Concatenate node name with data, this is probably not necessary, will remove later
    char buf[99];
    localname.toCharArray(buf,localname.length()+1);  //Convert to C string for BLE to use, cant use arduino Strings
    BLE.setLocalName(buf);
    BLE.advertise();
    scanning = false;
    start = millis();
  }

  //If scanning, search for data and print it
  if(scanning){ 
    if (millis() % 50 < 40) {
      digitalWrite(IR_EMITTER, HIGH);
    }
    else {
      digitalWrite(IR_EMITTER, LOW);
    }
    BLEDevice peripheral = BLE.available();
    if(peripheral){
      Serial.println((millis() - start));
      String data = peripheral.localName();
      Serial.println(data);
    }
  }

}
