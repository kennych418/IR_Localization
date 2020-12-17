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

#define MAX_NUM_NODES 10
#define NUM_VALUES_PER_NODE 6
#define NUM_STATES 6

uint16_t valueArray[MAX_NUM_NODES][NUM_VALUES_PER_NODE];
uint16_t myID = 0;
uint16_t myIndex = 0;
uint16_t currentIndex = 0;
uint16_t numNodes = 0;

float mx, my, mz;
float mx_off, my_off, mz_off;

int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
int max_ir[3] = {0, 0, 0}; //This stores the filtered version.
const int deg_ir[3] = {0, 120, 240};
const int pin_location[3] = {IR_REC_0, IR_REC_120, IR_REC_240};

BLEService KEY( BLE_UUID_KEY ); 
bool scanning = true;
unsigned long start = 0;

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

void bubbleSortIDs() {
  //Sort the entries in column 1 of the data array for rows 0 to numNodes-1
  //Bubble sort is ok here due to small number of entries.
  int temp = 0;
  for (int i = 1; i < numNodes; ++i) {
    for (int k = 0; k < numNodes - i; ++k) {
      if (valueArray[k][0] > valueArray[k+1][0]) {
        //Swap values
        temp = valueArray[k][0];
        valueArray[k][0] = valueArray[k+1][0];
        valueArray[k+1][0] = temp;
      }
    }
  }
  return;
}

bool discoverNodes() {
  //Add self to table
  valueArray[numNodes][0] = myID;
  numNodes = numNodes + 1;
  Serial.println(myID);
  while (Serial.read() != 'd') {
    int random_int = random(1,100);
    //Pseudocode in loop:
    //Scan for state information
//    Serial.print("Scanning ");
    BLE.stopAdvertise();
    BLE.scanForUuid(BLE_UUID_KEY);    //TODO: We need to have multiple BLE_UUID_KEY
    start = millis();
    while (millis() - start < 250+random_int) {
      BLEDevice peripheral = BLE.available();
      if(peripheral){
        String data = peripheral.localName();
//        Serial.println(data);
        int ID = data.toInt();
        bool newFlag = true;
        for (int i = 0; i < numNodes; ++i) {
          if (valueArray[i][0] == ID) newFlag = false;
        }
        if (newFlag) {
          if (numNodes+1 > MAX_NUM_NODES) return false;
          valueArray[numNodes][0] = ID;
          numNodes = numNodes + 1;
          Serial.println(data);
        }
  //      delay(random(1,100)); //Preventing accidental synchronization.
  //      break;
      }
    }
//    Serial.print("Broadcasting ");
    BLE.stopScan();
    String nodename = String(myID);
  //  nodename.concat((char)(myID>>8));
  //  nodename.concat((char)(myID&0x0f));  //Start with node name
    char buf[99];
    nodename.toCharArray(buf,nodename.length()+1);  //Convert to C string for BLE to use, can't use arduino Strings
    BLE.setLocalName(buf);
    start = millis();
    BLE.advertise();
//    Serial.println(nodename);
    delay(150-random_int);
//    Serial.println();
  }
  BLE.stopAdvertise();
  Serial.println("Before sort");
  for (int i = 0; i < numNodes; ++i) {
    Serial.print(valueArray[i][0]);
    Serial.print(",");
  }
  Serial.println();
  bubbleSortIDs();
  Serial.println("After sort");
  for (int i = 0; i < numNodes; ++i) {
    Serial.print(valueArray[i][0]);
    Serial.print(",");
  }
  Serial.println();
  //Find myIndex after sorting
  for (int i = 0; i < numNodes; ++i) {
    if (valueArray[i][0] == myID) {
      myIndex = i;
    }
  }
  //Initialize states
  valueArray[0][5] = 0;
  for (int i = 1; i < numNodes; ++i) {
    valueArray[i][5] = 2;
  }

  Serial.print("myIndex: ");
  Serial.println(myIndex);
  Serial.print("currentIndex: ");
  Serial.println(currentIndex);
  Serial.print("numNodes: ");
  Serial.println(numNodes);

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

  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }

  //Generate randomness for scan time and Node ID
  start = millis();

//Can we use this to generate the random number instead?
  randomSeed(analogRead(FLOATING_PIN1)+analogRead(FLOATING_PIN2)+analogRead(IR_REC_0)+analogRead(IR_REC_120)+analogRead(IR_REC_240));
  myID = random(0,1000);
  
//  unsigned long randDelay = random(0,1000);
//  Serial.print("Random delay: "); Serial.println(randDelay);
//  while((millis() - start) < randDelay); 

  //Initialize temporary/initial values
  BLE.setLocalName("0");
  BLE.setAdvertisedService( KEY );
  BLE.setAdvertisedServiceUuid( BLE_UUID_KEY );
  BLE.addService( KEY );

  discoverNodes();
  while (Serial.read()!='d') ;
}

void loop() {
//  Serial.print("currentIndex: ");
//  Serial.println(currentIndex);
//  Serial.print(", numNodes: ");
//  Serial.println(numNodes);
  int random_int = random(1,100);
  //Pseudocode in loop:
  //Scan for state information
//  Serial.print("Scanning ");
  BLE.stopAdvertise();
  BLE.scanForUuid(BLE_UUID_KEY);    //TODO: We need to have multiple BLE_UUID_KEY
  start = millis();
  while (millis() - start < 250+random_int) { //Previously 250+random_int
    BLEDevice peripheral = BLE.available();
    if(peripheral){
      String data = peripheral.localName();
//      Serial.println(data);
      int sc_position = 0;
      for (int i = 0; i < data.length(); ++i) {
        if (data[i] == ';') {
          sc_position = i;
          i = data.length();
        }
      }
      String id_str = data;
      String state_str = data;
      id_str.remove(sc_position);
      state_str.remove(0,sc_position+1);
      uint16_t id = id_str.toInt();
      uint16_t st = state_str.toInt();
      for (int i = 0; i < numNodes; ++i) {
        if (valueArray[i][0] == id) valueArray[i][5] = st;
      }
//      delay(random(1,100)); //Preventing accidental synchronization.
//      break;
    }
  }

  //Execute node's current state
  switch (valueArray[myIndex][NUM_VALUES_PER_NODE-1]) { //valueArray[myIndex][5] is the node state.
    case 0: {
      //TODO: Turn emitter on for a cycle.
      while (!IMU.magneticFieldAvailable());
      IMU.readMagneticField(mx,my,mz);
      digitalWrite(IR_EMITTER,HIGH);
      delay(10);
      digitalWrite(IR_EMITTER,LOW);
//      for (int i = 0; i < 2; ++i) {
//        digitalWrite(IR_EMITTER,HIGH);
//        delay(10);
//        digitalWrite(IR_EMITTER,LOW);
//        delay(10);
//      }
      //Check that all other nodes are in state 4
      bool continue_flag = true;
      for (int i = 0; i < numNodes; ++i) {
        if (i != myIndex && valueArray[i][NUM_VALUES_PER_NODE-1] != 4) {  //Change this
          continue_flag = false;
        }
      }
      if (continue_flag) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 1;
      }
      break;
    }
    case 1: {
      //Receive data
      delay(100);
      currentIndex = (currentIndex+1)%numNodes;
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 2;
      break;
    }
    case 2: {
      if (valueArray[currentIndex][5] == 0) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 3;
      }
      break;
    }
    case 3: {
      //Collect data
      for (int i = 0; i < 3; ++i) {
        max_ir[i] = 0;
      }
      start = millis();
      int prev_millis = millis();
      while (millis() < start+540) {
        if (millis() != prev_millis) {
          read_IR();
          for (int i = 0; i < 3; ++i) {
            if (raw_ir[i] > max_ir[i]) {
              max_ir[i] = raw_ir[i];
            }
          }
          prev_millis = millis();
        }
      }
      Serial.print((int)getCompassHeading());
      Serial.print(",");
      Serial.print(max_ir[0]);
      Serial.print(",");
      Serial.print(max_ir[1]);
      Serial.print(",");
      Serial.println(max_ir[2]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 4;
      break;
    }
    case 4: {
      //Transmit Data
      delay(100);
      if (valueArray[currentIndex][NUM_VALUES_PER_NODE-1] == 2) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 5;
      }
      break;
    }
    case 5: {
      //State 5 is a transient state where the node decides whether to move to the emitter state or read values from a different emitter. 
      //State 5 always lasts exactly one loop cycle.
      currentIndex = (currentIndex+1)%numNodes;
      if (currentIndex == myIndex) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 0;
      }
      else {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 2;
      }
      break;
    }
    default: {
      Serial.println("ERROR: Node is in an undefined state.");
      break;
    }
  }
  
  //Transmit state information
//  Serial.print("Broadcasting ");
  BLE.stopScan();
  String nodename = String(myID);
//  nodename.concat((char)(myID>>8));
//  nodename.concat((char)(myID&0x0f));  //Start with node name
  String statestring = String(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
  String localname = nodename + ";" + statestring;
  char buf[99];
  localname.toCharArray(buf,localname.length()+1);  //Convert to C string for BLE to use, can't use arduino Strings
  BLE.setLocalName(buf);
  start = millis();
  BLE.advertise();
//  Serial.println(localname);
  delay(150-random_int); //Previously 150-random_int
//  Serial.println(statestring);
}
