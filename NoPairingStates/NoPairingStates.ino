//#include <TensorFlowLite.h>
//#include "tensorflow/lite/micro/all_ops_resolver.h"
//#include "tensorflow/lite/micro/micro_error_reporter.h"
//#include "tensorflow/lite/micro/micro_interpreter.h"
//#include "tensorflow/lite/schema/schema_generated.h"
//#include "tensorflow/lite/version.h"
//
//#include "dist_model.h"
//#include "angle_model.h"

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
#define NUM_VALUES_PER_NODE 7 //These 7 values are as follows: ID,compass,IR0,IR120,IR240,data_good,state
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
bool receiving_data = false;
unsigned long start = 0;




//// Globals, used for compatibility with Arduino-style sketches.
//namespace {
//tflite::ErrorReporter* error_reporter = nullptr;
//const tflite::Model* dist_model = nullptr;
//const tflite::Model* angle_model = nullptr;
//tflite::MicroInterpreter* interpreter = nullptr;
//TfLiteTensor* input = nullptr;
//TfLiteTensor* output = nullptr;
//int inference_count = 0;
//
//// Create an area of memory to use for input, output, and intermediate arrays.
//// Minimum arena size, at the time of writing. After allocating tensors
//// you can retrieve this value by invoking interpreter.arena_used_bytes().
//const int kModelArenaSize = 64*1024;//2468;
//// Extra headroom for model + alignment + future interpreter changes.
//const int kExtraArenaSize = 560 + 16 + 100;
//const int kTensorArenaSize = kModelArenaSize + kExtraArenaSize;
//uint8_t tensor_arena[kTensorArenaSize];
//}  // namespace




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
  valueArray[0][NUM_VALUES_PER_NODE-1] = 0;
  for (int i = 1; i < numNodes; ++i) {
    valueArray[i][NUM_VALUES_PER_NODE-1] = 2;
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
  BLE.setAdvertisingInterval(60);

  discoverNodes();

//  Serial.println("Starting TensorFlow Lite setup...");
//
//  static tflite::MicroErrorReporter micro_error_reporter;
//  error_reporter = &micro_error_reporter;
//
//  Serial.println("...");
//
//  // Map the model into a usable data structure. This doesn't involve any
//  // copying or parsing, it's a very lightweight operation.
//  dist_model = tflite::GetModel(d_model);
//  if (dist_model->version() != TFLITE_SCHEMA_VERSION) {
//    Serial.println("Error");
//    TF_LITE_REPORT_ERROR(error_reporter,
//                         "Model provided is schema version %d not equal "
//                         "to supported version %d.",
//                         dist_model->version(), TFLITE_SCHEMA_VERSION);
//    return;
//  }
//  Serial.println("...");
//
//  // This pulls in all the operation implementations we need.
//  // NOLINTNEXTLINE(runtime-global-variables)
//  static tflite::AllOpsResolver resolver;
//  Serial.println("...");
//
//  // Build an interpreter to run the model with.
//  static tflite::MicroInterpreter static_interpreter(
//      dist_model, resolver, tensor_arena, kTensorArenaSize, error_reporter);
//  interpreter = &static_interpreter;
//  Serial.println("...");
//
//  Serial.println("Allocating space for tensors");
//
//  // Allocate memory from the tensor_arena for the model's tensors.
//  TfLiteStatus allocate_status = interpreter->AllocateTensors();
//  if (allocate_status != kTfLiteOk) {
//    Serial.println("Error");
//    TF_LITE_REPORT_ERROR(error_reporter, "AllocateTensors() failed");
//    return;
//  }
//
//  Serial.print("Arena used bytes: ");
//  Serial.println(interpreter->arena_used_bytes());
//
//  // Obtain pointers to the model's input and output tensors.
//  input = interpreter->input(0);
//  output = interpreter->output(0);
//
//  // Keep track of how many inferences we have performed.
//  inference_count = 0;
//
//  Serial.println("TensorFlow Lite setup complete.");
  
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
//  Serial.print("About to stop advertise in state receive ... ");
  BLE.stopAdvertise();
//  Serial.print("stopped advertising, scanning for uuid ... ");
  BLE.scanForUuid(BLE_UUID_KEY);    //TODO: We need to have multiple BLE_UUID_KEY
//  Serial.print("scanned for uuid ... entering receive loop ... ");
  start = millis();
  while (millis() - start < 250+random_int) { //Previously 250+random_int
    BLEDevice peripheral = BLE.available();
    if(peripheral){
      String data = peripheral.localName();
//      Serial.print("Received ");
//      Serial.println(data);

      if (data[0] == '_') {
        if (receiving_data) {
          //This is a data stream not a state update.
          data.remove(0,1);
          int ID = data.toInt();
          data.remove(0,data.indexOf(',')+1);
          for (int i = 0; i < numNodes; ++i) {
            if (ID == valueArray[i][0]) {
//              Serial.println(data);
              valueArray[i][1] = data.toInt();
              data.remove(0,data.indexOf(',')+1);
//              Serial.println(data);
              valueArray[i][2] = data.toInt();
              data.remove(0,data.indexOf(',')+1);
//              Serial.println(data);
              valueArray[i][3] = data.toInt();
              data.remove(0,data.indexOf(',')+1);
//              Serial.println(data);
              valueArray[i][4] = data.toInt();
              valueArray[i][5] = 1;
            }
          }
          receiving_data = false;
          //Parse the string
          //Store in valueArray
          //Update the data_good field to 1
        }
      }
      else {
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
          if (valueArray[i][0] == id) valueArray[i][NUM_VALUES_PER_NODE-1] = st;
        }
  //      delay(random(1,100)); //Preventing accidental synchronization.
  //      break;
      }
    }
  }
//  Serial.println("after serial receive loop.");

  //Execute node's current state
  switch (valueArray[myIndex][NUM_VALUES_PER_NODE-1]) { //valueArray[myIndex][NUM_VALUES_PER_NODE-1] is the node state.
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
        for (int i = 0; i < numNodes; ++i) {  //Clear the data good flag for values that still need to be written.
          if (i != myIndex) {
            valueArray[i][5] = 0;
          }
        }
      }
      break;
    }
    case 1: {
      //Receive data
      receiving_data = false;
      bool data_received = true;
      for (int i = 0; i < numNodes; ++i) {
        if (i != myIndex && valueArray[i][NUM_VALUES_PER_NODE-2] == 0) {
          data_received = false;
          receiving_data = true;
        }
      }
      if (data_received) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 2;
        Serial.print((int)getCompassHeading());
//        input->data.f[0] = 0; //Not using compass directly
        Serial.print(",");
        Serial.print(max_ir[0]);
//        input->data.f[1] = max_ir[0];
        Serial.print(",");
        Serial.print(max_ir[1]);
//        input->data.f[2] = max_ir[1];
        Serial.print(",");
        Serial.print(max_ir[2]);
//        input->data.f[3] = max_ir[2];
        Serial.print(",");
        Serial.print(valueArray[(myIndex+1)%2][1]);
//        input->data.f[4] = 0; //Not using compass directly
        Serial.print(",");
        Serial.print(valueArray[(myIndex+1)%2][2]);
//        input->data.f[5] = valueArray[(myIndex+1)%2][2];
        Serial.print(",");
        Serial.print(valueArray[(myIndex+1)%2][3]);
//        input->data.f[6] = valueArray[(myIndex+1)%2][2];
        Serial.print(",");
        Serial.println(valueArray[(myIndex+1)%2][4]);
//        input->data.f[7] = valueArray[(myIndex+1)%2][2];
//        input->data.f[8] = (valueArray[(myIndex+1)%2][1]-(uint16_t)getCompassHeading()+180)%360;
        currentIndex = (currentIndex+1)%numNodes;
//        TfLiteStatus invoke_status = interpreter->Invoke();
//        if (invoke_status != kTfLiteOk) {
//          TF_LITE_REPORT_ERROR(error_reporter, "Invoke failed\n");
//          return;
//        }
//        Serial.print("Distance: ");
//        Serial.println(output->data.f[0]*10);
      }
      break;
    }
    case 2: {
      if (valueArray[currentIndex][NUM_VALUES_PER_NODE-1] == 0) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 3;
      }
      break;
    }
    case 3: {
//      Serial.print("At the start of state 3 ... ");
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
//      Serial.print((int)getCompassHeading());
//      Serial.print(",");
//      Serial.print(max_ir[0]);
//      Serial.print(",");
//      Serial.print(max_ir[1]);
//      Serial.print(",");
//      Serial.print(max_ir[2]);
//      Serial.print(" at the end of state 3 ... ");
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 4;
//      Serial.println("next state set to 4");
      break;
    }
    case 4: {
      if (valueArray[currentIndex][NUM_VALUES_PER_NODE-1] == 2) {
        valueArray[myIndex][NUM_VALUES_PER_NODE-1] = 5;
      }
      else {
        //Transmit Data
//        Serial.print("Before stop scan ... ");
        BLE.stopScan();
//        Serial.print("after stop scan before stop advertise ... ");
//        BLE.stopAdvertise();
//        Serial.print("after stop advertise ... ");
        String nodename = String(myID);
        String localname = "_" + nodename + "," + String((int)getCompassHeading()) + "," + String(max_ir[0]) + "," + String(max_ir[1]) + "," + String(max_ir[2]);
        char buf[99];
  //      Serial.print("Sending: ");
  //      Serial.println(localname);
        localname.toCharArray(buf,localname.length()+1);  //Convert to C string for BLE to use, can't use arduino Strings
        BLE.setLocalName(buf);
//        start = millis();
//        Serial.print("before state 4 advertise ... remote state is ");
//        Serial.print(valueArray[currentIndex][NUM_VALUES_PER_NODE-1]);
        BLE.advertise();
//        Serial.println(" ... after advertise.");
        delay(150);
        BLE.stopAdvertise();
        delay(500);
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
//  Serial.print("About to stop advertising in broadcast ... ");
  BLE.stopAdvertise();
//  Serial.print("stopped advertising, about to stop scan ... ");
  BLE.stopScan();
//  Serial.println("stopped scanning.");
  String nodename = String(myID);
//  nodename.concat((char)(myID>>8));
//  nodename.concat((char)(myID&0x0f));  //Start with node name
  String statestring = String(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
  String localname = nodename + ";" + statestring;
  char buf[99];
  localname.toCharArray(buf,localname.length()+1);  //Convert to C string for BLE to use, can't use arduino Strings
  BLE.setLocalName(buf);
//  start = millis();
//  Serial.print("About to advertise for broadcast state ... ");
  BLE.advertise();
//  Serial.println("after advertise command.");
//  Serial.println(localname);
  delay(150-random_int); //Previously 150-random_int
//  Serial.println(statestring);
}
