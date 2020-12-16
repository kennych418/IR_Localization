#include <ArduinoBLE.h>

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

BLEService KEY( BLE_UUID_KEY ); 
bool scanning = true;
unsigned long start = 0;

void bubbleSortIDs() {
  //Sort the entries in column 1 of the data array for rows 0 to numNodes-1
  //Bubble sort is ok here due to small number of entries.
  return;
}

bool discoverNodes() {
  //Add the code to discover the IDs of remote nodes here.
  //This can be terminated with a key press similar to the calibration step.
  while (Serial.read() != d) {
    int random_int = random(1,100);
    //Pseudocode in loop:
    //Scan for state information
    Serial.println("Scanning");
    BLE.stopAdvertise();
    BLE.scanForUuid(BLE_UUID_KEY);    //TODO: We need to have multiple BLE_UUID_KEY
    start = millis();
    while (millis() - start < 350+random_int) {
      BLEDevice peripheral = BLE.available();
      if(peripheral){
        String data = peripheral.localName();
        Serial.println(data);
        int ID = data.toInt();
        bool newFlag = true;
        for (int i = 0; i < numNodes; ++i) {
          if (valueArray[i][0] == ID) newFlag = false;
        }
        if (newFlag) {
          if (numNodes+1 > MAX_NUM_NODES) return false;
          valueArray[numNodes][0] = ID;
          numNodes = numNodes + 1;
        }
  //      delay(random(1,100)); //Preventing accidental synchronization.
  //      break;
      }
    }
    Serial.println("Broadcasting");
    BLE.stopScan();
    String nodename = String(myID);
  //  nodename.concat((char)(myID>>8));
  //  nodename.concat((char)(myID&0x0f));  //Start with node name
    char buf[99];
    nodename.toCharArray(buf,nodename.length()+1);  //Convert to C string for BLE to use, can't use arduino Strings
    BLE.setLocalName(buf);
    start = millis();
    BLE.advertise();
    Serial.println(localname);
    delay(150-random_int);
  }
  bubbleSortIDs();
  return true;
}

void setup() {
  Serial.begin(9600);
  while (!Serial);

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
}

void loop() {
  int random_int = random(1,100);
  //Pseudocode in loop:
  //Scan for state information
  Serial.println("Scanning");
  BLE.stopAdvertise();
  BLE.scanForUuid(BLE_UUID_KEY);    //TODO: We need to have multiple BLE_UUID_KEY
  start = millis();
  while (millis() - start < 350+random_int) {
    BLEDevice peripheral = BLE.available();
    if(peripheral){
      String data = peripheral.localName();
      Serial.println(data);
//      delay(random(1,100)); //Preventing accidental synchronization.
//      break;
    }
  }

  //Execute node's current state
  switch (valueArray[myIndex][NUM_VALUES_PER_NODE-1]) { //valueArray[myIndex][5] is the node state.
    case 0:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    case 1:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    case 2:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    case 3:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    case 4:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    case 5:
      Serial.println(valueArray[myIndex][NUM_VALUES_PER_NODE-1]);
      valueArray[myIndex][NUM_VALUES_PER_NODE-1] = (valueArray[myIndex][NUM_VALUES_PER_NODE-1]+1)%NUM_STATES;
      break;
    default:
      Serial.println("ERROR: Node is in an undefined state.");
      break;
  }
  
  //Transmit state information
  Serial.println("Broadcasting");
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
  Serial.println(localname);
  delay(150-random_int);
}
