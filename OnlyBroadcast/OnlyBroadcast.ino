#include <ArduinoBLE.h>

#define BLE_UUID_KEY        "59920589-d782-492d-8beb-0d211d66312f"

#define FLOATING_PIN1 A7
#define FLOATING_PIN2 A6

unsigned long NODE_NAME = 1;

///////// SET HOW LONG YOU WANT TO SCAN/BROADCAST DATA IN MILLISECONDS///////////////
unsigned long SCAN_TIME = 1300;    
unsigned long BROADCAST_TIME = 2000; 
////////////////////////////////////////////////////////////////////////////////////

int IRLED0 = 0;
int IRLED120 = 0;
int IRLED240 = 0;
int compass = 0;

BLEService KEY( BLE_UUID_KEY ); 
bool scanning = true;
unsigned long start = 0;

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
  IRLED0 = 0;
  IRLED120 = 0;
  IRLED240 = 0;
  compass = 0;
  
  //////////////////////////////////////////////////////////

  //Switch from broadcasting to scanning
  if(((millis() - start) > SCAN_TIME) && !scanning){  
    Serial.println("Scanning");
    BLE.stopAdvertise();
    BLE.scanForUuid(BLE_UUID_KEY);
    scanning = true;
    start = millis();
  }

  //Switch from scanning to broadcasting
  if(((millis() - start) > BROADCAST_TIME) && scanning){  
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
    BLEDevice peripheral = BLE.available();
    if(peripheral){
      Serial.println((millis() - start));
      String data = peripheral.localName();
      Serial.println(data);
    }
  }

}
