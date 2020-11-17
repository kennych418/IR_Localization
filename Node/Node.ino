#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE               "59920589-d782-492d-8beb-0d211d66312f"
#define BLE_UUID_ACCELERATION               "2713" //https://www.bluetooth.com/specifications/assigned-numbers/units/

BLEService testService( BLE_UUID_TEST_SERVICE );
BLEIntCharacteristic accelerationCharacteristic( BLE_UUID_ACCELERATION, BLERead | BLENotify );

bool mode = false;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
 
  bool mode = initializeCentralorPeripheral();
  if(mode)
    Serial.println("Entering Central Mode");
  else
    Serial.println("Entering Peripheral Mode");
}

bool initializeCentralorPeripheral(){ //not capable of meeting racing conditions
  BLE.scanForUuid( BLE_UUID_TEST_SERVICE );
  delay(500); //create a random number between 500 and 2000
  //add loop?
  if(BLE.available()){ //act as central
    digitalWrite(LED_BUILTIN, LOW);
    BLE.setDeviceName( "Arduino Nano 33 BLE Central" );
    BLE.setLocalName( "Arduino Nano 33 BLE Central" );
    return true;
  }
  else{ //act as peripheral
    BLE.stopScan();
    digitalWrite(LED_BUILTIN, HIGH);
    BLE.setDeviceName( "Arduino Nano 33 BLE Peripheral" );
    BLE.setLocalName( "Arduino Nano 33 BLE Peripheral" );
    BLE.setAdvertisedService( testService );
    BLE.setAdvertisedServiceUuid( BLE_UUID_TEST_SERVICE );
    testService.addCharacteristic( accelerationCharacteristic );
    BLE.addService( testService );
    accelerationCharacteristic.writeValue( 52 );
    BLE.advertise();
    return false;
  }
}

void loop() {
  // check if a peripheral has been discovered
  
//  if(mode){
//    BLEDevice peripheral = BLE.available();
//  }
//  else{
//    
//  }
//  BLEDevice peripheral = BLE.available();
//
//  if (peripheral) {
//    // discovered a peripheral
//    Serial.println("Discovered a peripheral");
//    Serial.println("-----------------------");
//
//    // print address
//    Serial.print("Address: ");
//    Serial.println(peripheral.address());
//
//    // print the local name, if present
//    if (peripheral.hasLocalName()) {
//      Serial.print("Local Name: ");
//      Serial.println(peripheral.localName());
//    }
//
//    // print the advertised service UUIDs, if present
//    if (peripheral.hasAdvertisedServiceUuid()) {
//      Serial.print("Service UUIDs: ");
//      for (int i = 0; i < peripheral.advertisedServiceUuidCount(); i++) {
//        Serial.print(peripheral.advertisedServiceUuid(i));
//        Serial.print(" ");
//      }
//      Serial.println();
//    }
//
//    // print the RSSI
//    Serial.print("RSSI: ");
//    Serial.println(peripheral.rssi());
//
//    Serial.println();
//  }
}
