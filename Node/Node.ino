#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE               "59920589-d782-492d-8beb-0d211d66312f"
#define BLE_UUID_DIST                       "271C" //https://www.bluetooth.com/specifications/assigned-numbers/units/
#define BLE_UUID_TIME                       "2703" //https://www.bluetooth.com/specifications/assigned-numbers/units/

#define FLOATING_PIN1 A7
#define FLOATING_PIN2 A6

BLEService testService( BLE_UUID_TEST_SERVICE );
BLEUnsignedLongCharacteristic timeCharacteristic( BLE_UUID_TIME, BLERead | BLENotify );
BLEUnsignedLongCharacteristic distCharacteristic( BLE_UUID_DIST, BLERead | BLENotify );

bool mode = false;
BLEDevice peripheral;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
 
  mode = initializeCentralorPeripheral();
}

bool initializeCentralorPeripheral(){ //not capable of meeting racing conditions
  BLE.scanForUuid( BLE_UUID_TEST_SERVICE );
  
  unsigned long start = millis();
  unsigned long rand1 = analogRead( FLOATING_PIN1 )/6;
  unsigned long rand2 = analogRead( FLOATING_PIN2 )/4;
  unsigned long randDelay = rand1*rand2;
  
  while((millis() - start) < randDelay) Serial.println(randDelay); 
  peripheral = BLE.available();
  
  if(peripheral){ //act as central
    digitalWrite(LED_BUILTIN, LOW);
    BLE.setDeviceName( "Arduino Nano 33 BLE Central" );
    BLE.setLocalName( "Arduino Nano 33 BLE Central" );
    Serial.println("Entering Central Mode");
    return true;
  }
  else{ //act as peripheral
    BLE.stopScan();
    digitalWrite(LED_BUILTIN, HIGH);
    BLE.setDeviceName( "Arduino Nano 33 BLE Peripheral" );
    BLE.setLocalName( "Arduino Nano 33 BLE Peripheral" );
    BLE.setAdvertisedService( testService );
    BLE.setAdvertisedServiceUuid( BLE_UUID_TEST_SERVICE );
    testService.addCharacteristic( timeCharacteristic );
    testService.addCharacteristic( distCharacteristic );
    BLE.addService( testService );
    timeCharacteristic.writeValue( 0 );
    distCharacteristic.writeValue( 0 );
    BLE.advertise();
    Serial.println("Entering Peripheral Mode");
    return false;
  }
}

void loop() {
  if(mode){ //Central
    while(!peripheral.connect()) Serial.println("Connecting...");
    while(peripheral.connected()){
      Serial.println("Connected");
    }
    initializeCentralorPeripheral();
  }
  else{ //Peripheral
    BLEDevice central = BLE.central();
    Serial.println("Searching");
    while(central){
      Serial.print( "Connected to central: " );
      Serial.println( central.deviceName() );
      timeCharacteristic.writeValue( 0 );
      distCharacteristic.writeValue( 0 );
    }
  }
}
