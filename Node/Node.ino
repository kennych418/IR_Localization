#include <ArduinoBLE.h>

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

void setup() {
  Serial.begin(9600);
  while (!Serial);

  pinMode(LED_BUILTIN, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
}

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
        if(peripheral.connect()){
          while(!peripheral.discoverAttributes());
          Serial.println("Connected");
          BLECharacteristic ENABLE = peripheral.characteristic(BLE_UUID_ENABLE);
          BLECharacteristic IR = peripheral.characteristic(BLE_UUID_IR0);
          BLECharacteristic TIME = peripheral.characteristic(BLE_UUID_TIME);
          while(peripheral.connected()){
            unsigned long measurement = 0;
            unsigned long timemeasurement = 0;
            IR.writeValue(measurement); // <----------------------------Write values in here
            TIME.writeValue(timemeasurement); // <----------------------------Write values in here
            ENABLE.readValue(enable); // <----------------------Read values here
            Serial.println(enable);
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
    BLE.advertise();
    
    start = millis();    
    while(1){
      BLE.poll();

      if((millis()-start) > 1000){    //<----------------------------Set timer
        enableCharacteristic.writeValue(0); //<----------------------------Alternate your emitter + Read/Write Stuff
        start = millis();
      }
      else{
        enableCharacteristic.writeValue(1);      //<----------------------------Alternate your emitter + Read/Write Stuff
      }

      BLE.setDeviceName("IRNode");
      BLE.setLocalName("IRNode");
      BLE.setAdvertisedService( DataService );
      BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );
      BLE.advertise();
    }
}
