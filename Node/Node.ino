#include <ArduinoBLE.h>

#define BLE_UUID_IDANDENABLE_SERVICE        "59920589-d782-492d-8beb-0d211d66312f"
#define BLE_UUID_DATA_SERVICE               "19B10000-E8F2-537E-4F6C-D104768A1214"

#define BLE_UUID_IDS0                       "2813"
#define BLE_UUID_IDS1                       "2456"
#define BLE_UUID_IDS2                       "2690"
#define BLE_UUID_IDS3                       "2344"
#define BLE_UUID_IDS4                       "1234"
#define BLE_UUID_ENABLE                     "2134"
#define BLE_UUID_DIST                       "271C" 
#define BLE_UUID_TIME                       "2703" 

#define FLOATING_PIN1 A7
#define FLOATING_PIN2 A6

unsigned long ID = 0;

BLEService IDandEnableService( BLE_UUID_IDANDENABLE_SERVICE );
BLEService DataService( BLE_UUID_DATA_SERVICE );

BLEUnsignedLongCharacteristic IDs0Characteristic(BLE_UUID_IDS0, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic IDs1Characteristic(BLE_UUID_IDS1, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic IDs2Characteristic(BLE_UUID_IDS2, BLEWrite|BLERead);
//BLEUnsignedLongCharacteristic IDs3Characteristic(BLE_UUID_IDS3, BLEWrite|BLERead);
//BLEUnsignedLongCharacteristic IDs4Characteristic(BLE_UUID_IDS4, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic enableCharacteristic(BLE_UUID_ENABLE, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic timeCharacteristic( BLE_UUID_TIME, BLEWrite|BLERead);
BLEUnsignedLongCharacteristic distCharacteristic( BLE_UUID_DIST, BLEWrite|BLERead);

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
  ID = randDelay;
  Serial.print("Random delay & ID: "); Serial.println(randDelay);
  while((millis() - start) < randDelay); 




  /////////////////Central/////////////////
  int trials = 6000;
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
          Serial.println("Connected");
          while(!peripheral.discoverAttributes());
          BLECharacteristic IDS = peripheral.characteristic(BLE_UUID_IDS0);
          BLECharacteristic ENABLE = peripheral.characteristic(BLE_UUID_ENABLE);
          BLECharacteristic DIST = peripheral.characteristic(BLE_UUID_DIST);
          BLECharacteristic TIME = peripheral.characteristic(BLE_UUID_TIME);
          while(peripheral.connected()){
            unsigned long distancevalue = 0;
            IDS.writeValue(ID);
            DIST.readValue(distancevalue);
            Serial.println(distancevalue);
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

    int count = 0;
    unsigned long ID = 0;

    BLE.setAdvertisedService( DataService );
    BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );
    IDandEnableService.addCharacteristic( IDs0Characteristic );
    IDandEnableService.addCharacteristic( IDs1Characteristic );
    IDandEnableService.addCharacteristic( IDs2Characteristic );
    IDandEnableService.addCharacteristic( enableCharacteristic );
    DataService.addCharacteristic( timeCharacteristic );
    DataService.addCharacteristic( distCharacteristic );
    BLE.addService( IDandEnableService );
    BLE.addService( DataService );
    IDs0Characteristic.writeValue( 0 );
    IDs1Characteristic.writeValue( 0 );
    IDs2Characteristic.writeValue( 0 );
    enableCharacteristic.writeValue( 2 );
    timeCharacteristic.writeValue( 0 );
    distCharacteristic.writeValue( 0 );
    BLE.advertise();

    while(1){
      BLE.poll();

      if(count < 2000)
        count++;
      else
        count = 0;
      //timeCharacteristic.writeValue( count );
      //distCharacteristic.writeValue( count );

      //for (int centralIdx; centralIdx < ){
      //  
      //}


      if(BLE.centralCount() < 3){
        BLE.setDeviceName("IRNode");
        BLE.setLocalName("IRNode");
        BLE.setAdvertisedService( DataService );
        BLE.setAdvertisedServiceUuid( BLE_UUID_DATA_SERVICE );
        Serial.print("Num central: ");
        Serial.println(BLE.centralCount());
        BLE.advertise();
      }
      else{
        Serial.print("Num central: ");
        Serial.println(BLE.centralCount());
      }
      //IDs0Characteristic.readValue(ID);
      //Serial.println(ID);
//      if(central){
//        //if(central.deviceName() != "IRNode"){
//        //  Serial.println("Detected but Missed");
//        //  continue;
//        //}
//        unsigned long count = 0;
//        unsigned long ID = 0;
//        while(central.connected()){
//          if(count > 2000){
//            count = 0;
//            //IDs0Characteristic.writeValue(0);
//          }
//          else
//            count++;
//          timeCharacteristic.writeValue( count );
//          distCharacteristic.writeValue( count );
//          IDs0Characteristic.readValue(ID);
//          Serial.println(ID);
//        }
//      }
    }
}
