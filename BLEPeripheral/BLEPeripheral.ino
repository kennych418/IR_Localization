/*characteristics. We are serving as the bulletin(peripheral) that will publish and have central devices read
 */
#include <ArduinoBLE.h>
#include <Arduino_LSM9DS1.h> //IMU


#define BLE_UUID_TEST_SERVICE               "59920589-d782-492d-8beb-0d211d66312f"
#define BLE_UUID_ACCELERATION               "2713" //https://www.bluetooth.com/specifications/assigned-numbers/units/

BLEService testService( BLE_UUID_TEST_SERVICE );
BLEIntCharacteristic accelerationCharacteristic( BLE_UUID_ACCELERATION, BLERead | BLENotify );

unsigned long start_timer = millis();
unsigned long time_to_wait_IMU = 500;

void init(){
  BLE.scanForUuid( BLE_UUID_TEST_SERVICE );
  BLEDevice peripheral = BLE.available();
  if(peripheral){
    //Function as central
    Serial.println("Configuring as central");
    Serial.print("Connected to: ");
    Serial.println(peripheral.address());
  }
  else{
    //Function as peripheral
    Serial.println("Configuring as peripheral");
    BLE.advertise();
    Serial.println("Advertising");
  }
}

bool setupBleMode()
{
  if ( !BLE.begin() )
  {
    return false;
  }

  // set advertised local name and service UUID:
  BLE.setDeviceName( "Arduino Nano 33 BLE Peripheral" );
  BLE.setLocalName( "Arduino Nano 33 BLE Peripheral" );
  BLE.setAdvertisedService( testService );
  BLE.setAdvertisedServiceUuid( BLE_UUID_TEST_SERVICE );

  // BLE add characteristics
  testService.addCharacteristic( accelerationCharacteristic );

  // add service
  BLE.addService( testService );

  // set the initial value for the characeristic:
  accelerationCharacteristic.writeValue( 52 );

  // start advertising
  BLE.advertise();

  return true;
}

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    
    if (!IMU.begin()) {
      Serial.println("Failed to initialize IMU!");
      while (1);
    }

    if( setupBleMode() )
    {
      digitalWrite(LED_BUILTIN, LOW );
    }
}

float x, y, z;

void loop() {
  // listen for BLE peripherals to connect:
   if (IMU.accelerationAvailable() ) {
            IMU.readAcceleration(x, y, z);
            int i = (int)x*100;
            String st = String(i);
            //Serial.println(st);
   }
   
  BLEDevice central = BLE.central();
  if ( central )
  {
    Serial.print( "Connected to central: " );
    Serial.println( central.address() );

    while ( central.connected() )
    {
        //For every second
      if(millis()-start_timer >= time_to_wait_IMU){
        if (IMU.accelerationAvailable()) {
            //Send IMU in format: A<imu_string>
            IMU.readAcceleration(x, y, z);
            int imu_int = (int)100*x; 
            String imu_str = String(imu_int);
            accelerationCharacteristic.writeValue('A');
            for(int i =0;i<imu_str.length();i++)
              accelerationCharacteristic.writeValue((char)imu_str[i]);
        }
        start_timer = millis(); //reset timer
      }
    } // while connected

    Serial.print( "Disconnected from central: ");
    Serial.println( central.address() );
  } // if central
  else{
    Serial.println("Waiting...");
  }
}
