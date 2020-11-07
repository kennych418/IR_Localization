/*Arduino is BLE server publishing the accelerometer and proximity sensor data in 2 separate 
 *characteristics. We are serving as the bulletin(peripheral) that will publish and have central devices read
 */
#include <ArduinoBLE.h>

#define BLE_UUID_TEST_SERVICE               "59920589-d782-492d-8beb-0d211d66312f"

void setup() {
  // put your setup code here, to run once:
    Serial.begin(9600);
    while(!Serial);
    
    if ( !BLE.begin() ){
      Serial.println("Failed to setup BLE");
      return;
    }
    else{
      Serial.println("BLE Central");
    }

    BLE.scanForUuid("59920589-d782-492d-8beb-0d211d66312f");

    digitalWrite(LED_BUILTIN, HIGH );
}

void loop() {
  BLEDevice peripheral = BLE.available();
  if ( peripheral ) {
    Serial.print("Found ");
    Serial.print(peripheral.address());
    Serial.print(" '");
    Serial.print(peripheral.localName());
    Serial.print("' ");
    Serial.print(peripheral.advertisedServiceUuid());
    Serial.println();
  }
}
