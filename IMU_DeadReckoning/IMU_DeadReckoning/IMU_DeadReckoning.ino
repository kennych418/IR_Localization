
#include <Arduino_LSM9DS1.h>


float ax, ay, az;
float gx, gy, gz;
float gx_off, gy_off, gz_off;
float mx, my, mz;
float mx_off, my_off, mz_off;

float x = 0;
float y = 0;
//float z = 0;
//float phi = 0;
float theta = 0;
float v_x = 0;
float v_y = 0;
//float v_z = 0;

float getM() {
  return 0;
}

float getTheta() {
  return 0;
}

bool updateCoordinates(float M, float theta) {
  x = M*cos(theta);
  y = M*sin(theta);
  
  return true;
}

bool calibrateOffestsIMU() {
  int start = millis();
//  float gx_max = 0;
//  float gy_max = 0;
//  float gz_max = 0;
//  float gx_min = 0;
//  float gy_min = 0;
//  float gz_min = 0;
  int count = 0;
  double gx_total = 0;
  double gy_total = 0;
  double gz_total = 0;
  float mx_max = 0;
  float my_max = 0;
  float mz_max = 0;
  float mx_min = 0;
  float my_min = 0;
  float mz_min = 0;
  while (millis() < start + 1000) {
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);
//    if (gx > gx_max) gx_max = gx;
//    if (gx < gx_min) gx_min = gx;
//    if (gy > gy_max) gy_max = gy;
//    if (gy < gy_min) gy_min = gy;
//    if (gz > gz_max) gz_max = gz;
//    if (gz < gz_min) gz_min = gz;
    gx_total += gx;
    gy_total += gy;
    gz_total += gz;
    ++count;
    if (mx > mx_max) mx_max = mx;
    if (mx < mx_min) mx_min = mx;
    if (my > my_max) my_max = my;
    if (my < my_min) my_min = my;
    if (mz > mz_max) mz_max = mz;
    if (mz < mz_min) mz_min = mz;
  }
//  gx_off = (gx_max+gx_min)/2;
//  gy_off = (gy_max+gy_min)/2;
//  gz_off = (gz_max+gz_min)/2;
  gx_off = gx_total/count;
  gy_off = gy_total/count;
  gz_off = gz_total/count;
  mx_off = (mx_max+mx_min)/2;
  my_off = (my_max+my_min)/2;
  mz_off = (mz_max+mz_min)/2;

  return true;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }
  calibrateOffestsIMU();
}


void loop() {
  // put your main code here, to run repeatedly:
  if (IMU.accelerationAvailable() ) {
    IMU.readAcceleration(ax, ay, az);
    Serial.print("Acc_x = ");
    Serial.print(ax);
    Serial.print(", Acc_y = ");
    Serial.print(ay);
    Serial.print(", Acc_z = ");
    Serial.print(az);
    IMU.readGyroscope(gx, gy, gz);
    Serial.print(", Gyr_x = ");
//    Serial.print(gx);
    Serial.print(gx-gx_off);
    Serial.print(", Gyr_y = ");
//    Serial.print(gy);
    Serial.print(gy-gy_off);
    Serial.print(", Gyr_z = ");
//    Serial.print(gz);
    Serial.print(gz-gz_off);
    IMU.readMagneticField(mx, my, mz);
    Serial.print(", Mag_x = ");
    Serial.print(mx);
//    Serial.print(mx-mx_off);
    Serial.print(", Mag_y = ");
    Serial.print(my);
//    Serial.print(my-my_off);
    Serial.print(", Mag_z = ");
    Serial.println(mz);
//    Serial.println(mz-mz_off);
    //int i = (int)x*100;
    //String st = String(i);
    //Serial.println(st);
  }
}
