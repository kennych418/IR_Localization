#define IR_EMITTER 3
#define IR_REC_0 A4
#define IR_REC_120 A0
#define IR_REC_240 A3
#define IR_READ_AVE_N 40

int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
int max_ir[3] = {0, 0, 0}; //This stores the filtered version.
const int deg_ir[3] = {0, 120, 240};
const int pin_location[3] = {IR_REC_0, IR_REC_120, IR_REC_240};
const bool rec = true;

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

float calculateDistance(){
  //This function calculates distance based on the maximum IR_raw value stored globally.
//  return -1.793*log(max_ir[0]) + 15.297;
//  return -1.396*log(max_ir[0])+13.734;
  return -1.542*log(max_ir[0])+14.13;
}

float calculateStepwiseDistance(){
  int reading = max_ir[0];
  if (reading > 272) {
    return 5.0;
  }
  else if (reading > 252) {
    return 5.0+(reading-252.0)/20.0;
  }
  else if (reading > 188) {
    return 5.5+(reading-188.0)/64.0;
  }
  else if (reading > 85) {
    return 6.5+(reading-85.0)/103.0;
  }
  else if (reading > 44) {
    return 7.5+(reading-44.0)/41.0;
  }
  else if (reading > 18) {
    return 8.5+(reading-18.0)/26.0;
  }
  else if (reading > 9) {
    return 9.5+(reading-9.0)/9.0;
  }
  else {
    return -1.396*log(reading)+13.734;
  }
}

void setup() {
  // put your setup code here, to run once:`
  pinMode(IR_EMITTER, OUTPUT);
  pinMode(IR_REC_0, INPUT);
  pinMode(IR_REC_120, INPUT);
  pinMode(IR_REC_240, INPUT);
  Serial.begin(9600);
}


int count = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (rec) {
    ++count;
    char c = Serial.read();
    while (c != '\n' && c != 'r' && c != 'b') {
      delay(1);
      c = Serial.read();
    }
    if (c == 'b') --count;
    Serial.print(count);
    Serial.print(',');
    for (int i = 0; i < 3; ++i) {
      max_ir[i] = 0;
    }
    for (int n = 0; n < 12; ++n) {
      read_IR();
      for (int i = 0; i < 3; ++i) {
        if (raw_ir[i] > max_ir[i]) {
          max_ir[i] = raw_ir[i];
        }
      }
      delay(1);
    }
    for (int i = 0; i < 3; ++i) {
//      Serial.print(deg_ir[i]);
//      Serial.print(": ");
      Serial.print(max_ir[i]);
      Serial.print(',');
//      Serial.println();
    }
//    Serial.println(calculateDistance());
//    Serial.println(calculateStepwiseDistance());
    Serial.println();

  }
  else {
    digitalWrite(IR_EMITTER, HIGH);
    delay(10);
    digitalWrite(IR_EMITTER, LOW);
    delay(10);
  }

  //  while(millis()<current_tick+1000){}
  //  analogRead(IR_REC_0);
  //  analogRead(IR_REC_120);
  //  analogRead(IR_REC_240);
}
