#define IR_EMITTER 3
#define IR_REC_0 A4
#define IR_REC_120 A0
#define IR_REC_240 A3
#define IR_READ_AVE_N 40

int raw_ir[3] = {0, 0, 0}; //This allows iteration more easily.
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
  return -1.793*log(raw_ir[0]) + 15.297;
}

void setup() {
  // put your setup code here, to run once:
  pinMode(IR_EMITTER, OUTPUT);
  pinMode(IR_REC_0, INPUT);
  pinMode(IR_REC_120, INPUT);
  pinMode(IR_REC_240, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (rec) {
    while (Serial.read() != '\n') {
      delay(1);
    }
    int max_ir[3] = {0, 0, 0};
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
      Serial.print(deg_ir[i]);
      Serial.print(": ");
      Serial.println(max_ir[i]);
    }
    Serial.println(calculateDistance());
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
