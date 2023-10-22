#define L298N_enA 9  // PWM
#define L298N_enB 11 // PWM
#define L298N_in1 8
#define L298N_in2 7
#define L298N_in3 13
#define L298N_in4 12

int cmd = 0;

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set initial rotation direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  if (Serial.available())
  {
    cmd = Serial.readString().toInt();
  }
  analogWrite(L298N_enA, cmd);
}
