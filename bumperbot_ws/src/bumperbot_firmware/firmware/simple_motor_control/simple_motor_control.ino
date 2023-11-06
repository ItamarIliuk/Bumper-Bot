// L298N H-Bridge Connection PINs
#define L298N_enA 25  // PWM
#define L298N_in2 33
#define L298N_in1 32

// Setting PWM properties
const int freq = 30000;
const int ch_A = 0;
const int resolution = 8;

int cmd = 0;

void setup() {
  // Set pin modes
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  
  // configure LED PWM functionalitites
  ledcSetup(ch_A, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(L298N_enA, ch_A);

  // Set Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available())
  {
    cmd = Serial.readString().toInt();
  }
  ledcWrite(ch_A, cmd);
}