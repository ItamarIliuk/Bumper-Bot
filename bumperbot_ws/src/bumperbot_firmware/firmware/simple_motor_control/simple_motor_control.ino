#define enA 9
#define in1 6


void setup() {
  // Initialize motor pins
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);

  // Set initial rotation direction
  digitalWrite(in1, HIGH);

  // Start the Serial communication with ROS 2
  Serial.begin(115200);
  Serial.setTimeout(1);
}

void loop() {
  // Check for new messages
  if (Serial.available())
  {
    int vel = Serial.readString().toInt();
    analogWrite(enA, vel); // Send PWM signal to L298N Enable pin
  }
  delay(0.1);
}