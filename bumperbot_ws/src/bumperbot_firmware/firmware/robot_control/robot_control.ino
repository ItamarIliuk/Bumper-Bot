// L298N H-Bridge Connection PINs
#define L298N_enA 25  // PWM
#define L298N_enB 26  // PWM
#define L298N_in4 19  // Dir Motor B
#define L298N_in3 4  // Dir Motor B
#define L298N_in2 32  // Dir Motor A
#define L298N_in1 33  // Dir Motor A

// Wheel Encoders Connection PINs
#define right_encoder_phaseA 0  // Interrupt
#define right_encoder_phaseB 13  // Interrupt
#define left_encoder_phaseA 27   // Interrupt
#define left_encoder_phaseB 14   // Interrupt

// Setting PWM properties
const int freq = 30000;
const int ch_A = 0;
const int ch_B = 1;
const int resolution = 8;

// Encoders
unsigned int right_encoder_A_counter = 0;
unsigned int right_encoder_B_counter = 0;
unsigned int left_encoder_A_counter = 0;
unsigned int left_encoder_B_counter = 0;
unsigned long last_millis = 0;
const unsigned long interval = 100;

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
float right_wheel_cmd_vel = 0.0; // rad/s
float left_wheel_cmd_vel = 0.0;  // rad/s
char value[] = "000";
uint8_t value_idx = 0;


void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // configure LED PWM functionalitites
  ledcSetup(ch_A, freq, resolution);
  ledcSetup(ch_B, freq, resolution);
  
  // attach the channel to the GPIO to be controlled
  ledcAttachPin(L298N_enA, ch_A);
  ledcAttachPin(L298N_enB, ch_B);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);

  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseA), rightEncoderACallback, RISING);
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseB), rightEncoderBCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseA), leftEncoderACallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseB), leftEncoderBCallback, RISING);
}

void loop() {
  // Read and Interpret Wheel Velocity Commands
  if (Serial.available())
  {
    char chr = Serial.read();
    // Right Wheel Motor
    if(chr == 'r')
    {
      is_right_wheel_cmd = true;
      is_left_wheel_cmd = false;
      value_idx = 0;
    }
    // Left Wheel Motor
    else if(chr == 'l')
    {
      is_right_wheel_cmd = false;
      is_left_wheel_cmd = true;
      value_idx = 0;
    }
    // Positive direction
    else if(chr == 'p')
    {
      if(is_right_wheel_cmd && !is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = true;
      }
      else if(is_left_wheel_cmd && !is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = true;
      }
    }
    // Negative direction
    else if(chr == 'n')
    {
      if(is_right_wheel_cmd && is_right_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
        digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
        is_right_wheel_forward = false;
      }
      else if(is_left_wheel_cmd && is_left_wheel_forward)
      {
        // change the direction of the rotation
        digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
        digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
        is_left_wheel_forward = false;
      }
    }
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atoi(value);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atoi(value);
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 3)
      {
        value[value_idx] = chr;
        value_idx++;
      }
    }
  }

  // Encoder
  unsigned long current_millis = millis();
  if(current_millis - last_millis >= interval)
  {
    String encoder_read = "ra" + String(right_encoder_A_counter) + ",rb" + String(right_encoder_B_counter) + ",la" + String(left_encoder_A_counter) + ",lb" + String(left_encoder_B_counter) + ",";
    Serial.println(encoder_read);
    last_millis = current_millis;
    right_encoder_A_counter = 0;
    right_encoder_B_counter = 0;
    left_encoder_A_counter = 0;
    left_encoder_B_counter = 0;
  }

  ledcWrite(ch_A, right_wheel_cmd_vel);
  ledcWrite(ch_B, left_wheel_cmd_vel);
}

// New pulse from Right Wheel Encoder
void rightEncoderACallback()
{
  right_encoder_A_counter++;
}

void rightEncoderBCallback()
{
  right_encoder_B_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderACallback()
{
  left_encoder_A_counter++;
}

void leftEncoderBCallback()
{
  left_encoder_B_counter++;
}
