// L298N H-Bridge Connection PINs
#define L298N_enA 9  // PWM
#define L298N_enB 11 // PWM
#define L298N_in1 8
#define L298N_in2 7
#define L298N_in3 13
#define L298N_in4 12

// Wheel Encoders Connection PINs
#define right_encoder_phaseB 3  // Interrupt
#define left_encoder_phaseB 2   // Interrupt

// Wheel Motors Mechanical Specs
const unsigned int gear_ratio = 35;
const unsigned int encoder_ratio = 11; 
const unsigned long interval = 100;
const unsigned int n_of_interval = 1000/interval;
const float speed_conversion = (n_of_interval * PI) / (gear_ratio * encoder_ratio); // 0.081558418

unsigned int right_encoder_counter = 0;
unsigned int left_encoder_counter = 0;
unsigned long last_millis = 0;
float right_wheel_vel = 0.0;  // rad/s
float left_wheel_vel = 0.0;   // rad/s

bool is_right_wheel_cmd = false;
bool is_left_wheel_cmd = false;
bool is_right_wheel_forward = true;
bool is_left_wheel_forward = true;
float right_wheel_cmd_vel = 0.0; // rad/s
float left_wheel_cmd_vel = 0.0;  // rad/s
int right_wheel_cmd = 0;
int left_wheel_cmd = 0;
char value[] = "00000";
uint8_t value_idx = 0;

long map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setup() {
  // Init L298N H-Bridge Connection PINs
  pinMode(L298N_enA, OUTPUT);
  pinMode(L298N_enB, OUTPUT);
  pinMode(L298N_in1, OUTPUT);
  pinMode(L298N_in2, OUTPUT);
  pinMode(L298N_in3, OUTPUT);
  pinMode(L298N_in4, OUTPUT);

  // Set Motor Rotation Direction
  digitalWrite(L298N_in1, HIGH);
  digitalWrite(L298N_in2, LOW);
  digitalWrite(L298N_in3, HIGH);
  digitalWrite(L298N_in4, LOW);

  Serial.begin(115200);
  Serial.setTimeout(1);

  // Set Callback for Wheel Encoders Pulse
  attachInterrupt(digitalPinToInterrupt(right_encoder_phaseB), rightEncoderCallback, RISING);
  attachInterrupt(digitalPinToInterrupt(left_encoder_phaseB), leftEncoderCallback, RISING);
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
    // Separator
    else if(chr == ',')
    {
      if(is_right_wheel_cmd)
      {
        right_wheel_cmd_vel = atof(value);
        // Change the rotation direction
        if(is_right_wheel_forward && right_wheel_cmd_vel < 0)
        {
          digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
          digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
          is_right_wheel_forward = false;
        }
        else if(!is_right_wheel_forward && right_wheel_cmd_vel > 0)
        {
          digitalWrite(L298N_in1, HIGH - digitalRead(L298N_in1));
          digitalWrite(L298N_in2, HIGH - digitalRead(L298N_in2));
          is_right_wheel_forward = true;
        }
        right_wheel_cmd_vel = abs(right_wheel_cmd_vel);
      }
      else if(is_left_wheel_cmd)
      {
        left_wheel_cmd_vel = atof(value);
        // Change the rotation direction
        if(is_left_wheel_forward && left_wheel_cmd_vel < 0)
        {
          digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
          digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
          is_left_wheel_forward = false;
        }
        else if(!is_left_wheel_forward && left_wheel_cmd_vel > 0)
        {
          digitalWrite(L298N_in3, HIGH - digitalRead(L298N_in3));
          digitalWrite(L298N_in4, HIGH - digitalRead(L298N_in4));
          is_left_wheel_forward = true;
        }
        left_wheel_cmd_vel = abs(left_wheel_cmd_vel);
      }
      // Reset for next command
      value_idx = 0;
      value[0] = '0';
      value[1] = '0';
      value[2] = '0';
      value[3] = '0';
      value[4] = '0';
      value[5] = '\0';
    }
    // Command Value
    else
    {
      if(value_idx < 5)
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
    right_wheel_vel = right_encoder_counter * speed_conversion;
    left_wheel_vel = left_encoder_counter * speed_conversion;
    last_millis = current_millis;
    right_encoder_counter = 0;
    left_encoder_counter = 0;
    String encoder_read = "r" + String(right_wheel_vel) + ",l" + String(left_wheel_vel) + ",";
    Serial.println(encoder_read);
  }

  right_wheel_cmd = map(right_wheel_cmd_vel, 0.0, 8.9, 0.0, 255.0);
  left_wheel_cmd = map(left_wheel_cmd_vel, 0.0, 8.9, 0.0, 255.0);
  analogWrite(L298N_enA, right_wheel_cmd);
  analogWrite(L298N_enB, left_wheel_cmd);
}

// New pulse from Right Wheel Encoder
void rightEncoderCallback()
{
  right_encoder_counter++;
}

// New pulse from Left Wheel Encoder
void leftEncoderCallback()
{
  left_encoder_counter++;
}
