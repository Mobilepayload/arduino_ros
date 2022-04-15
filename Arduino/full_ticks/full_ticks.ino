// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3

#define ENC_IN_RIGHT_B 4
#define ENC_IN_LEFT_B 5

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;

// 100ms interval for measurements
const int interval = 100;
long previousMillis = 0;
long currentMillis = 0;
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if(val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_pulse_count == encoder_maximum) {
      right_wheel_pulse_count = encoder_minimum;
    }
    else {
      right_wheel_pulse_count++;  
    }    
  }
  else {
    if (right_wheel_pulse_count == encoder_minimum) {
     right_wheel_pulse_count = encoder_maximum;
    }
    else {
      right_wheel_pulse_count--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if(val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_pulse_count == encoder_maximum) {
      left_wheel_pulse_count = encoder_minimum;
    }
    else {
      left_wheel_pulse_count++;  
    }  
  }
  else {
    if (left_wheel_pulse_count == encoder_minimum) {
      left_wheel_pulse_count = encoder_maximum;
    }
    else {
      left_wheel_pulse_count--;  
    }   
  }
}

void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
 
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
   
}
 
void loop() {
 
   
    Serial.print("Right Ticks: ");
    Serial.println(right_wheel_pulse_count); 
   Serial.print("Left Ticks: ");
    Serial.println(left_wheel_pulse_count); 
 
}

/*
// Increment the number of pulses by 1
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

// Increment the number of pulses by 1
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}*/
