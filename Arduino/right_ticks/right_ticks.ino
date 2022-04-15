// Encoder output to Arduino Interrupt pin. Tracks the pulse count.
#define ENC_IN_RIGHT_A 2
#define ENC_IN_LEFT_A 3
 
// Keep track of the number of right wheel pulses
volatile long right_wheel_pulse_count = 0;
volatile long left_wheel_pulse_count = 0;
 
void setup() {
 
  // Open the serial port at 9600 bps
  Serial.begin(9600); 
 
  // Set pin states of the encoder
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
 
  // Every time the pin goes high, this is a pulse
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_pulse, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_pulse, RISING);
   
}
 
void loop() {
 
   
    Serial.print("Right Ticks: ");
    Serial.println(right_wheel_pulse_count); 
   // Serial.print("Left Ticks: ");
   // Serial.println(left_wheel_pulse_count); 
    delay(10);
}
 
// Increment the number of pulses by 1
void right_wheel_pulse() {
  right_wheel_pulse_count++;
}

// Increment the number of pulses by 1
void left_wheel_pulse() {
  left_wheel_pulse_count++;
}
