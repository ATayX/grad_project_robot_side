#include <Servo.h>
// DECLARATIONS
// DC motor Declarations
#define MOTOR_A_IN1_PIN 46  // motor1 forward Pin
#define MOTOR_A_IN2_PIN 48 // motor1 reverse Pin
#define MOTOR_B_IN1_PIN 50 // motor2 forward Pin
#define MOTOR_B_IN2_PIN 52 // motor2 reverse Pin
byte mot_A_val1 = 0, mot_A_val2 = 0, mot_B_val1 = 0, mot_B_val2 = 0;
#define PWM_PIN_A 2
#define PWM_PIN_B 3
bool mot_A_break = false, mot_B_break = false;
// Servo declarations
#define SERVO1_PIN 4  // servo signal pin
#define SERVO2_PIN 5  // servo signal pin
Servo SERVO1;
Servo SERVO2;
byte servo1_val = 84, servo2_val = 0; // servo2 midpoint = 99
// Ultrasonic sensor Declarations
#define TRIG_PIN 34
#define ECHO_PIN 35
long duration;
int distance; 
// Photosensor wheel speed declarations and break system declarations
#define WHEEL_A_PIN_D 40
#define WHEEL_B_PIN_D 42
#define WHEEL_A_PIN_A A0
#define WHEEL_B_PIN_A A1
int threshold = 110;
int wheel_A, wheel_B, prev_wheel_A = 0, prev_wheel_B = 0;
long wheel_duration = 100;
unsigned long timestamp = 0;
bool prev_photo_A = false, prev_photo_B = false;
// Break System Declarations
//byte prev_mot_A_val1 = 0, prev_mot_A_val2 = 0, prev_mot_B_val1 = 0, prev_mot_B_val2 = 0;
//unsigned long mot_A_timer = 0, mot_B_timer = 0;
//long  break_action_time = 100;
// Transmission Declarations
bool wait_report_response = true;
char signal = '~';// handshake signal
char write_msg_prefix = 'W';
char end_msg_prefix = 'E';
char report_msg_prefix = 'R';
int receive_delay = 5; // delay for receiving all message bytes 
bool enable_comms = false;

void setup() // CONFIGURED FOR: Transmission, Motors, Servos
{
  // open serial port
  Serial.begin(115200);
  //Serial.println("beginning Serial comms"); // debug line
  Serial1.begin(115200);
  //Serial.println("beginning Serial1 comms"); // debug line
  Serial1.print(signal); // request connection signal
  //Serial.print("Sent wifi handshake\n");
  // DC motor PINs Setup
  pinMode(MOTOR_A_IN1_PIN, OUTPUT);
  pinMode(MOTOR_A_IN2_PIN, OUTPUT);
  pinMode(MOTOR_B_IN1_PIN, OUTPUT);
  pinMode(MOTOR_B_IN2_PIN, OUTPUT);
  pinMode(PWM_PIN_A, OUTPUT);
  pinMode(PWM_PIN_B, OUTPUT);
  write_motor_vals();
  // Servo PINs Setup
  SERVO1.attach(SERVO1_PIN);
  SERVO2.attach(SERVO2_PIN);
  write_servo_vals();
  // ultrasonic sensor declarations
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  // photosensors wheel speed declarations
  pinMode(WHEEL_A_PIN_A, INPUT);
  pinMode(WHEEL_B_PIN_A, INPUT);
  pinMode(WHEEL_A_PIN_D, OUTPUT);
  pinMode(WHEEL_B_PIN_D, OUTPUT);
  digitalWrite(WHEEL_A_PIN_D, HIGH);  
  digitalWrite(WHEEL_B_PIN_D, HIGH);
}

void loop() 
{
  /////////////WHEEL SPEED READ////////////////
  wheel_read();
  /////////////////////////////////////////////
  // check for response message
  read_transmission();
  // write a report message
  report_message_write_transmission();

  // breaks_system(); // identify need for breaks and takes action
  
}

void read_transmission()
{
  // note: byte can transmit 0~255
  // suggested prefixes: <Th> </Th>  
  // transmitted values' ranges: Th1 0~180, Th2 0~180, motor1_right_val -255~255, motor2_left_val -255~255, distance 2~400cm,
  // Read Serial1
  if(Serial1.available()>= 1)
  {
    char message_identifier = Serial1.read(); // read prefix flag
    Serial.println(message_identifier); // debug line
    Serial.println(enable_comms); // debug line
    if (enable_comms)
    {
      if (message_identifier == signal) 
      {
        wait_report_response = false; // handshake prefix flag
        Serial.println(message_identifier);
      }
      else if(message_identifier ==  write_msg_prefix) // Instruction prefix flag
      {
        delay(receive_delay); // wait for all the message bytes 
        // read motor value bytes
        mot_A_val1 = Serial1.read();
        mot_A_val2 = Serial1.read();
        mot_B_val1 = Serial1.read();
        mot_B_val2 = Serial1.read();
        write_motor_vals();
        // read servo value bytes
        servo1_val = Serial1.read();
        servo2_val = Serial1.read();
        write_servo_vals();
        // FUNCTION! if(Serial1.read() == end_msg_prefix) write_motor_vals(); // end message flag _ assure message transmitted correctly
      }
      else if (message_identifier == '-') // wifi disconnected, stop motors
      {
        // stop motors
        mot_A_val1 = 0;
        mot_A_val2 = 0;
        mot_B_val1 = 0;
        mot_B_val2 = 0;
        write_motor_vals();
        enable_comms = false;
        Serial.write("TCP Disconnected: Turning off motors");
      }
      else if (message_identifier == '+') 
      {
        Serial1.print(signal); // wifi connected, establish handshake  
        Serial.println("TCP connected");
      } 
    }
    else if(enable_comms == false) // comms are disabled in order to avoid noise at mcu node startup
    {
      Serial.println("recieving data on off comms");
      if(message_identifier == 'E')
      {
        if (Serial1.read() == 'C')
        { 
          enable_comms = true; // Serial.write("unlocked comms"); // debug line
          Serial.println("comms on");
        }
      }
      else if (message_identifier == signal) // incase arduino mega restarts while mcu node is still connected
      {
        enable_comms = true;
        wait_report_response = false; // start reporting
      }
    }
  }
}

void report_message_write_transmission() // reports DCmotor, servo values
{
  if(wait_report_response == false)
  {
    
    // measure distance
    measure_distance();  
    // read_wheel_spd();  
    // report message example byte array = B1:R B2:W B3:M1IN1 B4:M1IN2 B5:M2IN1 B6:M2IN2
    // write prefixes
    Serial1.write(report_msg_prefix); // "report" message flag
    Serial1.write(write_msg_prefix); // write message flag
    // write current motor values
    Serial1.write(mot_A_val1);
    Serial1.write(mot_A_val2);
    Serial1.write(mot_B_val1);
    Serial1.write(mot_B_val2);
    // write current servo values
    Serial1.write(servo1_val);
    Serial1.write(servo2_val);
    // write distance value
    if (distance <= 255)
    {
      Serial1.write((byte)distance);
      Serial1.write((byte)0);
    }
    else
    {
      Serial1.write((byte)255);
      Serial1.write((byte)(distance - 255));      
    }
    // write wheel speed values
    Serial1.write((byte)prev_wheel_A);
    Serial1.write((byte)prev_wheel_B);
    // write prefix
    Serial1.write(end_msg_prefix); // "end" message flag
    // Serial.print("sent report to wifi\n");
    wait_report_response = true;
  }
}

void write_motor_vals()
{
  // set motor direction NOTE: ALWAYS WRITE LOW THEN HIGH
  if(mot_A_val1 - mot_A_val2 > 0)
  {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, HIGH);
  } 
  else if (mot_A_val1 - mot_A_val2 < 0)
  {
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
    digitalWrite(MOTOR_A_IN1_PIN, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_A_IN1_PIN, LOW);
    digitalWrite(MOTOR_A_IN2_PIN, LOW);
  }
  if (mot_B_val1 - mot_B_val2 > 0)
  {
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, HIGH);
  }  
  else if (mot_B_val1 - mot_B_val2 < 0)
  {
    digitalWrite(MOTOR_B_IN2_PIN, LOW);
    digitalWrite(MOTOR_B_IN1_PIN, HIGH);
  }
  else
  {
    digitalWrite(MOTOR_B_IN1_PIN, LOW);
    digitalWrite(MOTOR_B_IN2_PIN, LOW);    
  }
  // set motor pwm value
  //Serial.println(abs(mot_A_val1 - mot_A_val2));
  //Serial.println(abs(mot_B_val1 - mot_B_val2));
  analogWrite(PWM_PIN_A, abs(mot_A_val1 - mot_A_val2)); //*1.3333336); // multiplied by rpm different value because of hardware problems
  analogWrite(PWM_PIN_B, abs(mot_B_val1 - mot_B_val2));
}

void write_servo_vals()
{
  SERVO1.write(servo1_val);
  SERVO2.write(servo2_val);
}

void measure_distance()
{
  // Clears the trigPin
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(ECHO_PIN, HIGH, 10000);
  // Calculating the distance
  distance = duration * 0.034 / 2;
  // Serial.println(distance);
}

void wheel_read()
{
  if (timestamp < millis()) // duration expired, start new read
  {
    // update wheel speed for 100ms readings
    prev_wheel_A = wheel_A;
    prev_wheel_B = wheel_B;  
    // reset wheel speed
    wheel_A = 0;
    wheel_B = 0;
    // speed goes from 0 - 1023
    // 44 -45 at 10ms is always white
    timestamp = millis() + wheel_duration;
    prev_photo_A = false;
    prev_photo_B = false;
    // initial state reads
    if (analogRead(WHEEL_A_PIN_A)> threshold) prev_photo_A = true;
    if (analogRead(WHEEL_B_PIN_A)> threshold) prev_photo_B = true;
  }
  else // else if (timestamp >= millis()) // duration not expired, contine the read
  {
    if (analogRead(WHEEL_A_PIN_A)> threshold && prev_photo_A == false) 
    {
      prev_photo_A = true;
      wheel_A++;
    }
    else  if (analogRead(WHEEL_A_PIN_A)< threshold && prev_photo_A == true)
    {
      prev_photo_A = false;
    }
    
    if (analogRead(WHEEL_B_PIN_A)> threshold && prev_photo_B == false) 
    {
      prev_photo_B = true;
      wheel_B++;
    }
    else if (analogRead(WHEEL_B_PIN_A)< threshold && prev_photo_B == true)
    {
      prev_photo_B = false;
    }
  }
}

// pc code detects changes and overrides break system, thus break system is best made on pc side
// BREAK SYSTEM canceled
/*
void breaks_system()
{
  // break wheel A
  if ((int)mot_A_val1 + (int)mot_A_val2 == 0 && (int)prev_mot_A_val1 + (int)prev_mot_A_val2 != 0 && mot_A_break == false) // detect stop after movement
  {
    // turn on breaks: identify previous movement direction and reverse it for breaks
    if ((int)prev_mot_A_val1 > 0) // prev_mot_A_val1 ranges from 0-255
    {
      mot_A_val1 = 0; // preventing HIGH on both outputs 
      mot_A_val2 = prev_mot_A_val1; // reverse movement
    }
    else if ((int)prev_mot_A_val2 > 0) // prev_mot_A_val1 ranges from 0-255
    {
      mot_A_val2 = 0; // preventing HIGH on both outputs
      mot_A_val1 = prev_mot_A_val2; // reverse movement
    }
    mot_A_break = true;
    mot_A_timer = millis() + 2000;
  }
  else if (mot_A_break && mot_A_timer < millis()) // shutdown breaks movement
  {
    mot_A_val1 = 0;
    mot_A_val2 = 0;
    mot_A_break = false;
  } 
  prev_mot_A_val1 = mot_A_val1;
  prev_mot_A_val2 = mot_A_val2;
    

  // break wheel B
  if ((int)mot_B_val1 + (int)mot_B_val2 == 0 && (int)prev_mot_B_val1 + (int)prev_mot_B_val2 != 0 && mot_B_break == false) // detect stop after movement
  {
    // turn on breaks: identify previous movement direction and reverse it for breaks
    if ((int)prev_mot_B_val1 > 0) // prev_mot_B_val1 ranges from 0-255
    {
      mot_B_val1 = 0; // preventing HIGH on both outputs
      mot_B_val2 = prev_mot_B_val1; // reverse movement
    }
    else if ((int)prev_mot_B_val2 > 0) // prev_mot_B_val1 ranges from 0-255
    {
      mot_B_val2 = 0; // preventing HIGH on both outputs
      mot_B_val1 = prev_mot_B_val2; // reverse movement
    }
    mot_B_break = true; // itr 1 see this
    mot_B_timer = millis() + 2000;
  }
  else if (mot_B_break && mot_B_timer < millis())// shutdown breaks movement
  {
    mot_B_val1 = 0;
    mot_B_val2 = 0;
    mot_B_break = false;
  }
  // if mot A stops || if mot B stop || if mot A reverse || if mot B reverse: write motor values
  if ((mot_A_val1 == 0 && mot_A_val2 == 0) || (mot_B_val1 == 0 && mot_B_val2 == 0) || mot_A_val2 == prev_mot_A_val1 || mot_A_val1 == prev_mot_A_val2 || mot_B_val2 == prev_mot_B_val1 || mot_B_val1 == prev_mot_B_val2) write_motor_vals();
  // continue monitoring mot_B_val changes
  prev_mot_B_val1 = mot_B_val1;
  prev_mot_B_val2 = mot_B_val2;
}
*/