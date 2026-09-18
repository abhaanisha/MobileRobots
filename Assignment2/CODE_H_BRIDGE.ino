// ==========================================================
//        L298N DUAL MOTOR SPEED + DIRECTION CONTROL
//        Arduino Mega / Mega ADK
// ==========================================================
//
// LEFT MOTOR:
//   ENA -> D5
//   IN1 -> D7
//   IN2 -> D8
//
// RIGHT MOTOR:
//   ENB -> D6
//   IN3 -> D9
//   IN4 -> D10
//
// Encoder pins are reserved:
//   Left encoder  DO -> D2
//   Right encoder DO -> D3
//
// Speed range:
//   0   = STOP
//   255 = MAXIMUM
//
// Direction:
//   FORWARD
//   BACKWARD
//   STOP
// ==========================================================


// ---------------- PIN DEFINITIONS ----------------

// Left motor
const int ENA = 5;
const int IN1 = 7;
const int IN2 = 8;

// Right motor
const int ENB = 6;
const int IN3 = 9;
const int IN4 = 10;


// ---------------- MOTOR SETTINGS ----------------

// Speed: 0 to 255
int leftSpeed  = 150;
int rightSpeed = 150;


// ==========================================================
// MOTOR A (LEFT) CONTROL
// ==========================================================

void leftMotor(int speed, int direction)
{
  speed = constrain(speed, 0, 255);

  if (direction == 1)
  {
    // FORWARD
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if (direction == -1)
  {
    // BACKWARD
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    // STOP
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }

  analogWrite(ENA, speed);
}


// ==========================================================
// MOTOR B (RIGHT) CONTROL
// ==========================================================

void rightMotor(int speed, int direction)
{
  speed = constrain(speed, 0, 255);

  if (direction == 1)
  {
    // FORWARD
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if (direction == -1)
  {
    // BACKWARD
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    // STOP
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }

  analogWrite(ENB, speed);
}


// ==========================================================
// SETUP
// ==========================================================

void setup()
{
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(115200);

  // Start with motors stopped
  leftMotor(0, 0);
  rightMotor(0, 0);

  Serial.println("=================================");
  Serial.println("   MOTOR SPEED + DIRECTION TEST");
  Serial.println("=================================");
}


// ==========================================================
// LOOP
// ==========================================================

void loop()
{
  // --------------------------------------------------------
  // FORWARD
  // --------------------------------------------------------

  Serial.println("FORWARD");

  leftMotor(leftSpeed, 1);
  rightMotor(rightSpeed, 1);

  delay(3000);


  // --------------------------------------------------------
  // STOP
  // --------------------------------------------------------

  Serial.println("STOP");

  leftMotor(0, 0);
  rightMotor(0, 0);

  delay(2000);


  // --------------------------------------------------------
  // BACKWARD
  // --------------------------------------------------------

  Serial.println("BACKWARD");

  leftMotor(leftSpeed, -1);
  rightMotor(rightSpeed, -1);

  delay(3000);


  // --------------------------------------------------------
  // STOP
  // --------------------------------------------------------

  Serial.println("STOP");

  leftMotor(0, 0);
  rightMotor(0, 0);

  delay(2000);
}