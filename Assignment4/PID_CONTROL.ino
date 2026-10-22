// ============================================================
// DUAL PID FINAL — BOTH MOTORS SAME SPEED, STABLE
// ============================================================

int ENA = 5;
int IN1 = 7;
int IN2 = 8;

int ENB = 6;
int IN3 = 9;
int IN4 = 10;

const byte LEFT_ENCODER_PIN  = 3;
const byte RIGHT_ENCODER_PIN = 2;

// YOUR ORIGINAL CPR VALUES
float LEFT_CPR  = 41.95;
float RIGHT_CPR = 40.27;

float LEFT_MAX_RPM  = 450.0;
float RIGHT_MAX_RPM = 390.0;

// TARGET — both motors same RPM
double SetP1 = 200.0;
double SetP2 = 200.0;  // 87% of left (390/450 ratio)

// ============================================================
// LEFT MOTOR
// ============================================================

unsigned long L_prevTime = 0;
double        L_period   = 100.0;
volatile long L_counter  = 0;
double        L_RPM      = 0;
double        L_RPMf     = 0;
double        L_alpha    = 0.15;

double L_kp = 0.3;
double L_ki = 0.05;
double L_kd = 0.001;
double L_Ts = 0.1;

double L_error    = 0;
double L_errPrev1 = 0;
double L_errPrev2 = 0;
double L_cv       = 0;
double L_cvPrev   = 0;
double L_pwm      = 0;
double L_integral = 0;

// ============================================================
// RIGHT MOTOR
// ============================================================

unsigned long R_prevTime = 0;
double        R_period   = 100.0;
volatile long R_counter  = 0;
double        R_RPM      = 0;
double        R_RPMf     = 0;
double        R_alpha    = 0.15;

double R_kp = 0.3;
double R_ki = 0.05;
double R_kd = 0.001;
double R_Ts = 0.1;

double R_error    = 0;
double R_errPrev1 = 0;
double R_errPrev2 = 0;
double R_cv       = 0;
double R_cvPrev   = 0;
double R_pwm      = 0;
double R_integral = 0;

// ============================================================
// ENCODER ISRs
// ============================================================

volatile unsigned long L_lastDebounce = 0;
volatile unsigned long R_lastDebounce = 0;

void encoderLeft() {
  unsigned long t = micros();
  if (t - L_lastDebounce >= 1000) {
    L_lastDebounce = t;
    L_counter++;
  }
}

void encoderRight() {
  unsigned long t = micros();
  if (t - R_lastDebounce >= 1000) {
    R_lastDebounce = t;
    R_counter++;
  }
}

// ============================================================
// LEFT PID
// ============================================================

double runLeftPID() {
  L_error = SetP1 - L_RPMf;

  // anti-windup — only integrate when not saturated
  if (L_cvPrev > 0 && L_cvPrev < 255) {
    L_integral += L_error * L_Ts;
  }

  L_integral = constrain(L_integral, -150, 150);

  double derivative = (L_error - L_errPrev1) / L_Ts;

  L_cv = L_kp * L_error
       + L_ki * L_integral
       + L_kd * derivative;

  L_cv = L_cvPrev + L_cv;
  L_cv = constrain(L_cv, 0, 255);

  L_cvPrev   = L_cv;
  L_errPrev2 = L_errPrev1;
  L_errPrev1 = L_error;

  return L_cv;
}

// ============================================================
// RIGHT PID
// ============================================================

double runRightPID() {
  R_error = SetP2 - R_RPMf;

  // anti-windup — only integrate when not saturated
  if (R_cvPrev > 0 && R_cvPrev < 255) {
    R_integral += R_error * R_Ts;
  }

  R_integral = constrain(R_integral, -150, 150);

  double derivative = (R_error - R_errPrev1) / R_Ts;

  R_cv = R_kp * R_error
       + R_ki * R_integral
       + R_kd * derivative;

  R_cv = R_cvPrev + R_cv;
  R_cv = constrain(R_cv, 0, 255);

  R_cvPrev   = R_cv;
  R_errPrev2 = R_errPrev1;
  R_errPrev1 = R_error;

  return R_cv;
}

// ============================================================
// MOTOR DRIVE
// ============================================================

void driveMotors(int pwmL, int pwmR) {
  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  pinMode(LEFT_ENCODER_PIN,  INPUT_PULLUP);
  pinMode(RIGHT_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCODER_PIN),  encoderLeft,  RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCODER_PIN), encoderRight, RISING);

  L_prevTime = millis();
  R_prevTime = millis();

  Serial.begin(115200);
  Serial.println("== DUAL PID READY ==");
  Serial.print("L_Target="); Serial.print(SetP1);
  Serial.print(" R_Target="); Serial.println(SetP2);
}

// ============================================================
// LOOP
// ============================================================

void loop() {

  unsigned long now = millis();

  // ---- LEFT PID ----
  if (now - L_prevTime >= L_period) {

    noInterrupts();
    long L = L_counter;
    L_counter = 0;
    interrupts();

    float dt   = (now - L_prevTime) / 1000.0;
    L_prevTime = now;

    L_RPM  = (L / LEFT_CPR) / dt * 60.0;
    L_RPM  = constrain(L_RPM, 0, LEFT_MAX_RPM);
    L_RPMf = L_alpha * L_RPM + (1.0 - L_alpha) * L_RPMf;
    L_pwm  = runLeftPID();
  }

  // ---- RIGHT PID ----
  if (now - R_prevTime >= R_period) {

    noInterrupts();
    long R = R_counter;
    R_counter = 0;
    interrupts();

    float dt   = (now - R_prevTime) / 1000.0;
    R_prevTime = now;

    R_RPM  = (R / RIGHT_CPR) / dt * 60.0;
    R_RPM  = constrain(R_RPM, 0, RIGHT_MAX_RPM);
    R_RPMf = R_alpha * R_RPM + (1.0 - R_alpha) * R_RPMf;
    R_pwm  = runRightPID();
  }

  // ---- DRIVE ----
  driveMotors(int(L_pwm), int(R_pwm));

  // ---- SERIAL every 250ms ----
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 250) {
    lastPrint = now;
    Serial.print("L_RPM=");    Serial.print(L_RPMf, 1);
    Serial.print(" R_RPM=");   Serial.print(R_RPMf, 1);
    Serial.print(" | L_PWM="); Serial.print(L_pwm,  1);
    Serial.print(" R_PWM=");   Serial.println(R_pwm, 1);
  }
}