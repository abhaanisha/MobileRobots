// ============================================================
// DIFFERENTIAL DRIVE ROBOT
// ENCODER ODOMETRY + FORWARD KINEMATICS
// Arduino Mega
//
// Experiment:
// 1. Move forward 100 cm
// 2. Turn right 90 degrees
// 3. Move forward 50 cm
//
// Output every 10 cm:
// X, Y, Theta, Vx, Vy
//
// FIXES APPLIED vs original:
//  1. Encoder ISRs now apply direction sign (leftDir/rightDir),
//     set by leftMotor()/rightMotor() -- previously both ISRs
//     only incremented, so turnRight90() saw omega ~= 0 and
//     the turn loop never terminated.
//  2. Position update uses midpoint heading (theta + omega*dt/2)
//     for vx/vy instead of the post-update theta -- reduces
//     heading-integration error, especially during turns.
//  3. moveForward() and turnRight90() now have a millis()-based
//     timeout that force-stops the robot and prints a warning,
//     instead of an unconditional while(true).
//  4. previousTime in updateOdometry() is explicitly initialized
//     in setup() (right after zeroing encoder counts) instead of
//     relying on a static variable defaulting to 0, which would
//     otherwise inflate the first dt by however long setup()'s
//     delay()s had already run.
//  5. Encoder ISRs now debounce with a minimum micros() interval
//     between accepted edges (ENCODER_DEBOUNCE_US). LM393 slot
//     sensors commonly chatter at the edges, especially at high/
//     unloaded RPM, and without this a single physical edge can
//     be counted 2-4x, producing wildly inflated and unstable
//     v/omega/theta readings (this is what caused the garbage
//     numbers seen when the robot was run wheels-off-ground).
//  6. updateOdometry() now only runs its kinematics/integration
//     block once at least ODOM_MIN_INTERVAL_MS has elapsed (it
//     still returns immediately otherwise, leaving counts to
//     keep accumulating in the background). At 20 counts/rev,
//     a single tick of lead by one wheel over the other swings
//     theta by ~4.3 degrees; calling the kinematics every loop
//     pass means deltaLeft/deltaRight are usually 0-2, so a
//     +/-1 tick mismatch is a 50-100% relative error on that
//     update. Widening the window so more ticks accumulate per
//     update shrinks that relative error and the resulting
//     theta jumps -- this does NOT fix a real per-wheel speed
//     mismatch (see the note in the chat), only the quantization
//     jitter from sampling too fast relative to encoder CPR.
// ============================================================


// ============================================================
// 1. MOTOR PINS
// ============================================================

// LEFT MOTOR
const byte ENA = 5;
const byte IN1 = 7;
const byte IN2 = 8;

// RIGHT MOTOR
const byte ENB = 6;
const byte IN3 = 9;
const byte IN4 = 10;


// ============================================================
// 2. ENCODER PINS
// ============================================================

const byte LEFT_ENCODER_PIN  = 2;
const byte RIGHT_ENCODER_PIN = 3;


// ============================================================
// 3. ROBOT PARAMETERS
// ============================================================

// 20 slots -> 20 rising-edge counts/revolution
const float COUNTS_PER_REV = 20.0;

// Wheel diameter
const float WHEEL_DIAMETER = 0.065;       // m

// Wheel radius
const float WHEEL_RADIUS = 0.0325;        // m

// Distance between wheel centres
const float WHEEL_BASE = 0.135;            // m


// ============================================================
// 4. MOTOR SPEED
// ============================================================

const int FORWARD_SPEED = 150;

const int TURN_SPEED = 130;


// ============================================================
// 5. ENCODER COUNTERS + DIRECTION
// ============================================================

volatile long leftCount = 0;
volatile long rightCount = 0;

// Commanded direction, set by leftMotor()/rightMotor().
// +1 = forward, -1 = backward, 0 = stopped.
// The ISR uses this to sign the count increment, since the
// LM393 setup here is single-channel and cannot sense direction
// on its own -- direction is INFERRED from what we commanded.
volatile int leftDir = 0;
volatile int rightDir = 0;

// FIX 5: software debounce for the encoder ISRs. LM393 comparator
// outputs commonly chatter at slot edges, which the ISR would
// otherwise count as multiple real ticks. An edge arriving sooner
// than ENCODER_DEBOUNCE_US after the last accepted edge is treated
// as bounce and ignored.
//
// 1500 us caps counted frequency at ~667 Hz/wheel, i.e. up to
// 2000 RPM at 20 counts/rev -- generous headroom above any
// plausible speed for this hardware, while rejecting bounce
// trains. Tune down only after confirming your motor's real
// free-run RPM is being clipped.
const unsigned long ENCODER_DEBOUNCE_US = 1500;

volatile unsigned long lastLeftEdgeMicros = 0;
volatile unsigned long lastRightEdgeMicros = 0;


// ============================================================
// 6. ODOMETRY VARIABLES
// ============================================================

float x = 0.0;
float y = 0.0;
float theta = 0.0;

// Robot linear velocity
float v = 0.0;

// Robot angular velocity
float omega = 0.0;

// World-frame velocities
float vx = 0.0;
float vy = 0.0;


// Previous encoder counts
long previousLeftCount = 0;
long previousRightCount = 0;

// Previous timestamp used by updateOdometry() for dt.
// Explicitly initialized in setup() -- see FIX 4 above.
unsigned long previousOdomTime = 0;

// FIX 6: minimum time between kinematics updates. updateOdometry()
// still returns immediately (without touching counts/integration)
// if called before this much time has passed since the last
// accepted update -- encoder counts keep accumulating via the ISRs
// in the meantime, so no ticks are lost, only the update cadence
// is slowed. 30 ms is a starting point; raise it if theta jumps
// are still too coarse, lower it if you need faster response
// (e.g. for the turn-stop check) once real speed mismatch (if any)
// is separately confirmed/fixed.
const unsigned long ODOM_MIN_INTERVAL_MS = 30;


// ============================================================
// 7. ENCODER INTERRUPTS
// ============================================================

void leftEncoderISR()
{
  // FIX 5: reject edges that arrive too soon after the last
  // accepted one -- almost certainly comparator bounce, not a
  // real new slot edge.
  unsigned long now = micros();
  if (now - lastLeftEdgeMicros < ENCODER_DEBOUNCE_US)
    return;
  lastLeftEdgeMicros = now;

  if (leftDir > 0)
    leftCount++;
  else if (leftDir < 0)
    leftCount--;
  // if leftDir == 0, ignore (shouldn't normally tick while stopped)
}


void rightEncoderISR()
{
  // FIX 5: same debounce as leftEncoderISR().
  unsigned long now = micros();
  if (now - lastRightEdgeMicros < ENCODER_DEBOUNCE_US)
    return;
  lastRightEdgeMicros = now;

  if (rightDir > 0)
    rightCount++;
  else if (rightDir < 0)
    rightCount--;
}


// ============================================================
// 8. MOTOR CONTROL
// ============================================================

void leftMotor(int speed, int direction)
{
  speed = constrain(speed, 0, 255);

  // FIX 1: record commanded direction for the ISR to use.
  leftDir = direction;

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


void rightMotor(int speed, int direction)
{
  speed = constrain(speed, 0, 255);

  // FIX 1: record commanded direction for the ISR to use.
  rightDir = direction;

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


void stopRobot()
{
  leftMotor(0, 0);
  rightMotor(0, 0);
}


// ============================================================
// 9. UPDATE ODOMETRY
// ============================================================
//
// This implements:
//
// omega_L = 2*pi*DeltaNL / (N*dt)
// omega_R = 2*pi*DeltaNR / (N*dt)
//
// vL = r*omega_L
// vR = r*omega_R
//
// v     = (vR + vL)/2
// omega = (vR - vL)/L
//
// theta_mid = theta + omega*dt/2      <-- FIX 2 (midpoint)
// vx = v*cos(theta_mid)
// vy = v*sin(theta_mid)
//
// x = x + vx*dt
// y = y + vy*dt
// theta = theta + omega*dt            <-- updated last
//
// ============================================================

void updateOdometry()
{
  unsigned long currentTime = millis();

  // FIX 6: gate the kinematics update to a minimum interval so
  // enough ticks accumulate per update to keep +/-1 tick timing
  // jitter from dominating the theta calculation. Counts/ISRs are
  // untouched here -- they keep incrementing regardless of when
  // this function is called; we're only deferring when we *read*
  // them and run the integration.
  if (currentTime - previousOdomTime < ODOM_MIN_INTERVAL_MS)
    return;

  float dt = (currentTime - previousOdomTime) / 1000.0;

  if (dt <= 0.0)
    return;

  previousOdomTime = currentTime;


  // ----------------------------------------------------------
  // Copy encoder counts safely
  // ----------------------------------------------------------

  noInterrupts();

  long currentLeftCount = leftCount;
  long currentRightCount = rightCount;

  interrupts();


  // ----------------------------------------------------------
  // Encoder count change
  // ----------------------------------------------------------

  long deltaLeft =
    currentLeftCount - previousLeftCount;

  long deltaRight =
    currentRightCount - previousRightCount;


  previousLeftCount = currentLeftCount;
  previousRightCount = currentRightCount;


  // ----------------------------------------------------------
  // Convert encoder counts to wheel angular displacement
  // ----------------------------------------------------------

  float deltaPhiLeft =
    (2.0 * PI * deltaLeft) / COUNTS_PER_REV;

  float deltaPhiRight =
    (2.0 * PI * deltaRight) / COUNTS_PER_REV;


  // ----------------------------------------------------------
  // Wheel displacement
  // ----------------------------------------------------------

  float deltaSLeft =
    WHEEL_RADIUS * deltaPhiLeft;

  float deltaSRight =
    WHEEL_RADIUS * deltaPhiRight;


  // ----------------------------------------------------------
  // Wheel linear velocity
  // ----------------------------------------------------------

  float vLeft =
    deltaSLeft / dt;

  float vRight =
    deltaSRight / dt;


  // ----------------------------------------------------------
  // Differential-drive forward kinematics
  // ----------------------------------------------------------

  v =
    (vRight + vLeft) / 2.0;

  omega =
    (vRight - vLeft) / WHEEL_BASE;


  // ----------------------------------------------------------
  // FIX 2: World-frame velocity using MIDPOINT heading,
  // computed from the pre-update theta, before theta itself
  // is advanced. This approximates the true arc travelled
  // during this interval far better than using either the
  // stale pre-update theta or the fully-advanced post-update
  // theta.
  // ----------------------------------------------------------

  float thetaMid =
    theta + 0.5 * omega * dt;

  vx =
    v * cos(thetaMid);

  vy =
    v * sin(thetaMid);


  // ----------------------------------------------------------
  // Update position (using midpoint-derived vx, vy)
  // ----------------------------------------------------------

  x =
    x + vx * dt;

  y =
    y + vy * dt;


  // ----------------------------------------------------------
  // Update orientation LAST, now that thetaMid has already
  // been derived from the pre-update theta.
  // ----------------------------------------------------------

  theta =
    theta + omega * dt;
}


// ============================================================
// 10. PRINT ROBOT STATE
// ============================================================

void printRobotState()
{
  Serial.print("X = ");
  Serial.print(x, 3);

  Serial.print(" m | Y = ");
  Serial.print(y, 3);

  Serial.print(" m | Theta = ");
  Serial.print(theta * 180.0 / PI, 2);

  Serial.print(" deg | Vx = ");
  Serial.print(vx, 3);

  Serial.print(" m/s | Vy = ");
  Serial.print(vy, 3);

  Serial.print(" m/s | v = ");
  Serial.print(v, 3);

  Serial.print(" m/s | omega = ");
  Serial.print(omega, 3);

  Serial.println(" rad/s");
}


// ============================================================
// 11. RESET ODOMETRY TIMING
// ============================================================
//
// Call this immediately before a motion stage if you want that
// stage's first updateOdometry() call to see a clean dt and
// zero encoder deltas, rather than carrying over counts/timing
// from whatever happened during the preceding delay().
// ============================================================

void resetOdometryTiming()
{
  noInterrupts();
  previousLeftCount = leftCount;
  previousRightCount = rightCount;
  interrupts();

  previousOdomTime = millis();
}


// ============================================================
// 12. MOVE FORWARD A SPECIFIC DISTANCE
// ============================================================

// FIX 3: safety timeout so a stalled/miscounting encoder can't
// leave the robot driving forever. Generous margin over the
// expected time for FORWARD_SPEED to cover targetDistance.
const unsigned long FORWARD_TIMEOUT_MS = 15000UL;

void moveForward(float targetDistance)
{
  float startX = x;
  float startY = y;

  float startDistance = 0.0;

  bool firstPrint = true;

  unsigned long motionStart = millis();
  bool timedOut = false;


  Serial.println();
  Serial.println("========== FORWARD MOTION ==========");

  resetOdometryTiming();

  leftMotor(FORWARD_SPEED, 1);
  rightMotor(FORWARD_SPEED, 1);


  while (true)
  {
    updateOdometry();


    // Distance travelled from starting point
    float dx = x - startX;
    float dy = y - startY;

    float travelled =
      sqrt(dx * dx + dy * dy);


    // --------------------------------------------------------
    // Print every 10 cm
    // --------------------------------------------------------

    if (firstPrint || travelled >= startDistance + 0.10)
    {
      printRobotState();

      startDistance =
        floor(travelled / 0.10) * 0.10;

      firstPrint = false;
    }


    // --------------------------------------------------------
    // Stop at target distance
    // --------------------------------------------------------

    if (travelled >= targetDistance)
    {
      break;
    }


    // --------------------------------------------------------
    // FIX 3: safety timeout
    // --------------------------------------------------------

    if (millis() - motionStart > FORWARD_TIMEOUT_MS)
    {
      timedOut = true;
      break;
    }
  }


  stopRobot();

  delay(500);

  printRobotState();

  if (timedOut)
  {
    Serial.println("!!!!!!!!!! FORWARD TIMEOUT - CHECK ENCODERS/MOTORS !!!!!!!!!!");
  }

  Serial.println("========== FORWARD COMPLETE ==========");
}


// ============================================================
// 13. TURN RIGHT 90 DEGREES
// ============================================================
//
// For a right turn:
//
// Left wheel  -> FORWARD
// Right wheel -> BACKWARD
//
// Desired:
//
// Delta theta = -90 degrees
//
// ============================================================

// FIX 3: safety timeout for the turn as well.
const unsigned long TURN_TIMEOUT_MS = 10000UL;

void turnRight90()
{
  float targetAngle =
    -PI / 2.0;


  float initialTheta =
    theta;

  unsigned long motionStart = millis();
  bool timedOut = false;


  Serial.println();
  Serial.println("========== RIGHT TURN 90 DEG ==========");

  resetOdometryTiming();


  // ----------------------------------------------------------
  // Start turning
  // ----------------------------------------------------------

  leftMotor(TURN_SPEED, 1);
  rightMotor(TURN_SPEED, -1);


  while (true)
  {
    updateOdometry();


    float angleTurned =
      theta - initialTheta;


    // Right turn is negative
    if (angleTurned <= targetAngle)
    {
      break;
    }


    // --------------------------------------------------------
    // FIX 3: safety timeout -- with the direction-sign fix in
    // place this should no longer be needed to escape an
    // infinite loop, but it's kept as insurance against a
    // stalled wheel, disconnected encoder, etc.
    // --------------------------------------------------------

    if (millis() - motionStart > TURN_TIMEOUT_MS)
    {
      timedOut = true;
      break;
    }
  }


  stopRobot();

  delay(500);


  printRobotState();

  if (timedOut)
  {
    Serial.println("!!!!!!!!!! TURN TIMEOUT - CHECK ENCODERS/MOTORS !!!!!!!!!!");
  }

  Serial.println("========== TURN COMPLETE ==========");
}


// ============================================================
// 14. SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);


  // ----------------------------------------------------------
  // Motor pins
  // ----------------------------------------------------------

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(ENB, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);


  // ----------------------------------------------------------
  // Encoder pins
  // ----------------------------------------------------------

  pinMode(LEFT_ENCODER_PIN, INPUT);
  pinMode(RIGHT_ENCODER_PIN, INPUT);


  // ----------------------------------------------------------
  // Encoder interrupts
  // ----------------------------------------------------------

  attachInterrupt(
    digitalPinToInterrupt(LEFT_ENCODER_PIN),
    leftEncoderISR,
    RISING
  );

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_ENCODER_PIN),
    rightEncoderISR,
    RISING
  );


  // ----------------------------------------------------------
  // Stop motors initially
  // ----------------------------------------------------------

  stopRobot();


  delay(1000);


  // ----------------------------------------------------------
  // Initial odometry
  // ----------------------------------------------------------

  x = 0.0;
  y = 0.0;
  theta = 0.0;


  previousLeftCount = leftCount;
  previousRightCount = rightCount;

  // FIX 4: explicitly set the odometry clock reference here,
  // right where counts are zeroed, instead of letting it
  // default to 0 and inflating the first dt by everything
  // that happened since power-on (including the delay()s
  // below and in setup() before the first motion stage).
  previousOdomTime = millis();


  Serial.println();
  Serial.println("==============================================");
  Serial.println(" DIFFERENTIAL DRIVE ODOMETRY EXPERIMENT");
  Serial.println("==============================================");

  Serial.println("Wheel diameter = 65 mm");
  Serial.println("Wheel base      = 135 mm");
  Serial.println("Encoder         = 20 counts/revolution");
  Serial.println("Sampling        = continuous timing");

  Serial.println();
  Serial.println("Initial State:");

  printRobotState();


  delay(2000);


  // ========================================================
  // EXPERIMENT
  // ========================================================


  // --------------------------------------------------------
  // 1. Move forward 1 metre
  // --------------------------------------------------------

  moveForward(1.0);


  delay(1000);


  // --------------------------------------------------------
  // 2. Turn right 90 degrees
  // --------------------------------------------------------

  turnRight90();


  delay(1000);


  // --------------------------------------------------------
  // 3. Move forward 50 cm
  // --------------------------------------------------------

  moveForward(0.50);


  delay(1000);


  // --------------------------------------------------------
  // Final state
  // --------------------------------------------------------

  Serial.println();
  Serial.println("==============================================");
  Serial.println("              FINAL STATE");
  Serial.println("==============================================");

  printRobotState();

  Serial.println("==============================================");
}


// ============================================================
// 15. LOOP
// ============================================================

void loop()
{
  // Experiment is performed once in setup().
}