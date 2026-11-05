/*
 * PS4 Drive (ESP32 self-balancing robot) - D-pad, pulse version

 * WHAT IT DOES:
    The PID balancing sketch plus Bluetooth driving from a PS4 controller. The
    D-pad never commands the wheels directly. Each press "kicks" the balance
    loop and lets it catch itself:
      - D-pad Up / Down    -> a short lean pulse forward / back (PULSE_MS), repeated every
                              PULSE_MS + PULSE_GAP_MS while held. Pulse height = maxLeanFwd / maxLeanBack.
      - D-pad Left / Right -> a short differential wheel pulse (TURN_PULSE_MS), repeated while held
                              (set TURN_PULSED 0 for a continuous held turn instead).
                              Balance always has priority; steering keeps TURN_MIN_ROOM PWM of authority.

    Uses the saved MPU6050 calibration (MpuCal.h) and the motor speed match (RIGHT_REF_PWM, LEFT_MATCH_PWM).

 * HOW TO USE:
    1. Run the MPU calibration tool (saves to flash). Make sure MpuCal.h and Ps4Input.h are in
      include/.
    2. Upload. Keep the robot still until the "ready" beep, then hold it upright. It engages after
       1 s of steady hold.
    3. Pair the controller (hold Share + PS until the lightbar double-flashes). Blue lightbar =
       connected, waiting; green = balancing.
    Serial (115200, ENTER to send):
       pVALUE, iVALUE, dVALUE, tVALUE - set Kp / Ki / Kd / balance trim (deg) e.g. p20
       fVALUE - lean pulse height FORWARD (deg)
       bVALUE - lean pulse height BACKWARD (deg)
       wVALUE - steering pulse height (PWM)
       s - save all of the above to flash (shares the "pid" values with the balancing sketch)
       v - telemetry on/off
       ? - print current values

 * CONTROLS:
    1. D-pad Up / Down - kick forward / back
    2. D-pad Left / Right - kick left / right
    3. L1 (hold) - gentle mode: half the lean and half the steering
    4. Circle - stop: motors off. Hold the robot upright 1 s to re-engage.
    5. Controller lost or out of range: commands fade to 0, the robot keeps balancing in place.

 * BEEPS:
    2 short - startup (before anything is loaded)
    1 medium - saved calibration loaded (startup)
    1 short - gyro zeroed, ready: hold the robot upright
    2 long - gyro zero rejected (robot moved), saved bias used
    3 quick - balancing engaged
    1 long - fell over, motors off
    1 medium - Circle pressed, motors off
    2 quick - settings saved
    1 tiny - PS4 connected
    2 tiny - PS4 lost
    3 long, repeating - saved MPU calibration missing or invalid (halted)
    5 quick, repeating - MPU6050 not found (halted)

 * NOTE:
    There is no wheel speed loop, so a held lean would be a held ACCELERATION. Pulsing the lean avoids that: the robot tips, steps to catch itself, and the balance loop settles it. Tune PULSE_MS for step size and PULSE_GAP_MS for how long it gets to settle between steps. Keep the lean limits small. Positive angle = leaning forward. Loop rate and filter match the calibration tool (250 Hz, 0.98). Uses Bluepad32 through Ps4Input.h.
*/

#include <Arduino.h>
#include <Preferences.h>
#include "MpuCal.h"
#include "Ps4Input.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define CORE3 1
#else
  #define CORE3 0
#endif

// Settings
// Motor A = LEFT, Motor B = RIGHT
#define PWMA 25
#define AIN1 26
#define AIN2 27
#define PWMB 14
#define BIN1 4
#define BIN2 13
#define STBY 33
#define BUZZER_PIN 23
#define BUZZER_PASSIVE 1  // 0 for an active buzzer
#define BUZZER_FREQ 2500
#define PWM_FREQ 1000
#define LEFT_DIR 1  // -1 if that wheel drives the wrong way
#define RIGHT_DIR 1
#define RIGHT_REF_PWM 200
#define LEFT_MATCH_PWM 140
#define SAMPLE_HZ 250
#define GYRO_ZERO_AT_BOOT  1
#define GYRO_ZERO_MAX_DELTA_DPS 2.0f // reject refresh if it moved more than this
#define UPRIGHT_WINDOW 5.0f // deg from balance point to start engaging
#define UPRIGHT_HOLD_MS 1000
#define FALL_ANGLE 35.0f  // deg from balance point to cut motors
#define I_MAX 80.0f

// Remote settings
#define STEER_DIR 1 // -1 if the robot turns the wrong way
#define GENTLE_SCALE 0.5f // L1 held

// Lean pulses (forward / back)
#define PULSE_MS 150   // length of each lean pulse
#define PULSE_GAP_MS 200  // rest between pulses while a button is held

// Steering pulses (left / right)
#define TURN_PULSED 0    // 1 = short differential pulses, 0 = continuous held turn
#define TURN_PULSE_MS 100 // length of each steering pulse
#define TURN_GAP_MS 150 // rest between steering pulses while held
#define TURN_MIN_ROOM 60.0f // PWM of steering always allowed, even when balance saturates

// Permissible lean (deg from the balance point) and steering limits
#define MAX_LEAN_FWD_DEFAULT 3.0f
#define MAX_LEAN_BACK_DEFAULT 3.0f
#define LEAN_HARD_CAP 10.0f // serial can never raise a lean limit above this
#define MAX_TURN_DEFAULT 100.0f // PWM, same scale as the balance output
#define TURN_HARD_CAP 120.0f
#define LEAN_SLEW_UP_DPS 40.0f   // deg/s, lean command growing away from 0 (fast enough for a pulse)
#define LEAN_SLEW_DOWN_DPS 60.0f  // deg/s, lean command returning toward 0 or reversing
#if TURN_PULSED
  #define TURN_SLEW_PER_S 2000.0f  // PWM/s, fast so a short pulse reaches its height
#else
  #define TURN_SLEW_PER_S 400.0f  // PWM/s
#endif
#define SAT_START 0.80f // lean allowance shrinks once |output| passes this fraction of 255
#define SAT_TAU 0.15f // s, smoothing of the output magnitude used for that

static const float ALPHA = 0.98f;
static const float LEFT_SCALE = (float)LEFT_MATCH_PWM / RIGHT_REF_PWM;
static const uint32_t LOOP_US = 1000000UL / SAMPLE_HZ;

// PID starting values, tune over serial
static float kp = 20.0f, ki = 0.0f, kd = 0.8f, trim = 0.0f;
static float maxLeanFwd = MAX_LEAN_FWD_DEFAULT, maxLeanBack = MAX_LEAN_BACK_DEFAULT;
static float maxTurn = MAX_TURN_DEFAULT;

#define LEFT_CH 0
#define RIGHT_CH 1
#define BUZ_CH 2

Adafruit_MPU6050 mpu;
MpuCalibration cal;

static float angle = 0.0f, iTerm = 0.0f, lastOut = 0.0f;
static float leanCmd = 0.0f, turnCmd = 0.0f, uMag = 0.0f;
static bool engaged = false, holding = false, telemetry = false;
static uint32_t holdStart = 0, lastUs = 0, lastTel = 0;

static float clampf(float v, float lo, float hi)
{
  return v < lo ? lo : (v > hi ? hi : v);
}

// PWM helpers
static void pwmAttach(int pin, int ch, int freq)
{
#if CORE3
  ledcAttach(pin, freq, 8);
#else
  ledcSetup(ch, freq, 8);
  ledcAttachPin(pin, ch);
#endif
}

static void pwmWrite(int pin, int ch, int duty)
{
#if CORE3
  ledcWrite(pin, duty);
#else
  ledcWrite(ch, duty);
#endif
}

// Buzzer (non-blocking)
static int bzLeft = 0, bzOn = 0, bzOff = 0;
static bool bzState = false;
static uint32_t bzT = 0;

static void buzz(bool on)
{
#if BUZZER_PASSIVE
  pwmWrite(BUZZER_PIN, BUZ_CH, on ? 128 : 0);
#else
  digitalWrite(BUZZER_PIN, on);
#endif
}

static void buzzerInit()
{
#if BUZZER_PASSIVE
  pwmAttach(BUZZER_PIN, BUZ_CH, BUZZER_FREQ);
#else
  pinMode(BUZZER_PIN, OUTPUT);
#endif
  buzz(false);
}

static void beep(int n, int onMs = 80, int offMs = 80)
{
  bzLeft = n;
  bzOn = onMs;
  bzOff = offMs;
  bzState = true;
  bzT = millis();
  buzz(true);
}

static void beepUpdate()
{
  if (bzLeft <= 0) return;

  uint32_t now = millis();
  if (bzState && now - bzT >= (uint32_t)bzOn)
  {
    buzz(false);
    bzState = false;
    bzT = now;
    bzLeft--;
  }
  else if (!bzState && bzLeft > 0 && now - bzT >= (uint32_t)bzOff)
  {
    buzz(true);
    bzState = true;
    bzT = now;
  }
}

static void beepWait()
{
  while (bzLeft > 0)
  {
    beepUpdate();
    delay(1);
  }
}

// Motors
static void setMotor(int pwmPin, int ch, int in1, int in2, int v)
{
  digitalWrite(in1, v > 0);
  digitalWrite(in2, v < 0);
  pwmWrite(pwmPin, ch, constrain(abs(v), 0, 255));
}

static void motorsStop()
{
  setMotor(PWMA, LEFT_CH,  AIN1, AIN2, 0);
  setMotor(PWMB, RIGHT_CH, BIN1, BIN2, 0);
  digitalWrite(STBY, LOW);
}

static void motorsInit()
{
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, LOW);

  const int dirPins[] = {AIN1, AIN2, BIN1, BIN2};
  for (int p : dirPins)
  {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  pwmAttach(PWMA, LEFT_CH, PWM_FREQ);
  pwmAttach(PWMB, RIGHT_CH, PWM_FREQ);
  motorsStop();
}

// u = balance output (+ = forward), turn = steering (+ = turn right).
// Right turn: left wheel faster, right wheel slower. Both in right-motor PWM units.
static void drive(float u, float turn)
{
  float ur = u - turn;
  float ul = u + turn;
  int r = (int)lroundf(ur) * RIGHT_DIR;
  int l = (int)lroundf(ul * LEFT_SCALE) * LEFT_DIR;

  digitalWrite(STBY, HIGH);
  setMotor(PWMB, RIGHT_CH, BIN1, BIN2, r);
  setMotor(PWMA, LEFT_CH,  AIN1, AIN2, l);
}

[[noreturn]] static void fault(int n, int onMs, int offMs)
{
  motorsStop();
  for (;;)
  {
    beep(n, onMs, offMs);
    beepWait();
    delay(1500);
  }
}

// IMU
// accAngle: accelerometer angle relative to the balance point (deg)
// rate: pitch rate, positive = leaning forward (deg/s)
static void readImu(float &accAngle, float &rate)
{
  sensors_event_t a, g, t;
  mpu.getEvent(&a, &g, &t);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  mpuCorrectAccel(cal, ax, ay, az);

  accAngle = wrap180(accelPitchDeg(ax, ay, az) - cal.balanceAngle);
  rate = (g.gyro.y - cal.gyroBias[1]) * RAD_TO_DEG;
}

// Settings in flash (same "pid" namespace as the balancing sketch)
static void loadPid()
{
  Preferences p;
  if (!p.begin("pid", true)) return;
  kp   = p.getFloat("kp", kp);
  ki   = p.getFloat("ki", ki);
  kd   = p.getFloat("kd", kd);
  trim = p.getFloat("trim", trim);
  maxLeanFwd  = clampf(p.getFloat("lf", maxLeanFwd),  0.0f, LEAN_HARD_CAP);
  maxLeanBack = clampf(p.getFloat("lb", maxLeanBack), 0.0f, LEAN_HARD_CAP);
  maxTurn     = clampf(p.getFloat("tw", maxTurn),     0.0f, TURN_HARD_CAP);
  p.end();
}

static void savePid()
{
  Preferences p;
  p.begin("pid", false);
  p.putFloat("kp", kp);
  p.putFloat("ki", ki);
  p.putFloat("kd", kd);
  p.putFloat("trim", trim);
  p.putFloat("lf", maxLeanFwd);
  p.putFloat("lb", maxLeanBack);
  p.putFloat("tw", maxTurn);
  p.end();
}

// Serial tuning
static void printSettings()
{
  Serial.printf("Kp=%.3f Ki=%.3f Kd=%.3f trim=%.2f | pulse lean fwd=%.1f back=%.1f deg, turn=%.0f PWM | balance=%.3f deg gyroBiasY=%.4f deg/s\n", kp, ki, kd, trim, maxLeanFwd, maxLeanBack, maxTurn, cal.balanceAngle, cal.gyroBias[1] * RAD_TO_DEG);
}

static void handleCmd(char *s)
{
  float v = atof(s + 1);
  switch (tolower(s[0]))
  {
    case 'p': kp = v; break;
    case 'i': ki = v; iTerm = 0.0f; break;
    case 'd': kd = v; break;
    case 't': trim = v; break;
    case 'f': maxLeanFwd  = clampf(v, 0.0f, LEAN_HARD_CAP); break;
    case 'b': maxLeanBack = clampf(v, 0.0f, LEAN_HARD_CAP); break;
    case 'w': maxTurn     = clampf(v, 0.0f, TURN_HARD_CAP); break;
    case 's': savePid(); beep(2, 60, 60); break;
    case 'v': telemetry = !telemetry; break;
    case '?': break;
    default: return;
  }
  printSettings();
}

static void readSerial()
{
  static char buf[24];
  static uint8_t n = 0;

  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\n' || c == '\r')
    {
      if (n)
      {
        buf[n] = 0;
        handleCmd(buf);
        n = 0;
      }
    }
    else if (n < sizeof(buf) - 1)
    {
      buf[n++] = c;
    }
  }
}

// Remote
// One pulse generator per axis. dir = +1 / -1 / 0 (button state).
// While a button is held it fires a pulse of onMs, then rests gapMs, then fires again.
struct Pulser
{
  uint32_t start = 0;
  int dir = 0;
};

static float pulseOut(Pulser &p, int dir, float pos, float neg, uint32_t onMs, uint32_t gapMs)
{
  uint32_t now = millis();

  if (dir != 0 && now - p.start >= onMs + gapMs)
  {
    p.dir = dir;
    p.start = now;
  }
  if (p.dir != 0 && now - p.start < onMs) return p.dir > 0 ? pos : -neg;
  return 0.0f;
}

// Reads the D-pad and returns the wanted lean (deg from balance point) and steering (PWM).
// Always 0 when the controller is not connected.
static void readRemote(float &leanTarget, float &turnTarget)
{
  static Pulser leanP, turnP;

  leanTarget = 0.0f;
  turnTarget = 0.0f;
  if (!PS4.isConnected()) return;

  float scale = PS4.L1() ? GENTLE_SCALE : 1.0f;

  int leanDir = (PS4.Up() ? 1 : 0) - (PS4.Down() ? 1 : 0);
  int turnDir = (PS4.Right() ? 1 : 0) - (PS4.Left() ? 1 : 0);

  leanTarget = pulseOut(leanP, leanDir, maxLeanFwd, maxLeanBack, PULSE_MS, PULSE_GAP_MS) * scale;

#if TURN_PULSED
  turnTarget = pulseOut(turnP, turnDir, maxTurn, maxTurn, TURN_PULSE_MS, TURN_GAP_MS) * scale * STEER_DIR;
#else
  turnTarget = turnDir * maxTurn * scale * STEER_DIR;
#endif
}

static float slewTo(float cur, float target, float maxStep)
{
  return cur + clampf(target - cur, -maxStep, maxStep);
}

// Lean grows fast away from 0 (so a short pulse reaches its height) and comes back faster
static float slewLean(float cur, float target, float dt)
{
  bool growing = (fabsf(target) > fabsf(cur)) && (target * cur >= 0.0f);
  return slewTo(cur, target, (growing ? LEAN_SLEW_UP_DPS : LEAN_SLEW_DOWN_DPS) * dt);
}

// Connection beeps and lightbar colour (blue = waiting, green = balancing)
static void remoteStatus()
{
  static bool was = false;
  static int shown = -1;
  static uint32_t ledT = 0;

  bool conn = PS4.isConnected();
  if (conn != was)
  {
    was = conn;
    Serial.println(conn ? "PS4 connected" : "PS4 lost: commands fade to 0, still balancing");
    if (conn) beep(1, 40);
    else      beep(2, 40, 80);
  }

  int want = conn ? (engaged ? 2 : 1) : 0;
  if (conn && (want != shown || millis() - ledT > 2000))
  {
    ledT = millis();
    if (want == 2) PS4.setLed(0, 255, 0);
    else           PS4.setLed(0, 0, 255);
    PS4.sendToController();
  }
  shown = want;
}

static void disengage(int beepMs)
{
  motorsStop();
  engaged = false;
  holding = false;
  iTerm = 0.0f;
  lastOut = 0.0f;
  leanCmd = 0.0f;
  turnCmd = 0.0f;
  uMag = 0.0f;
  beep(1, beepMs);
}

void setup()
{
  Serial.begin(115200);

  motorsInit();
  buzzerInit();

  beep(2, 100, 100);
  beepWait();
  delay(700);

  if (!mpuCalLoad(cal)) fault(3, 400, 250);
  loadPid();
  beep(1, 200);
  beepWait();

  if (!mpuBegin(mpu)) fault(5, 80, 80);

  bool zeroed = true;
#if GYRO_ZERO_AT_BOOT
  MpuCalibration fresh = cal;
  mpuQuickGyroZero(mpu, fresh);
  float delta = fabsf(fresh.gyroBias[1] - cal.gyroBias[1]) * RAD_TO_DEG;
  if (delta <= GYRO_ZERO_MAX_DELTA_DPS) cal = fresh;
  else zeroed = false;
#endif
  if (zeroed) beep(1, 80);
  else        beep(2, 400, 200);
  beepWait();

  // Bluetooth starts after the gyro zeroing so it cannot disturb it
  PS4.begin();

  float rate;
  readImu(angle, rate);

  printSettings();
  lastUs = micros();
}

void loop()
{
  // Service Bluepad32 every 2 ms. Calling it flat out (thousands of times a
  // second) can starve the Bluetooth task of the lock it needs to hand over data.
  static uint32_t lastBp = 0;
  uint32_t ms = millis();
  if (ms - lastBp >= 2)
  {
    lastBp = ms;
    PS4.update();
  }
  beepUpdate();
  readSerial();
  remoteStatus();

  uint32_t now = micros();
  if (now - lastUs < LOOP_US) return;
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;

  float accAngle, rate;
  readImu(accAngle, rate);

  angle = ALPHA * (angle + rate * dt) + (1.0f - ALPHA) * accAngle;
  if (!isfinite(angle)) fault(5, 80, 80);

  float tilt = angle - trim;    // lean from the balance point

  if (!engaged)
  {
    motorsStop();
    lastOut = 0.0f;
    leanCmd = 0.0f;
    turnCmd = 0.0f;
    uMag = 0.0f;

    if (fabsf(tilt) < UPRIGHT_WINDOW && !PS4.Circle())
    {
      if (!holding)
      {
        holding = true;
        holdStart = millis();
      }
      else if (millis() - holdStart >= UPRIGHT_HOLD_MS)
      {
        iTerm = 0.0f;
        engaged = true;
        holding = false;
        beep(3, 80, 80);
      }
    }
    else
    {
      holding = false;
    }
  }
  else if (fabsf(tilt) > FALL_ANGLE)
  {
    disengage(700);
  }
  else if (PS4.Circle())
  {
    disengage(300);
  }
  else
  {
    float leanTarget, turnTarget;
    readRemote(leanTarget, turnTarget);

    // Permissible lean shrinks as the motors run out of headroom
    float sat = uMag / 255.0f;
    float headroom = 1.0f - clampf((sat - SAT_START) / (1.0f - SAT_START), 0.0f, 1.0f);
    leanTarget *= headroom;

    leanCmd = slewLean(leanCmd, leanTarget, dt);
    turnCmd = slewTo(turnCmd, turnTarget, TURN_SLEW_PER_S * dt);

    float err = tilt - leanCmd;
    iTerm = constrain(iTerm + ki * err * dt, -I_MAX, I_MAX);
    float u = kp * err + iTerm + kd * rate;
    u = constrain(u, -255.0f, 255.0f);
    lastOut = u;
    uMag += (fabsf(u) - uMag) * (dt / (SAT_TAU + dt));

    // Steering always keeps some authority, even when balance saturates a wheel
    float room = max(255.0f - fabsf(u), TURN_MIN_ROOM);
    drive(u, clampf(turnCmd, -room, room));
  }

  if (telemetry && millis() - lastTel >= 100)
  {
    lastTel = millis();
    Serial.printf("ANGLE=%8.3f  ACCEL=%8.3f  RATE=%9.3f  LEAN=%6.2f  TURN=%5.0f  OUT=%5.0f  PAD=%d  DATA=%d  DPAD=0x%02X  %s\n", angle, accAngle, rate, leanCmd, turnCmd, lastOut, PS4.isConnected(), PS4.hasData(), PS4.dpadRaw(), engaged ? "RUN" : "WAIT");
  }
}