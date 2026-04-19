//this project done by BENBERKAT AYOUB & BOUCETTA SLIMANE
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

// ─── Pin Definitions ─────────────────────────────────────────────────────────

#define SDA_PIN 21
#define SCL_PIN 22

// VL53L0X XSHUT pins (used to init each sensor one by one)
#define XSHUT_LEFT   15
#define XSHUT_CENTER  2
#define XSHUT_RIGHT   4

// I2C addresses assigned to each ToF sensor
#define ADDR_LEFT   0x30
#define ADDR_CENTER 0x39
#define ADDR_RIGHT  0x32

// Motor A pins
#define MA_IN1 12
#define MA_IN2 14

// Motor B pins
#define MB_IN1 27
#define MB_IN2 26

// IR line sensor pins (LOW = white line detected)
#define IR_LEFT   32
#define IR_CENTER 33
#define IR_RIGHT  25

// Status LED
#define LED_PIN 13

// ─── Tuning Parameters ────────────────────────────────────────────────────────

// Distance thresholds in mm
#define ATTACK_THRESHOLD  800   // Center sensor: attack if opponent within this range
#define SIDE_THRESHOLD    570   // Side sensors: start turning if opponent within this range
#define MAX_RANGE        1200   // Ignore readings beyond this

// Motor speeds (0–255)
#define SPEED_FWD  230
#define SPEED_TURN 180
#define SPEED_REV  195

// PWM config
#define PWM_FREQ 2000
#define PWM_BITS 8

// Search behavior timing
#define LOOP_INTERVAL_MS     30
#define SEARCH_INTERVAL_MS 1800
#define SWEEP_DURATION_MS   450
#define WOBBLE_PERIOD_MS    300

// EMA filter weight (higher = faster response, less smoothing)
#define EMA_ALPHA 0.8f

// ─── Globals ─────────────────────────────────────────────────────────────────

Adafruit_VL53L0X tof_left   = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_center = Adafruit_VL53L0X();
Adafruit_VL53L0X tof_right  = Adafruit_VL53L0X();

float ema_left = -1, ema_center = -1, ema_right = -1;

unsigned long last_loop      = 0;
unsigned long last_search    = 0;
unsigned long sweep_start    = 0;
bool in_sweep                = false;
int  sweep_dir               = 1;   // +1 = right, -1 = left

// ─── Function Prototypes ─────────────────────────────────────────────────────

void     init_tof(Adafruit_VL53L0X *sensor, int xshut, uint8_t addr, const char *label);
uint16_t read_tof(Adafruit_VL53L0X &sensor, float &ema);
void     set_motor_a(int pwm, bool fwd);
void     set_motor_b(int pwm, bool fwd);
void     stop_motors();
void     drive_forward(int speed);
void     drive_reverse(int speed);
void     spin_left(int speed);
void     spin_right(int speed);
void     handle_line_sensors(int l, int c, int r);
void     handle_distance_sensors(unsigned long now);

// ─── Setup ───────────────────────────────────────────────────────────────────

void setup() {
  // Blink LED to signal boot
  pinMode(LED_PIN, OUTPUT);
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(200);
    digitalWrite(LED_PIN, LOW);  delay(200);
  }

  Serial.begin(115200);
  Serial.println("Sumo robot starting...");

  // Init I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  // Disable all ToF sensors before init
  pinMode(XSHUT_LEFT,   OUTPUT); digitalWrite(XSHUT_LEFT,   LOW);
  pinMode(XSHUT_CENTER, OUTPUT); digitalWrite(XSHUT_CENTER, LOW);
  pinMode(XSHUT_RIGHT,  OUTPUT); digitalWrite(XSHUT_RIGHT,  LOW);
  delay(10);

  // Init sensors one by one and assign unique I2C addresses
  init_tof(&tof_left,   XSHUT_LEFT,   ADDR_LEFT,   "LEFT");
  init_tof(&tof_center, XSHUT_CENTER, ADDR_CENTER, "CENTER");
  init_tof(&tof_right,  XSHUT_RIGHT,  ADDR_RIGHT,  "RIGHT");

  // Attach motor pins to PWM channels
  ledcAttach(MA_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(MA_IN2, PWM_FREQ, PWM_BITS);
  ledcAttach(MB_IN1, PWM_FREQ, PWM_BITS);
  ledcAttach(MB_IN2, PWM_FREQ, PWM_BITS);

  // IR line sensors
  pinMode(IR_LEFT,   INPUT);
  pinMode(IR_CENTER, INPUT);
  pinMode(IR_RIGHT,  INPUT);

  stop_motors();

  last_loop   = millis();
  last_search = millis();
}

// ─── Main Loop ───────────────────────────────────────────────────────────────

void loop() {
  unsigned long now = millis();

  // Rate-limit the loop
  if (now - last_loop < LOOP_INTERVAL_MS) return;
  last_loop = now;

  int ir_l = digitalRead(IR_LEFT);
  int ir_c = digitalRead(IR_CENTER);
  int ir_r = digitalRead(IR_RIGHT);

  Serial.printf("IR  L:%d  C:%d  R:%d\n", ir_l, ir_c, ir_r);

  // Line sensors take priority over everything else
  if (ir_l == LOW || ir_c == LOW || ir_r == LOW) {
    handle_line_sensors(ir_l, ir_c, ir_r);
    return;
  }

  // No line detected — look for opponent
  handle_distance_sensors(now);
}

// ─── ToF Sensor Initialization ───────────────────────────────────────────────

void init_tof(Adafruit_VL53L0X *sensor, int xshut, uint8_t addr, const char *label) {
  // Power-cycle this sensor
  digitalWrite(xshut, LOW);  delay(5);
  digitalWrite(xshut, HIGH); delay(10);

  if (!sensor->begin()) {
    Serial.printf("ERROR: %s sensor failed to boot\n", label);
    return;
  }

  if (!sensor->setAddress(addr)) {
    Serial.printf("ERROR: %s address assignment failed\n", label);
  } else {
    Serial.printf("%s sensor ready at 0x%02X\n", label, addr);
  }
}

// ─── ToF Distance Reading with EMA Smoothing ─────────────────────────────────

uint16_t read_tof(Adafruit_VL53L0X &sensor, float &ema) {
  VL53L0X_RangingMeasurementData_t data;
  sensor.rangingTest(&data, false);

  uint16_t dist;

  if (data.RangeStatus != 4) {
    dist = data.RangeMilliMeter;
    if (dist == 0 || dist > MAX_RANGE) dist = MAX_RANGE + 1;
  } else {
    dist = MAX_RANGE + 1;
  }

  // Initialize or update EMA
  if (ema < 0) ema = dist;
  else         ema = ema * (1.0f - EMA_ALPHA) + dist * EMA_ALPHA;

  return dist;
}

// ─── Motor Control ───────────────────────────────────────────────────────────

void set_motor_a(int pwm, bool fwd) {
  pwm = constrain(pwm, 0, 255);
  ledcWrite(MA_IN1, fwd ? pwm : 0);
  ledcWrite(MA_IN2, fwd ? 0 : pwm);
}

void set_motor_b(int pwm, bool fwd) {
  pwm = constrain(pwm, 0, 255);
  ledcWrite(MB_IN1, fwd ? pwm : 0);
  ledcWrite(MB_IN2, fwd ? 0 : pwm);
}

void stop_motors()           { set_motor_a(0, true);           set_motor_b(0, true); }
void drive_forward(int spd)  { set_motor_a(spd, true);         set_motor_b(spd, true); }
void drive_reverse(int spd)  { set_motor_a(spd, false);        set_motor_b(spd, false); }
void spin_left(int spd)      { set_motor_a(spd, false);        set_motor_b(spd, true); }
void spin_right(int spd)     { set_motor_a(spd, true);         set_motor_b(spd, false); }

// ─── Line Sensor Handler ─────────────────────────────────────────────────────

void handle_line_sensors(int l, int c, int r) {
  // All three triggered: full front edge
  if (l == LOW && c == LOW && r == LOW) {
    drive_reverse(SPEED_REV); delay(400);
    spin_right(SPEED_TURN);   delay(300);
  }
  // Center only
  else if (c == LOW && l == HIGH && r == HIGH) {
    drive_reverse(SPEED_REV); delay(300);
    if (random(2)) spin_left(SPEED_TURN);
    else           spin_right(SPEED_TURN);
    delay(200);
  }
  // Left only
  else if (l == LOW && c == HIGH && r == HIGH) {
    drive_reverse(SPEED_REV); delay(200);
    spin_right(SPEED_TURN);   delay(250);
  }
  // Right only
  else if (r == LOW && c == HIGH && l == HIGH) {
    drive_reverse(SPEED_REV); delay(200);
    spin_left(SPEED_TURN);    delay(250);
  }
  // Left + center
  else if (l == LOW && c == LOW) {
    drive_reverse(SPEED_REV); delay(300);
    spin_right(SPEED_TURN);   delay(250);
  }
  // Right + center
  else if (r == LOW && c == LOW) {
    drive_reverse(SPEED_REV); delay(300);
    spin_left(SPEED_TURN);    delay(250);
  }

  stop_motors();
}

// ─── Distance Sensor Handler ─────────────────────────────────────────────────

void handle_distance_sensors(unsigned long now) {
  uint16_t d_left   = read_tof(tof_left,   ema_left);
  uint16_t d_center = read_tof(tof_center, ema_center);
  uint16_t d_right  = read_tof(tof_right,  ema_right);

  Serial.printf("ToF  L:%4d  C:%4d  R:%4d mm\n", d_left, d_center, d_right);

  // Determine which sensor has the closest valid reading
  bool see_left   = (d_left   <= SIDE_THRESHOLD);
  bool see_center = (d_center <= ATTACK_THRESHOLD);
  bool see_right  = (d_right  <= SIDE_THRESHOLD);

  if (see_center) {
    drive_forward(SPEED_FWD);
  } else if (see_left) {
    spin_left(SPEED_TURN); delay(120);
    drive_forward(SPEED_FWD);
  } else if (see_right) {
    spin_right(SPEED_TURN); delay(120);
    drive_forward(SPEED_FWD);
  } else {
    // No opponent in range — search pattern
    search_for_opponent(now);
  }
}

// ─── Search Pattern (no opponent visible) ────────────────────────────────────

void search_for_opponent(unsigned long now) {
  // Micro-wobble: slight left/right bias while driving forward
  unsigned long phase = now % WOBBLE_PERIOD_MS;
  if (phase < WOBBLE_PERIOD_MS / 2) {
    set_motor_a(SPEED_FWD - 20, true);
    set_motor_b(SPEED_FWD,      true);
  } else {
    set_motor_a(SPEED_FWD,      true);
    set_motor_b(SPEED_FWD - 20, true);
  }

  // Periodic sweep: spin in alternating directions to scan the arena
  if (now - last_search >= SEARCH_INTERVAL_MS) {
    last_search = now;
    sweep_start = now;
    in_sweep    = true;
    sweep_dir   = -sweep_dir;  // alternate direction each time
  }

  if (in_sweep) {
    unsigned long elapsed = now - sweep_start;
    if (elapsed < SWEEP_DURATION_MS) {
      if (sweep_dir > 0) spin_right(SPEED_TURN);
      else               spin_left(SPEED_TURN);
    } else {
      in_sweep = false;
      stop_motors();
    }
  }
}
