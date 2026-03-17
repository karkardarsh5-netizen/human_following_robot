// ── Motor pins ─────────────────────────────────────────────────
int IN1 = 2;
int IN2 = 3;
int IN3 = 4;
int IN4 = 5;
int ENA = 6;   // left  motor PWM ✔
int ENB = 9;   // right motor PWM ✔

// ── Sensor pins ────────────────────────────────────────────────
int trigm = 8;  int echom = 10;
int trigl = 11; int echol = 12;
int trigr = 13; int echor = 7;

// ── Thresholds ─────────────────────────────────────────────────
const float TOO_CLOSE    = 30.0;
const float TOO_FAR      = 45.0;
const float MAX_FOLLOW   = 100.0;
const float STEER_MARGIN = 8.0;
const float IGNORE_DIFF  = 15.0;

const int SPEED = 100;

long Df, Dl, Dr;

long prevL = 0, prevF = 0, prevR = 0;

// ══════════════════════════════════════════════════════════════
// Basic read for any sensor
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);

  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 15000);
  long distance = duration * 0.034 / 2;

  if (distance == 0) return -1; // invalid
  return distance;
}

// Median filter for any sensor
long getStableDistance(int trig, int echo, long &prev) {
  long values[3];

  for (int i = 0; i < 5; i++) {
    values[i] = readUltrasonic(trig, echo);
    delay(10);
  }

  // Sort
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (values[j] < values[i]) {
        long t = values[i];
        values[i] = values[j];
        values[j] = t;
      }
    }
  }

  long median = values[2];

  // Ignore sudden jump
  if (prev != 0 && abs(median - prev) > 20) {
    return prev;
  }

  prev = median;
  return median;
}
// ══════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(9600);

  pinMode(trigm, OUTPUT); pinMode(echom, INPUT);
  pinMode(trigl, OUTPUT); pinMode(echol, INPUT);
  pinMode(trigr, OUTPUT); pinMode(echor, INPUT);

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

  stopcar();
}

// ══════════════════════════════════════════════════════════════
void loop() {
  Df = getStableDistance(trigm, echom, prevF);
 delay(10);
  Dl = getStableDistance(trigl, echol, prevL);
  delay(10);
  Dr = getStableDistance(trigr, echor, prevR);
 delay(10);

  Serial.print("F:"); Serial.print(Df);
  Serial.print(" L:"); Serial.print(Dl);
  Serial.print(" R:"); Serial.println(Dr);

  navigate();
  delay(20);
}

// ══════════════════════════════════════════════════════════════
//  NAVIGATE
// ══════════════════════════════════════════════════════════════
void navigate() {

  // ── Too close — reverse only, no steering ─────────────────
  if (Df <= TOO_CLOSE) {
    backward();
    Serial.print(">> BACKWARD");
    
  }

  // ── Beyond max range — stop, no steering ──────────────────
 else if (Df > MAX_FOLLOW) {
    forward();
    Serial.print(">> FORWARD");
    
  }

  // ── 40–100 cm zone — decide forward or hold ───────────────
  else if (Df >= TOO_FAR && Df <= MAX_FOLLOW) {
    forward();
    Serial.print(">> FORWARD");
  } else {
    // Sweet spot 40–60 cm — stay still but allow steering
    stopcar();
    Serial.print(">> HOLD");
  }

  // ── Steering — runs in both HOLD and FORWARD zones ────────

  // Ignore if L and R difference is too small — person centred
  if (abs(Dr - Dl) < IGNORE_DIFF) {
    Serial.println("          steer: CENTRED");
    return;
  }

  // Person drifted left
 else if (Dl < MAX_FOLLOW - STEER_MARGIN && Dl < Dr && Df > MAX_FOLLOW) {
    left();
    Serial.println("          steer: LEFT");
    
  }

  // Person drifted right
 else if (Dr < MAX_FOLLOW - STEER_MARGIN && Dr < Dl && Df > MAX_FOLLOW) {
    right();
    Serial.println("          steer: RIGHT");
    
  }
  else if(Dr > MAX_FOLLOW && Dl > MAX_FOLLOW) {
    Serial.println("          steer: CENTRED");
    return;
  }
  
}

// ══════════════════════════════════════════════════════════════
//  MOTOR CONTROL — your wiring
// ══════════════════════════════════════════════════════════════
void forward() {
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void backward() {
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void left() {
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);
  digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
}

void right() {
  analogWrite(ENA, SPEED); analogWrite(ENB, SPEED);
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
}

void stopcar() {
  analogWrite(ENA, 0); analogWrite(ENB, 0);
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
