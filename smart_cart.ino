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

float Df, Dl, Dr;

// ══════════════════════════════════════════════════════════════
float readDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(4);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long dur = pulseIn(echoPin, HIGH, 25000);
  if (dur == 0) return 999;
  return 0.034 * dur / 2.0;
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
  delay(20);
}

// ══════════════════════════════════════════════════════════════
void loop() {
  Df = readDistance(trigm, echom);
  Dl = readDistance(trigl, echol);
  Dr = readDistance(trigr, echor);

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
 else if (Dl < MAX_FOLLOW - STEER_MARGIN && Dl < Dr) {
    left();
    Serial.println("          steer: LEFT");
    
  }

  // Person drifted right
 else if (Dr < MAX_FOLLOW - STEER_MARGIN && Dr < Dl) {
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
