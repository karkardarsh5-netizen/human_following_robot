// Human-following robot for Arduino Nano + L298N + 3x HC-SR04
// Motors use IN1/IN2/IN3/IN4 only (no PWM speed control).

// ---------------- Motor pins (L298N) ----------------
const byte IN1 = 2;
const byte IN2 = 3;
const byte IN3 = 4;
const byte IN4 = 5;

// Optional enable pins (set HIGH once if connected)
const byte ENA = 6;
const byte ENB = 9;

// ---------------- Ultrasonic pins ----------------
const byte TRIG_MID   = 8;
const byte ECHO_MID   = 10;
const byte TRIG_LEFT  = 11;
const byte ECHO_LEFT  = 12;
const byte TRIG_RIGHT = 13;
const byte ECHO_RIGHT = 7;

// ---------------- Behavior settings ----------------
const int FOLLOW_MIN_CM = 30;     // < 30 => backward
const int FOLLOW_MAX_CM = 100;    // 30..100 => forward, >100 => stop
const int SIDE_SIMILAR_CM = 5;    // left/right diff within this => straight

// Ultrasonic robustness
const unsigned long ECHO_TIMEOUT_US = 25000UL; // ~4.3m max
const int INVALID_DISTANCE_CM = -1;
const int DEFAULT_SAFE_DISTANCE_CM = 200;
const byte FILTER_SAMPLES = 5;                // odd number for median
const int MAX_JUMP_CM = 60;                   // ignore sudden spikes
const unsigned long INTER_SENSOR_DELAY_MS = 12;

// Keep last accepted values for spike rejection
int lastLeft = DEFAULT_SAFE_DISTANCE_CM;
int lastMid = DEFAULT_SAFE_DISTANCE_CM;
int lastRight = DEFAULT_SAFE_DISTANCE_CM;

// ---------------- Motor control ----------------
void forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void left() {
  // Left motor backward, right motor forward (pivot left)
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
}

void right() {
  // Left motor forward, right motor backward (pivot right)
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
}

void stopcar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ---------------- Ultrasonic helpers ----------------
int rawDistanceCm(byte trigPin, byte echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(3);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) {
    return INVALID_DISTANCE_CM;
  }

  int distance = (int)(duration * 0.0343f * 0.5f);
  if (distance <= 0) {
    return INVALID_DISTANCE_CM;
  }

  return distance;
}

int medianOfFive(int a, int b, int c, int d, int e) {
  int arr[5] = {a, b, c, d, e};

  // Insertion sort (small + efficient for fixed size)
  for (byte i = 1; i < 5; i++) {
    int key = arr[i];
    int j = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }

  return arr[2];
}

int filteredDistanceCm(byte trigPin, byte echoPin, int lastGood) {
  int valid[5];
  byte validCount = 0;

  // Read this sensor multiple times, keeping only valid values
  for (byte i = 0; i < FILTER_SAMPLES; i++) {
    int d = rawDistanceCm(trigPin, echoPin);
    if (d != INVALID_DISTANCE_CM) {
      valid[validCount++] = d;
    }
    delay(2);
  }

  // No valid readings: keep last value (safe fallback)
  if (validCount == 0) {
    return lastGood;
  }

  int candidate;
  if (validCount >= 5) {
    candidate = medianOfFive(valid[0], valid[1], valid[2], valid[3], valid[4]);
  } else {
    long sum = 0;
    for (byte i = 0; i < validCount; i++) {
      sum += valid[i];
    }
    candidate = (int)(sum / validCount);
  }

  // Spike rejection: ignore sudden unrealistic jump
  if (abs(candidate - lastGood) > MAX_JUMP_CM) {
    return lastGood;
  }

  return candidate;
}

// ---------------- Main logic ----------------
void setup() {
  Serial.begin(9600);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_MID, OUTPUT);
  pinMode(ECHO_MID, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // If ENA/ENB are wired, force them ON once.
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  stopcar();
}

void loop() {
  // Read sensors one by one to reduce ultrasonic cross-talk
  lastLeft = filteredDistanceCm(TRIG_LEFT, ECHO_LEFT, lastLeft);
  delay(INTER_SENSOR_DELAY_MS);

  lastMid = filteredDistanceCm(TRIG_MID, ECHO_MID, lastMid);
  delay(INTER_SENSOR_DELAY_MS);

  lastRight = filteredDistanceCm(TRIG_RIGHT, ECHO_RIGHT, lastRight);

  const int leftDist = lastLeft;
  const int midDist = lastMid;
  const int rightDist = lastRight;

  const bool sideSimilar = abs(leftDist - rightDist) <= SIDE_SIMILAR_CM;
  const bool preferLeft = leftDist < rightDist;
  const bool preferRight = rightDist < leftDist;

  const char* action = "STOP";

  // Rule 1: follow behavior by middle sensor distance
  if (midDist < FOLLOW_MIN_CM) {
    // Too close: move backward (safe retreat)
    backward();
    action = "BACKWARD";
  } else if (midDist <= FOLLOW_MAX_CM) {
    // Follow range: forward + direction correction by side sensors
    if (sideSimilar) {
      forward();
      action = "FORWARD";
    } else if (preferLeft) {
      left();
      action = "LEFT";
    } else if (preferRight) {
      right();
      action = "RIGHT";
    } else {
      forward();
      action = "FORWARD";
    }
  } else {
    // Too far: stop and wait
    stopcar();
    action = "STOP";
  }

  // Serial monitor output
  Serial.print("L:");
  Serial.print(leftDist);
  Serial.print(" cm  M:");
  Serial.print(midDist);
  Serial.print(" cm  R:");
  Serial.print(rightDist);
  Serial.print(" cm  Action:");
  Serial.println(action);

  // Small delay for responsiveness without flooding serial output
  delay(20);
}
