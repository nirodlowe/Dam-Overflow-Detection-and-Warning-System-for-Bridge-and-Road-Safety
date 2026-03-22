#include <Wire.h>
#include <Servo.h>
#include <LiquidCrystal_I2C.h>

/* ================= LCD 16x2 I2C ================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);

/* ================= PINS ================= */
#define TRIG1 8
#define ECHO1 9

#define TRIG2 10
#define ECHO2 11

#define LED_GREEN  2
#define LED_YELLOW 3
#define LED_RED    4

#define BUZZER1 5
#define BUZZER2 12

#define SERVO1_PIN 6
#define SERVO2_PIN 7

#define ROAD_LED1 A1
#define ROAD_LED2 A2

/* ================= DAM THRESHOLDS ================= */
/*
  Dam logic:
  > 17 cm   -> Stage 1 (Green)
  <= 17 cm  -> Stage 2 (Yellow)
  <= 14 cm  -> Stage 3 (Red + buzzer)
  <= 11 cm  -> Servo start delay, then open gates
*/
#define STAGE3_CM       14
#define STAGE2_CM       17
#define SERVO_START_CM  11

/* ================= BRIDGE THRESHOLDS ================= */
/*
  Bridge logic:
  > 5 cm              -> Safe
  <= 5 cm and > 4 cm  -> Stage 1 (ROAD_LED1 blinks)
  <= 4 cm             -> Stage 2 (ROAD_LED2 blinks + buzzer)
*/
#define BRIDGE_STAGE1_CM 5
#define BRIDGE_STAGE2_CM 4

#define DAM_HEIGHT_CM 30

/* ================= SERVO SETTINGS ================= */
#define SERVO_CLOSED 0
#define SERVO_OPEN   180
#define SERVO_OPEN_DELAY_MS 2000

Servo gate1, gate2;
bool gatesOpen = false;
unsigned long stage3Start = 0;

/* ================= TIMING ================= */
const unsigned long SENSOR_PERIOD_MS = 250;
unsigned long lastSensorRead = 0;

const unsigned long BLINK_MS = 300;
unsigned long bridgeNextToggle = 0;
bool bridgeBlinkState = false;

unsigned long damBuzzNext = 0;
int damBuzzPhase = 0;

unsigned long bridgeBuzzNext = 0;
int bridgeBuzzPhase = 0;

/* ================= HELPERS ================= */
long readDistanceCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  unsigned long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return -1;

  return (long)(duration * 0.034 / 2.0);
}

int calcWaterLevelCM(long distCM) {
  if (distCM < 0) return -1;

  int level = DAM_HEIGHT_CM - (int)distCM;

  if (level < 0) level = 0;
  if (level > DAM_HEIGHT_CM) level = DAM_HEIGHT_CM;

  return level;
}

void openGates() {
  gate1.write(SERVO_OPEN);
  gate2.write(SERVO_OPEN);
  gatesOpen = true;
}

void closeGates() {
  gate1.write(SERVO_CLOSED);
  gate2.write(SERVO_CLOSED);
  gatesOpen = false;
}

void damBuzzerUpdate(bool enable) {
  unsigned long now = millis();

  if (!enable) {
    noTone(BUZZER1);
    damBuzzPhase = 0;
    damBuzzNext = 0;
    return;
  }

  if (damBuzzNext == 0) {
    tone(BUZZER1, 1000);
    damBuzzPhase = 0;
    damBuzzNext = now + 200;
    return;
  }

  if (now >= damBuzzNext) {
    damBuzzPhase++;

    if (damBuzzPhase == 1) {
      noTone(BUZZER1);
      damBuzzNext = now + 200;
    }
    else if (damBuzzPhase == 2) {
      tone(BUZZER1, 1000);
      damBuzzNext = now + 200;
    }
    else if (damBuzzPhase == 3) {
      noTone(BUZZER1);
      damBuzzNext = now + 800;
    }
    else {
      damBuzzPhase = 0;
      tone(BUZZER1, 1000);
      damBuzzNext = now + 200;
    }
  }
}

void bridgeBuzzerUpdate(bool enable) {
  unsigned long now = millis();

  if (!enable) {
    noTone(BUZZER2);
    bridgeBuzzPhase = 0;
    bridgeBuzzNext = 0;
    return;
  }

  if (bridgeBuzzNext == 0) {
    tone(BUZZER2, 1200);
    bridgeBuzzPhase = 0;
    bridgeBuzzNext = now + 150;
    return;
  }

  if (now >= bridgeBuzzNext) {
    bridgeBuzzPhase++;

    if (bridgeBuzzPhase == 1) {
      noTone(BUZZER2);
      bridgeBuzzNext = now + 150;
    }
    else {
      bridgeBuzzPhase = 0;
      tone(BUZZER2, 1200);
      bridgeBuzzNext = now + 150;
    }
  }
}

int damStageFromDist(long damDistCM) {
  // returns: -1 error, 1 stage1, 2 stage2, 3 stage3
  if (damDistCM <= 0) return -1;
  if (damDistCM <= STAGE3_CM) return 3;  // <=14 cm
  if (damDistCM <= STAGE2_CM) return 2;  // <=17 cm
  return 1;                              // >17 cm
}

/* ================= LCD DISPLAY ================= */
void updateLCD_Always(long damDistCM, long bridgeDistCM) {
  static unsigned long lastLCDUpdate = 0;
  static int page = 0;

  if (millis() - lastLCDUpdate < 1200) return;
  lastLCDUpdate = millis();

  lcd.clear();

  int st = damStageFromDist(damDistCM);

  if (st == -1) {
    lcd.setCursor(0, 0);
    lcd.print("DAM SENSOR ERR");
    lcd.setCursor(0, 1);
    lcd.print("CHECK SENSOR");
    return;
  }

  int level = calcWaterLevelCM(damDistCM);

  if (page == 0) {
    lcd.setCursor(0, 0);
    lcd.print("Dam:");
    lcd.print(damDistCM);
    lcd.print("cm");

    lcd.setCursor(0, 1);
    lcd.print("Br:");
    if (bridgeDistCM > 0) lcd.print(bridgeDistCM);
    else lcd.print("ERR");
    lcd.print("cm");
  }
  else if (page == 1) {
    lcd.setCursor(0, 0);
    lcd.print("Level:");
    lcd.print(level);
    lcd.print("cm");

    lcd.setCursor(0, 1);
    lcd.print("Gate:");
    lcd.print(gatesOpen ? "OPEN" : "CLOSE");
  }
  else {
    lcd.setCursor(0, 0);
    lcd.print("Dam Status:");

    lcd.setCursor(0, 1);
    if (st == 3) lcd.print("STAGE3");
    else if (st == 2) lcd.print("STAGE2");
    else if (st == 1) lcd.print("STAGE1");
    else lcd.print("ERROR");
  }

  page++;
  if (page > 2) page = 0;
}

void updateBridgeBlinkStage(int stage) {
  unsigned long now = millis();

  if (stage == 0) {
    digitalWrite(ROAD_LED1, LOW);
    digitalWrite(ROAD_LED2, LOW);
    bridgeNextToggle = 0;
    bridgeBlinkState = false;
    return;
  }

  if (bridgeNextToggle == 0) {
    bridgeBlinkState = true;
    bridgeNextToggle = now + BLINK_MS;
  }
  else if (now >= bridgeNextToggle) {
    bridgeBlinkState = !bridgeBlinkState;
    bridgeNextToggle = now + BLINK_MS;
  }

  if (stage == 1) {
    digitalWrite(ROAD_LED1, bridgeBlinkState);
    digitalWrite(ROAD_LED2, LOW);
  }
  else {
    digitalWrite(ROAD_LED1, LOW);
    digitalWrite(ROAD_LED2, bridgeBlinkState);
  }
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(9600);

  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);

  pinMode(ROAD_LED1, OUTPUT);
  pinMode(ROAD_LED2, OUTPUT);

  pinMode(BUZZER1, OUTPUT);
  pinMode(BUZZER2, OUTPUT);

  gate1.attach(SERVO1_PIN);
  gate2.attach(SERVO2_PIN);
  closeGates();

  Wire.begin();
  lcd.init();
  lcd.backlight();

  lcd.setCursor(0, 0);
  lcd.print("DAM MONITORING");
  lcd.setCursor(0, 1);
  lcd.print("SYSTEM READY");
  delay(1500);
  lcd.clear();
}

/* ================= LOOP ================= */
void loop() {
  unsigned long now = millis();

  static long damDist = -1;
  static long bridgeDist = -1;

  // Read sensors periodically
  if (now - lastSensorRead >= SENSOR_PERIOD_MS) {
    lastSensorRead = now;
    damDist = readDistanceCM(TRIG1, ECHO1);
    bridgeDist = readDistanceCM(TRIG2, ECHO2);

    Serial.print("DamDist = ");
    Serial.print(damDist);
    Serial.print(" cm   BridgeDist = ");
    Serial.print(bridgeDist);
    Serial.println(" cm");
  }

  // Always update LCD
  updateLCD_Always(damDist, bridgeDist);

  // Default OFF dam LEDs
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  int damStage = damStageFromDist(damDist);

  /* ----- DAM LOGIC ----- */
  if (damStage == 3) {                      // <= 14 cm
    digitalWrite(LED_RED, HIGH);
    damBuzzerUpdate(true);                  // buzzer starts at 14 cm

    // Servo starts only when <= 11 cm
    if (damDist <= SERVO_START_CM) {
      if (stage3Start == 0) stage3Start = now;

      if (!gatesOpen && (now - stage3Start >= SERVO_OPEN_DELAY_MS)) {
        openGates();
        Serial.println(">> BOTH GATES OPENED to 180");
      }
    } else {
      stage3Start = 0;
    }
  }
  else if (damStage == 2) {                 // <= 17 cm
    digitalWrite(LED_YELLOW, HIGH);
    damBuzzerUpdate(false);
    stage3Start = 0;

    if (gatesOpen) {
      closeGates();
      Serial.println(">> BOTH GATES CLOSED back to 0 (at STAGE2)");
    }
  }
  else if (damStage == 1) {                 // > 17 cm
    digitalWrite(LED_GREEN, HIGH);
    damBuzzerUpdate(false);
    stage3Start = 0;

    if (gatesOpen) {
      closeGates();
      Serial.println(">> BOTH GATES CLOSED back to 0 (at STAGE1)");
    }
  }
  else {                                    // error
    damBuzzerUpdate(false);
    stage3Start = 0;

    if (gatesOpen) {
      closeGates();
      Serial.println(">> BOTH GATES CLOSED (ERROR)");
    }
  }

  /* ----- BRIDGE LOGIC ----- */
  int bridgeStage = 0;

  if (bridgeDist > 0) {
    if (bridgeDist <= BRIDGE_STAGE2_CM) {
      bridgeStage = 2;   // <= 4 cm
    }
    else if (bridgeDist <= BRIDGE_STAGE1_CM && bridgeDist > BRIDGE_STAGE2_CM) {
      bridgeStage = 1;   // <= 5 cm and > 4 cm
    }
    else {
      bridgeStage = 0;   // > 5 cm
    }
  }

  updateBridgeBlinkStage(bridgeStage);

  bool bridgeDanger = (bridgeStage == 2);
  bridgeBuzzerUpdate(bridgeDanger);
}