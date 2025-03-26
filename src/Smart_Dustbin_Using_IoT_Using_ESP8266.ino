#define BLYNK_TEMPLATE_ID   "TMPL6hgZg8Yiy"
#define BLYNK_TEMPLATE_NAME "Smart Dustbin Using IoT Using ESP8266"
#define BLYNK_AUTH_TOKEN    "iQ1PgQ-Yf7MEx8Qa_nSNcPN6_6QRpvi2"

#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <Servo.h>

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Creative House";
char pass[] = "Creative2025";

#define IR_SENSOR_PIN D5
#define SERVO_PIN     D6
#define TRIG_PIN      D7
#define ECHO_PIN      D8
#define BUZZER_PIN    D2

Servo myservo;
int servoAngle = 0;

long duration;
int distance;
int fullness;
bool isFull = false;

BlynkTimer timer;

// Servo/Lid control variables
bool lidIsOpen = false;
unsigned long lidLastDetectedTime = 0;
const unsigned long lidOpenDuration = 3000; // ৩ সেকেন্ড

//--------------------------------------
// Servo/Lid control functions
//--------------------------------------
void openLid() {
  myservo.write(90);      // লিড ৯০° এ খুলবে
  servoAngle = 90;
  lidIsOpen = true;
  Blynk.virtualWrite(V1, servoAngle); // Blynk এ আপডেট
}

void closeLid() {
  myservo.write(0);       // লিড বন্ধ (০°)
  servoAngle = 0;
  lidIsOpen = false;
  Blynk.virtualWrite(V1, servoAngle); // Blynk এ আপডেট
}

//--------------------------------------
// IR Sensor check with non-blocking delay
//--------------------------------------
void checkIrSensor() {
  if (digitalRead(IR_SENSOR_PIN) == LOW) {
    Blynk.virtualWrite(V0, "Hand Detected");
    // হাত সনাক্ত হলে detection time আপডেট করুন
    lidLastDetectedTime = millis();
    if (!lidIsOpen) {
      openLid();
    }
  } else {
    Blynk.virtualWrite(V0, "Not Detected");
    // হাত সরিয়ে গেলে ৩ সেকেন্ড পর লিড বন্ধ করুন
    if (lidIsOpen && (millis() - lidLastDetectedTime >= lidOpenDuration)) {
      closeLid();
    }
  }
  // সর্বদা servo angle আপডেট করুন
  Blynk.virtualWrite(V1, servoAngle);
}

//--------------------------------------
// Ultrasonic sensor check
//--------------------------------------
void checkDustbinFullness() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2; // cm তে দূরত্ব
  
  // ধরুন 30cm = খালি, 1cm = পূর্ণ => fullness হিসাব করুন
  fullness = map(distance, 30, 1, 0, 100);
  fullness = constrain(fullness, 0, 100);
  Blynk.virtualWrite(V2, fullness);
  
  if (fullness >= 100 && !isFull) {
    digitalWrite(BUZZER_PIN, HIGH);
    Blynk.virtualWrite(V3, 1);
    isFull = true;
  } else if (fullness < 100 && isFull) {
    digitalWrite(BUZZER_PIN, LOW);
    Blynk.virtualWrite(V3, 0);
    isFull = false;
  }
}

//--------------------------------------
// Sensor update function
//--------------------------------------
void updateSensors() {
  checkIrSensor();
  checkDustbinFullness();
}

void setup() {
  Serial.begin(115200);
  Blynk.begin(auth, ssid, pass);

  pinMode(IR_SENSOR_PIN, INPUT);
  myservo.attach(SERVO_PIN);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // প্রারম্ভিক অবস্থায় servo বন্ধ (০°)
  myservo.write(0);
  servoAngle = 0;
  Blynk.virtualWrite(V1, servoAngle);
  
  // 100 মিলিসেকেন্ডে sensor update, যাতে responsivness থাকে
  timer.setInterval(100L, updateSensors);
}

void loop() {
  Blynk.run();
  timer.run();
}
