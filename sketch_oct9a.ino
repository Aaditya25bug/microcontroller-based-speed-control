#include <LiquidCrystal.h>
#include <PID_v1.h>

#define ENC_A_PIN 32
#define ENC_B_PIN 33

#define PWM_A_PIN 14 
#define IN1_PIN 27
#define IN2_PIN 26

#define CURRENT_SENSOR 34 

const int rs = 19, en = 23, d4 = 18, d5 = 17, d6 = 16, d7 = 15;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

const int PPR = 600; 
volatile long pulseCount = 0;
double rpm = 0;
unsigned long lastRpmTime = 0;
long lastPulseCount = 0;

double setpointRPM = 100.0; // speed in rpm
double pidOutput = 0;

double Kp = 0.6, Ki = 0.8, Kd = 0.1;
PID motorPID(&rpm, &pidOutput, &setpointRPM, Kp, Ki, Kd, DIRECT);

const float ACS_SENSITIVITY = 185.0; 
const float VREF = 3.3; 
int zeroCurrentAdc = 1860; 
float current_mA = 0;

unsigned long lastDisplayTime = 0;

void IRAM_ATTR isr_encoder() {
  if (digitalRead(ENC_B_PIN) == HIGH) {
    pulseCount++; 
  } else {
    pulseCount--; 
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(PWM_A_PIN, OUTPUT);
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);
  pinMode(ENC_A_PIN, INPUT_PULLUP);
  pinMode(ENC_B_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A_PIN), isr_encoder, CHANGE);
  Serial.println("calibrating current sensor, ensure no load on motor.");
  long adc_sum = 0;
  for (int i = 0; i < 1000; i++) {
    adc_sum += analogRead(CURRENT_SENSOR);
    delay(1);
  }
  zeroCurrentAdc = adc_sum / 1000;
  Serial.print("Zero Current ADC Value: ");
  Serial.println(zeroCurrentAdc);
  delay(2000);
  motorPID.SetMode(AUTOMATIC);
  motorPID.SetOutputLimits(0, 255);
  motorPID.SetSampleTime(10);
  //starting lcd display
  lcd.begin(16, 2); 
  lcd.print("Motor Control");
  lcd.setCursor(0, 1);
  lcd.print("System Online");
  delay(2000);
}

void loop() {
  unsigned long currentTime = millis();

  if (currentTime - lastRpmTime >= 50) {
    long currentPulseCount;
    noInterrupts();
    currentPulseCount = pulseCount;
    interrupts();
    double deltaPulses = currentPulseCount - lastPulseCount;
    double deltaTime = (currentTime - lastRpmTime) / 1000.0;
    rpm = (deltaPulses / PPR) / deltaTime * 60.0;
    lastPulseCount = currentPulseCount;
    lastRpmTime = currentTime;
  }
  motorPID.Compute();
  setMotorSpeed(pidOutput);

  int currentAdc = analogRead(CURRENT_SENSOR);
  float voltage = (currentAdc - zeroCurrentAdc) * (VREF / 4095.0);
  current_mA = (voltage * 1000) / ACS_SENSITIVITY;

  if (currentTime - lastDisplayTime >= 250) {
    updateDisplay();
    lastDisplayTime = currentTime;
  }
}
void setMotorSpeed(int pwmValue) {
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  analogWrite(PWM_A_PIN, pwmValue);
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("S:");
  lcd.print(setpointRPM, 0);
  lcd.print(" A:");
  lcd.print(rpm, 0);
  lcd.print(" RPM");

  lcd.setCursor(0, 1);
  lcd.print("I:");
  lcd.print(current_mA, 0);
  lcd.print("mA PWM:");
  lcd.print((int)pidOutput);
}
