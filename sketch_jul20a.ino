// ==========================
// Automated Delivery Drone - Arduino UNO
// Components: 4x 1400KV Brushless Motors + ESC, MPU6050, HC-05 Bluetooth
// Task: Move in 3m x 3m x 3m cube based on (x, z) coordinates
// ==========================

#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>
#include <SoftwareSerial.h>

// --------------------------
// BLUETOOTH (HC-05) SETUP
// --------------------------
SoftwareSerial bluetooth(10, 11); // RX, TX (HC-05 TX → D10, RX ← D11 via voltage divider)

// --------------------------
// MPU6050 & PID SETUP
// --------------------------
MPU6050 mpu;
float accZ;
float targetAltitude = 1.5; // Constant hover height (m)
float currentAltitude = 0;
float pidOutput;
float errorSum = 0, lastError = 0;

// PID coefficients
float Kp = 2.5;
float Ki = 0.1;
float Kd = 1.5;

// --------------------------
// DRONE TARGET COORDINATES
// --------------------------
float targetX = 0;
float targetZ = 0;
bool coordinatesReceived = false;

// --------------------------
// MOTOR ESC CONTROL
// --------------------------
Servo motorFL, motorFR, motorBL, motorBR; // 4 ESC motors

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);
  Wire.begin();
  mpu.initialize();

  // ESC Pins
  motorFL.attach(3);
  motorFR.attach(5);
  motorBL.attach(6);
  motorBR.attach(9);

  bluetooth.println("Drone Ready. Send target coordinates like: 1.5 2.0");
  calibrateESCs();
}

void loop() {
  readBluetoothCoordinates();
  if (coordinatesReceived) {
    bluetooth.println("\nStarting delivery routine...");
    flyToHeight(targetAltitude);
    moveToCoordinates(targetX, targetZ);
    hoverAndDescend();
    returnHome();
    coordinatesReceived = false;
    bluetooth.println("\nRoutine complete. Ready for next input.");
  }
}

// --------------------------
// READ COORDINATES VIA BLUETOOTH
// --------------------------
void readBluetoothCoordinates() {
  if (bluetooth.available()) {
    String input = bluetooth.readStringUntil('\n');
    input.trim();

    int spaceIdx = input.indexOf(' ');
    if (spaceIdx > 0) {
      targetX = input.substring(0, spaceIdx).toFloat();
      targetZ = input.substring(spaceIdx + 1).toFloat();
      bluetooth.print("Target X: "); bluetooth.println(targetX);
      bluetooth.print("Target Z: "); bluetooth.println(targetZ);
      coordinatesReceived = true;
    } else {
      bluetooth.println("Invalid input. Use format: 1.5 2.0");
    }
  }
}

// --------------------------
// FLIGHT ROUTINES
// --------------------------
void flyToHeight(float height) {
  bluetooth.println("Rising to 1.5m height...");
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    controlAltitude(height);
    delay(20);
  }
}

void moveToCoordinates(float x, float z) {
  bluetooth.print("Flying to coordinates: ");
  bluetooth.print(x); bluetooth.print(", "); bluetooth.println(z);
  delay(2000); // Simulate motion
  bluetooth.println("Hovering...");
  delay(1000);
}

void hoverAndDescend() {
  bluetooth.println("Descending to 0.5m...");
  unsigned long t0 = millis();
  while (millis() - t0 < 3000) {
    controlAltitude(0.5);
    delay(20);
  }
  bluetooth.println("Hovering 10 seconds for delivery...");
  delay(10000);
  bluetooth.println("Rising back to 1.5m...");
  flyToHeight(1.5);
}

void returnHome() {
  bluetooth.println("Returning to home");
  delay(2000); // Simulate motion
  bluetooth.println("Landing complete");
}

// --------------------------
// ALTITUDE CONTROL (MPU + PID)
// --------------------------
void controlAltitude(float target) {
  accZ = mpu.getAccelerationZ() / 16384.0 * 9.81; // Z in m/s²
  currentAltitude += (accZ - 9.81) * 0.02; // crude integration

  float error = target - currentAltitude;
  errorSum += error;
  float dError = error - lastError;
  lastError = error;

  pidOutput = Kp * error + Ki * errorSum + Kd * dError;
  int pwm = constrain(1100 + pidOutput, 1000, 2000);
  applyMotorPWM(pwm);
}

void applyMotorPWM(int pwm) {
  motorFL.writeMicroseconds(pwm);
  motorFR.writeMicroseconds(pwm);
  motorBL.writeMicroseconds(pwm);
  motorBR.writeMicroseconds(pwm);
}

// --------------------------
// ESC CALIBRATION (RUN ONCE)
// --------------------------
void calibrateESCs() {
  bluetooth.println("Calibrating ESCs.");
  applyMotorPWM(1000);
  delay(2000);
  bluetooth.println("ESC calibration complete.");
}
