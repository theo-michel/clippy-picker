/*
 * Delta Robot Controller — 3x DRV8825 + 17HS19-2004S1 (NEMA 17)
 * 
 * Hardware notes:
 *   - 17HS19-2004S1: 200 steps/rev, 1.8°/step, 2 A rated
 *   - DRV8825: set the current limit pot to ~1.6–2.0 A (Vref ≈ 0.8–1.0 V)
 *   - Connect RESET and SLEEP together, pull HIGH (or tie to 5 V)
 *   - Microstepping is set by M0/M1/M2 on the DRV8825 board:
 *       M0  M1  M2  →  Microstep
 *        L   L   L  →  Full step
 *        H   L   L  →  1/2 step
 *        L   H   L  →  1/4 step
 *        H   H   L  →  1/8 step
 *        L   L   H  →  1/16 step
 *        H   H   H  →  1/32 step
 *
 * Serial protocol (115200 baud, newline-terminated):
 *   M  d1 d2 d3     – Move all motors to absolute degree positions
 *   M1 d             – Move motor 1 to absolute degree position
 *   M2 d             – Move motor 2 to absolute degree position
 *   M3 d             – Move motor 3 to absolute degree position
 *   R  d1 d2 d3      – Relative move (degrees) from current position
 *   SPD rpm           – Set max speed in RPM for all motors
 *   ACC val           – Set acceleration in RPM/s for all motors
 *   HOME              – Return all motors to 0
 *   STOP              – Decelerate to stop (respects acceleration)
 *   ESTOP             – Immediate hard stop (no deceleration)
 *   ENABLE            – Enable motor drivers (energize coils)
 *   DISABLE           – Disable motor drivers (coils free, no holding torque)
 *   POS               – Report current positions in steps
 *   STATUS            – Report MOVING or IDLE
 *   ZERO              – Set current position as the new zero reference
 *
 * Responses:
 *   OK: <message>     – Command accepted
 *   DONE              – All motors finished their current move
 *   ERR: <message>    – Bad command or parse error
 *   POS: s1 s2 s3     – Current positions in steps
 *   STATUS: IDLE/MOVING
 *
 * Requires: AccelStepper library (install via Library Manager)
 */

 #include <AccelStepper.h>

 // ===================== PIN DEFINITIONS =====================
 // Motor 1 — Tower A
 #define DIR_PIN_1   2
 #define STEP_PIN_1  4
 
 // Motor 2 — Tower B
 #define DIR_PIN_2   8
 #define STEP_PIN_2  6
 
 // Motor 3 — Tower C
 #define DIR_PIN_3   12
 #define STEP_PIN_3  11
 
 // Shared enable pin for all DRV8825 drivers (active LOW)
 // Wire all three EN pins together to this Arduino pin.
 // Set to -1 if you don't use an enable pin.
 #define ENABLE_PIN  8
 
 // ===================== MOTOR PARAMETERS =====================
 const float STEPS_PER_REV    = 200.0;   // 17HS19-2004S1 = 200 steps/rev
 const float MICROSTEPS       = 4.0;     // Match your M0/M1/M2 jumpers (1, 2, 4, 8, 16, 32)
 const float STEPS_PER_FULL_REV = STEPS_PER_REV * MICROSTEPS;
 
 // Default speed & acceleration
 float maxRPM        = 60.0;    // RPM
 float accelRPMPerS  = 120.0;   // RPM/s
 
 // ===================== STEPPER INSTANCES =====================
 AccelStepper motor1(AccelStepper::DRIVER, STEP_PIN_1, DIR_PIN_1);
 AccelStepper motor2(AccelStepper::DRIVER, STEP_PIN_2, DIR_PIN_2);
 AccelStepper motor3(AccelStepper::DRIVER, STEP_PIN_3, DIR_PIN_3);
 
 AccelStepper* motors[3] = { &motor1, &motor2, &motor3 };
 
 // ===================== STATE =====================
 String inputBuffer = "";
 bool wasMoving = false;   // Track transitions to send DONE message
 
 // ===================== HELPERS =====================
 long degreesToSteps(float degrees) {
     return (long)((degrees / 360.0) * STEPS_PER_FULL_REV);
 }
 
 float rpmToStepsPerSec(float rpm) {
     return (STEPS_PER_FULL_REV * rpm) / 60.0;
 }
 
 void applySpeedSettings() {
     float spd = rpmToStepsPerSec(maxRPM);
     float acc = rpmToStepsPerSec(accelRPMPerS);
     for (int i = 0; i < 3; i++) {
         motors[i]->setMaxSpeed(spd);
         motors[i]->setAcceleration(acc);
     }
 }
 
 bool anyMotorRunning() {
     return motor1.isRunning() || motor2.isRunning() || motor3.isRunning();
 }
 
 void enableMotors() {
     if (ENABLE_PIN >= 0) {
         digitalWrite(ENABLE_PIN, LOW);   // DRV8825 EN is active LOW
     }
 }
 
 void disableMotors() {
     if (ENABLE_PIN >= 0) {
         digitalWrite(ENABLE_PIN, HIGH);
     }
 }
 
 // ===================== STRING PARSING HELPERS =====================
 // Arduino AVR sscanf does NOT support %f — use atof() instead.
 
 // Return the substring starting after the N-th space (0-indexed).
 // e.g. nthToken("M 10 20 30", 0) → "10 20 30"
 //      nthToken("M 10 20 30", 1) → "20 30"
 String nthToken(const String& s, int n) {
     int pos = 0;
     for (int i = 0; i <= n; i++) {
         pos = s.indexOf(' ', pos);
         if (pos < 0) return "";
         pos++;  // skip the space
     }
     return s.substring(pos);
 }
 
 // Parse three space-separated floats from str into f1, f2, f3.
 // Returns true on success.
 bool parseThreeFloats(const String& str, float &f1, float &f2, float &f3) {
     // str should look like "10.5 20.3 -5.0"
     int sp1 = str.indexOf(' ');
     if (sp1 < 0) return false;
     int sp2 = str.indexOf(' ', sp1 + 1);
     if (sp2 < 0) return false;
 
     String s1 = str.substring(0, sp1);
     String s2 = str.substring(sp1 + 1, sp2);
     String s3 = str.substring(sp2 + 1);
 
     s1.trim(); s2.trim(); s3.trim();
     if (s1.length() == 0 || s2.length() == 0 || s3.length() == 0) return false;
 
     f1 = atof(s1.c_str());
     f2 = atof(s2.c_str());
     f3 = atof(s3.c_str());
     return true;
 }
 
 // ===================== COMMAND PARSER =====================
 void processCommand(String cmd) {
     cmd.trim();
 
     String upper = cmd;
     upper.toUpperCase();
 
     // ---------- M d1 d2 d3 (absolute move, all motors) ----------
     if (upper.startsWith("M ") && !upper.startsWith("M1") && !upper.startsWith("M2") && !upper.startsWith("M3")) {
         String args = cmd.substring(2);
         args.trim();
         float d1, d2, d3;
         if (parseThreeFloats(args, d1, d2, d3)) {
             motor1.moveTo(degreesToSteps(d1));
             motor2.moveTo(degreesToSteps(d2));
             motor3.moveTo(degreesToSteps(d3));
             Serial.println("OK: Moving to positions");
         } else {
             Serial.println("ERR: Expected M deg1 deg2 deg3");
         }
     }
     // ---------- M1/M2/M3 d (absolute move, single motor) ----------
     else if (upper.startsWith("M1 ") || upper.startsWith("M2 ") || upper.startsWith("M3 ")) {
         int idx = upper.charAt(1) - '1';  // 0, 1, or 2
         String arg = cmd.substring(3);
         arg.trim();
         if (arg.length() > 0) {
             float deg = atof(arg.c_str());
             motors[idx]->moveTo(degreesToSteps(deg));
             Serial.print("OK: Motor ");
             Serial.print(idx + 1);
             Serial.println(" moving");
         } else {
             Serial.println("ERR: Expected M<n> degrees");
         }
     }
     // ---------- R d1 d2 d3 (relative move, degrees) ----------
     else if (upper.startsWith("R ")) {
         String args = cmd.substring(2);
         args.trim();
         float d1, d2, d3;
         if (parseThreeFloats(args, d1, d2, d3)) {
             motor1.move(degreesToSteps(d1));
             motor2.move(degreesToSteps(d2));
             motor3.move(degreesToSteps(d3));
             Serial.println("OK: Relative move");
         } else {
             Serial.println("ERR: Expected R deg1 deg2 deg3");
         }
     }
     // ---------- SPD rpm ----------
     else if (upper.startsWith("SPD ")) {
         String arg = cmd.substring(4);
         arg.trim();
         if (arg.length() > 0) {
             float rpm = atof(arg.c_str());
             if (rpm > 0) {
                 maxRPM = rpm;
                 applySpeedSettings();
                 Serial.print("OK: Max speed set to ");
                 Serial.print(rpm);
                 Serial.println(" RPM");
             } else {
                 Serial.println("ERR: Expected SPD <positive rpm>");
             }
         } else {
             Serial.println("ERR: Expected SPD <positive rpm>");
         }
     }
     // ---------- ACC val ----------
     else if (upper.startsWith("ACC ")) {
         String arg = cmd.substring(4);
         arg.trim();
         if (arg.length() > 0) {
             float acc = atof(arg.c_str());
             if (acc > 0) {
                 accelRPMPerS = acc;
                 applySpeedSettings();
                 Serial.print("OK: Acceleration set to ");
                 Serial.print(acc);
                 Serial.println(" RPM/s");
             } else {
                 Serial.println("ERR: Expected ACC <positive value>");
             }
         } else {
             Serial.println("ERR: Expected ACC <positive value>");
         }
     }
     // ---------- HOME ----------
     else if (upper == "HOME") {
         motor1.moveTo(0);
         motor2.moveTo(0);
         motor3.moveTo(0);
         Serial.println("OK: Homing");
     }
     // ---------- STOP (decelerate) ----------
     else if (upper == "STOP") {
         motor1.stop();
         motor2.stop();
         motor3.stop();
         Serial.println("OK: Decelerating to stop");
     }
     // ---------- ESTOP (immediate) ----------
     else if (upper == "ESTOP") {
         // Set current position as target → zero remaining distance instantly
         motor1.setCurrentPosition(motor1.currentPosition());
         motor2.setCurrentPosition(motor2.currentPosition());
         motor3.setCurrentPosition(motor3.currentPosition());
         Serial.println("OK: Emergency stop");
     }
     // ---------- ENABLE / DISABLE ----------
     else if (upper == "ENABLE") {
         enableMotors();
         Serial.println("OK: Motors enabled");
     }
     else if (upper == "DISABLE") {
         disableMotors();
         Serial.println("OK: Motors disabled (no holding torque)");
     }
     // ---------- POS ----------
     else if (upper == "POS") {
         Serial.print("POS: ");
         Serial.print(motor1.currentPosition());
         Serial.print(" ");
         Serial.print(motor2.currentPosition());
         Serial.print(" ");
         Serial.println(motor3.currentPosition());
     }
     // ---------- STATUS ----------
     else if (upper == "STATUS") {
         Serial.println(anyMotorRunning() ? "STATUS: MOVING" : "STATUS: IDLE");
     }
     // ---------- ZERO ----------
     else if (upper == "ZERO") {
         for (int i = 0; i < 3; i++) {
             motors[i]->setCurrentPosition(0);
         }
         Serial.println("OK: Position zeroed");
     }
     // ---------- Unknown ----------
     else {
         Serial.println("ERR: Unknown command");
     }
 }
 
 // ===================== SETUP =====================
 void setup() {
     Serial.begin(115200);
 
     // Enable pin
     if (ENABLE_PIN >= 0) {
         pinMode(ENABLE_PIN, OUTPUT);
         enableMotors();
     }
 
     // Configure motors
     for (int i = 0; i < 3; i++) {
         motors[i]->setCurrentPosition(0);
     }
     applySpeedSettings();
 
     Serial.println("READY");
     Serial.println("Delta Robot Controller — 3x DRV8825 + 17HS19-2004S1");
     Serial.println("Commands: M d1 d2 d3 | M1/M2/M3 d | R d1 d2 d3 | HOME | STOP | POS | STATUS");
 }
 
 // ===================== LOOP =====================
 void loop() {
     // --- Serial input (non-blocking) ---
     while (Serial.available() > 0) {
         char c = Serial.read();
         if (c == '\n' || c == '\r') {
             if (inputBuffer.length() > 0) {
                 processCommand(inputBuffer);
                 inputBuffer = "";
             }
         } else {
             inputBuffer += c;
         }
     }
 
     // --- Step all motors (must be called as fast as possible) ---
     motor1.run();
     motor2.run();
     motor3.run();
 
     // --- Detect move completion and notify host ---
     bool moving = anyMotorRunning();
     if (wasMoving && !moving) {
         Serial.println("DONE");
     }
     wasMoving = moving;
 }
 