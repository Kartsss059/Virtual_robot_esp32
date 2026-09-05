/**
 * ESP32 Hardware-in-the-Loop (HIL) Simulation Firmware
 * 
 * Target: ESP32 (Arduino framework / PlatformIO)
 * Description:
 *   Receives robot telemetry (pose x, y, theta, target waypoint, obstacle distance)
 *   from a Pygame-based simulation via Serial UART.
 *   Executes waypoint tracking with heading PID / differential drive control
 *   and outputs wheel velocity/PWM commands back to the simulator.
 */

#include <Arduino.h>
#include <math.h>

// Onboard LED pin (GPIO 2 is standard on ESP32 DevKit)
#define LED_PIN 2

// Serial Baud Rate
#define SERIAL_BAUD 115200

// Communication Watchdog Timeout (milliseconds)
#define TIMEOUT_MS 1000

// Differential Drive Parameters
const float WHEEL_BASE = 40.0f;     // Wheel baseline in simulation units (pixels)
const float MAX_SPEED = 120.0f;     // Max linear speed (units/sec)
const float WAYPOINT_TOLERANCE = 15.0f; // Distance threshold to stop at waypoint

// Control Gains (PID for heading tracking)
const float KP_HEADING = 80.0f;
const float KD_HEADING = 10.0f;

// State Variables
unsigned long lastPacketTime = 0;
float prevHeadingError = 0.0f;
bool ledState = false;

// Normalize angle to range [-PI, PI]
float normalizeAngle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Process incoming telemetry packet and compute motor commands
void processTelemetry(float x, float y, float theta, float targetX, float targetY, float distFront) {
    // 1. Calculate distance and angle to target
    float dx = targetX - x;
    float dy = targetY - y;
    float distToTarget = sqrtf(dx * dx + dy * dy);
    float desiredHeading = atan2f(dy, dx);

    // 2. Heading error
    float headingError = normalizeAngle(desiredHeading - theta);
    float headingDerivative = headingError - prevHeadingError;
    prevHeadingError = headingError;

    // 3. Control logic
    float linearSpeed = 0.0f;
    float angularSpeed = 0.0f;

    if (distToTarget > WAYPOINT_TOLERANCE) {
        // Obstacle avoidance reflex if obstacle is close in front
        if (distFront > 0.0f && distFront < 60.0f) {
            // Back up and turn away from obstacle
            linearSpeed = -20.0f;
            angularSpeed = 60.0f;
        } else {
            // Proportional heading control
            angularSpeed = (KP_HEADING * headingError) + (KD_HEADING * headingDerivative);

            // Scale linear velocity based on alignment with the target
            float alignmentFactor = cosf(headingError);
            if (alignmentFactor > 0.0f) {
                linearSpeed = MAX_SPEED * alignmentFactor;
                // Slow down when approaching target
                if (distToTarget < 80.0f) {
                    linearSpeed *= (distToTarget / 80.0f);
                }
            } else {
                // If facing away, prioritize pure rotation
                linearSpeed = 0.0f;
            }
        }
    } else {
        // Reached target waypoint
        linearSpeed = 0.0f;
        angularSpeed = 0.0f;
    }

    // 4. Differential drive kinematics: convert (v, w) to (v_left, v_right)
    float leftCmd = linearSpeed - (angularSpeed * WHEEL_BASE / 2.0f);
    float rightCmd = linearSpeed + (angularSpeed * WHEEL_BASE / 2.0f);

    // Clamp wheel speeds to [-MAX_SPEED, MAX_SPEED]
    leftCmd = constrain(leftCmd, -MAX_SPEED, MAX_SPEED);
    rightCmd = constrain(rightCmd, -MAX_SPEED, MAX_SPEED);

    // 5. Send command back to simulation
    // Format: CMD,<left_speed>,<right_speed>\n
    Serial.print("CMD,");
    Serial.print(leftCmd, 2);
    Serial.print(",");
    Serial.println(rightCmd, 2);

    // Toggle onboard LED on valid packet
    ledState = !ledState;
    digitalWrite(LED_PIN, ledState ? HIGH : LOW);
}

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 2000) {
        // Wait for serial monitor connection
    }

    Serial.println("STATUS,ESP32_HIL_READY");
    lastPacketTime = millis();
}

void loop() {
    // Read incoming messages line by line
    if (Serial.available() > 0) {
        String line = Serial.readStringUntil('\n');
        line.trim();

        if (line.startsWith("TELEM,")) {
            // Packet format: TELEM,x,y,theta,targetX,targetY,distFront
            int idx1 = line.indexOf(',', 6);
            int idx2 = line.indexOf(',', idx1 + 1);
            int idx3 = line.indexOf(',', idx2 + 1);
            int idx4 = line.indexOf(',', idx3 + 1);
            int idx5 = line.indexOf(',', idx4 + 1);

            if (idx1 > 0 && idx2 > 0 && idx3 > 0 && idx4 > 0) {
                float x = line.substring(6, idx1).toFloat();
                float y = line.substring(idx1 + 1, idx2).toFloat();
                float theta = line.substring(idx2 + 1, idx3).toFloat();
                float targetX = line.substring(idx3 + 1, idx4).toFloat();
                float targetY = (idx5 > 0) ? line.substring(idx4 + 1, idx5).toFloat() 
                                           : line.substring(idx4 + 1).toFloat();
                float distFront = (idx5 > 0) ? line.substring(idx5 + 1).toFloat() : 999.0f;

                lastPacketTime = millis();
                processTelemetry(x, y, theta, targetX, targetY, distFront);
            }
        } else if (line.equals("PING")) {
            Serial.println("PONG");
            lastPacketTime = millis();
        }
    }

    // Safety watchdog: stop motors if serial telemetry stream times out
    if (millis() - lastPacketTime > TIMEOUT_MS) {
        Serial.println("CMD,0.00,0.00");
        digitalWrite(LED_PIN, LOW);
        delay(100);
    }
}

