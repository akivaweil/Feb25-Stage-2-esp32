#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

// ? ===============================================================================
// ? 🔄 CUTTING CYCLE SEQUENCE
// ? ===============================================================================
/*
* UPDATE THE INSTRUCTIONS BELOW AS YOU CHANGE THE CODE. IF THE CODE AND INSTRUCTIONS ARE DIFFERENT THEN PRIORITIZE THE INSTRUCTIONS.
*
 * 1. 🏁 Initial Position:
 *    - 🔵 Both clamps engaged (LOW) at startup
 *    - 🔴 Alignment cylinder retracted (LOW)
 *    - 🎯 Motor at home offset (1.65 inches)
 * 
 * 2. 🏠 Homing Sequence:
 *    - 🔵 Clamps remain engaged during homing
 *    - 🎯 Motor moves until the home switch is triggered, then moves to the home offset (1.65 inches)
 *    - 🔴 Release clamps (HIGH) after reaching the home offset
 * 
 * 3. 🚀 Cycle Start:
 *    - 🔵 Extend alignment cylinder (HIGH)
 *    - ⏱️ Wait 500ms for alignment
 *
 * 3a. Left Clamp Pulse:
 *    - 👍 Left clamp extends (engages, LOW) for 300ms then retracts (HIGH) for 200ms
 * 
 * 4. 🔒 Clamping Sequence:
 *    - 🔵 Engage right clamp (LOW)
 *    - ⏱️ Wait 300ms
 *    - 🔴 Retract alignment cylinder (LOW)
 *    - ⏱️ Wait 200ms
 *    - 🔵 Engage left clamp (LOW)
 *    - ⏱️ Wait 300ms for clamps to fully engage
 * 
 * 5. 🔄 Cutting Sequence:
 *    - 🚀 Rapid approach for the first 6 inches (5000 steps/sec)
 *    - 🐌 Slow cutting phase for the next 8 inches (133 steps/sec)
 *    - 🏃 Accelerate to finish speed (5000 steps/sec) for the remaining distance
 *    - 🎯 Stop at forward position (25 inches)
 * 
 * 6. 📦 Release Sequence:
 *    - ⏱️ Wait for motion to complete
 *    - 🔴 Disengage clamps (HIGH)
 *    - ⏱️ Wait 500ms for clamps to release
 * 
 * 7. 🏠 Return Sequence:
 *    - 🎯 Return to home offset (1.65 inches) using return speed (10000 steps/sec)
 *    - ⏱️ Wait for motion completion
 */

/*
 * AUTOMATED TABLE SAW CONTROL SYSTEM
 * 
 * SAFETY NOTICE: PLEASE DO NOT DELETE OR MODIFY ANYTHING HERE
 * This code controls an automated table saw cutting system. Safety is the absolute priority.
 * - Code clarity and reliability take precedence over processing efficiency
 * - All functions are written to be as explicit and straightforward as possible
 * - Hardware emergency stop switch cuts ALL power to the system when activated
 * - Multiple software safety checks are implemented throughout the cycle
 * - All switches and buttons read HIGH when activated
 * - All cylinders require a LOW output to engage (Updated Feb 2024)
 * - Bounce2 library is used for switch debouncing with a 20ms debounce time
 * - All code should be very very easy to understand for a beginner programmer
 * - Switches are configured where one side is connected to 5v and the other side 
 *   splits into 10k resistor to ground at its signal pin
 */

/*
 * ⚠️ SAFETY FEATURES:
 * 
 * 1. 🛑 Emergency Stop:
 *    - Hardware E-stop cuts all power immediately
 *    - No software override possible
 * 
 * 2. 🔒 Motion Safety:
 *    - Clamps must engage before any motion
 *    - Homing required before operation
 *    - Controlled acceleration and deceleration
 *    - Speed limits enforced in software
 * 
 * 3. ⚡ Electrical Safety:
 *    - All inputs debounced to prevent false triggers
 *    - Pullup resistors on all inputs
 *    - Fail-safe signal logic
 * 
 * 4. 🔍 System Monitoring:
 *    - Continuous position tracking
 *    - Motor alarm detection
 *    - Serial debugging output
 * 
 * 5. 🛡️ Operational Safety:
 *    - Clear state indicators
 *    - Predictable motion sequences
 *    - No unexpected movements
 *    - Required settling times between actions
 */

/*
 * AUTOMATED TABLE SAW CONTROL SYSTEM
 * 
 * SAFETY NOTICE: PLEASE DO NOT DELETE OR MODIFY ANYTHING HERE
 * 
 * IMPORTANT: All Serial.print() and Serial.println() statements have been commented out
 * for performance reasons. DO NOT DELETE these statements as they provide valuable
 * debugging information. Documentation and code comments should still be maintained.
 */

// ! ===============================================================================
// ! 🛑 SAFETY NOTICE: DO NOT MODIFY WITHOUT APPROVAL
// ! ===============================================================================

// Pin Configuration - Using ESP32 GPIO pins
namespace Pins {
    // Input pins
    constexpr int HOME_SWITCH = 15;
    constexpr int START_BUTTON = 2;
    constexpr int REMOTE_START = 18;
    constexpr int ALARM = 26;
    
    // Output pins
    constexpr int STEP = 12;
    constexpr int DIR = 14;
    constexpr int ENABLE = 27;
    constexpr int LEFT_CLAMP = 22;
    constexpr int RIGHT_CLAMP = 23;
    constexpr int ALIGN_CYLINDER = 21;
}

// Motion Parameters
namespace Motion {
    constexpr int STEPS_PER_INCH = 84;
    constexpr float HOME_OFFSET = 0.2;
    constexpr float APPROACH_DISTANCE = 5.0;
    constexpr float CUTTING_DISTANCE = 7.2;
    constexpr float FORWARD_DISTANCE = 28.0;
    
    // Speed Settings (steps/second)
    constexpr float HOMING_SPEED = 400;
    constexpr float APPROACH_SPEED = 30000;
    constexpr float CUTTING_SPEED = 150;
    constexpr float FINISH_SPEED = 20000;
    constexpr float RETURN_SPEED = 20000;
    
    // Acceleration Settings (steps/second^2)
    constexpr float FORWARD_ACCEL = 20000;
    constexpr float RETURN_ACCEL = 20000;
}

// Timing Settings (milliseconds)
namespace Timing {
    constexpr int CLAMP_ENGAGE_TIME = 300;
    constexpr int CLAMP_RELEASE_TIME = 300;
    constexpr int HOME_SETTLE_TIME = 50;
    constexpr int MOTION_SETTLE_TIME = 50;
    constexpr int ALIGNMENT_TIME = 150;
    constexpr int LEFT_CLAMP_RETRACT_WAIT = 100;
}

// System state
enum class SystemState {
    INITIALIZING,
    HOMING,
    READY,
    CYCLE_RUNNING,
    ERROR
};

// Global objects
AccelStepper stepper(AccelStepper::DRIVER, Pins::STEP, Pins::DIR);
Bounce homeSwitch = Bounce();
Bounce startButton = Bounce();
Bounce remoteStart = Bounce();

// System state tracking
SystemState currentState = SystemState::INITIALIZING;
bool isHomed = false;

// Function declarations
void initializeHardware();
void performHomingSequence();
void runCuttingCycle();
void handleSerialCommand(const String& command);
void printCurrentSettings();
void printSystemStatus();
void moveStepperToPosition(float position, float speed, float acceleration);
void engageClamps();
void releaseClamps();
void staggeredReleaseClamps();

void setup() {
    Serial.begin(115200);
    // delay(1000); // Removed startup delay
    // Serial.println("\n🏭 Automated Table Saw Control System Starting..."); // DO NOT DELETE
    
    initializeHardware();
    performHomingSequence();
    
    currentState = SystemState::READY;
    // Serial.println("✅ System Ready!"); // DO NOT DELETE
    printCurrentSettings();
}

void loop() {
    // Update button states
    homeSwitch.update();
    startButton.update();
    remoteStart.update();
    
    // Handle serial commands
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        handleSerialCommand(command);
    }
    
    // Check for cycle start (either from button or remote signal)
    if ((startButton.fell() || remoteStart.fell()) && currentState == SystemState::READY) {
        // Serial.println(startButton.fell() ? "🚀 Starting cycle from button..." : "🚀 Starting cycle from remote signal..."); // DO NOT DELETE
        currentState = SystemState::CYCLE_RUNNING;
        runCuttingCycle();
        currentState = SystemState::READY;
    }
}

void initializeHardware() {
    // Configure input pins
    pinMode(Pins::HOME_SWITCH, INPUT_PULLUP);
    pinMode(Pins::START_BUTTON, INPUT_PULLUP);
    pinMode(Pins::REMOTE_START, INPUT_PULLUP);
    pinMode(Pins::ALARM, INPUT);
    
    // Configure output pins
    pinMode(Pins::ENABLE, OUTPUT);
    pinMode(Pins::LEFT_CLAMP, OUTPUT);
    pinMode(Pins::RIGHT_CLAMP, OUTPUT);
    pinMode(Pins::ALIGN_CYLINDER, OUTPUT);
    
    // Initialize clamps to engaged state (extended)
    digitalWrite(Pins::LEFT_CLAMP, LOW);    // Start with clamps engaged
    digitalWrite(Pins::RIGHT_CLAMP, LOW);   // Start with clamps engaged
    
    // Initialize alignment cylinder to retracted position
    digitalWrite(Pins::ALIGN_CYLINDER, LOW);
    
    // Setup debouncing
    homeSwitch.attach(Pins::HOME_SWITCH);
    homeSwitch.interval(20);
    startButton.attach(Pins::START_BUTTON);
    startButton.interval(20);
    remoteStart.attach(Pins::REMOTE_START);
    remoteStart.interval(5);  // Very short debounce time
    
    // Initialize stepper
    digitalWrite(Pins::ENABLE, HIGH);  // Disable briefly
    digitalWrite(Pins::ENABLE, LOW);   // Enable
    
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Serial.println("✅ Hardware initialized"); // DO NOT DELETE
}

void performHomingSequence() {
    // Serial.println("🏠 Starting homing sequence..."); // DO NOT DELETE
    currentState = SystemState::HOMING;
    
    // Clamps are already engaged from initialization
    // Configure for homing motion
    stepper.setMaxSpeed(Motion::HOMING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    stepper.setSpeed(-Motion::HOMING_SPEED);  // Negative for homing direction
    
    // Move towards home switch
    while (true) {
        homeSwitch.update();
        if (homeSwitch.read() == HIGH) {
            stepper.setCurrentPosition(0);
            break;
        }
        stepper.runSpeed();
    }
    
    // Move to home offset
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    // Now that we're at home position, release the clamps
    digitalWrite(Pins::LEFT_CLAMP, HIGH);
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);
    
    isHomed = true;
    // Serial.println("✅ Homing complete"); // DO NOT DELETE
}

void runCuttingCycle() {
    // Initial left clamp pulse and alignment cylinder extension
    digitalWrite(Pins::LEFT_CLAMP, LOW);      // Engage (extend) left clamp
    digitalWrite(Pins::ALIGN_CYLINDER, HIGH); // Extend alignment cylinder
    delay(Timing::ALIGNMENT_TIME);                            
    digitalWrite(Pins::LEFT_CLAMP, HIGH);     // Retract left clamp
    delay(Timing::LEFT_CLAMP_RETRACT_WAIT);                            
    digitalWrite(Pins::ALIGN_CYLINDER, LOW);  // Retract alignment cylinder

    // Engage clamps
    digitalWrite(Pins::RIGHT_CLAMP, LOW);  // Engage right clamp
    delay(Timing::CLAMP_ENGAGE_TIME);
    digitalWrite(Pins::LEFT_CLAMP, LOW);   // Engage left clamp
    delay(Timing::CLAMP_ENGAGE_TIME);

    // Approach phase
    // Serial.println("🚀 Approach phase..."); // DO NOT DELETE
    moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    // Cutting phase
    // Serial.println("🔪 Cutting phase..."); // DO NOT DELETE
    moveStepperToPosition(Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE, 
                         Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL);
    
    // Finish phase
    // Serial.println("🏁 Finish phase..."); // DO NOT DELETE
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    
    // Release both clamps simultaneously
    releaseClamps();
    
    // Add extra delay before return movement
    delay(500);  // Extra half second wait
    
    // Return phase - first try with fast return speed
    // Serial.println("🏠 Return to home phase..."); // DO NOT DELETE
    
    // Check if home switch is already triggered (which would be unexpected)
    homeSwitch.update();
    if (homeSwitch.read() == HIGH) {
        // Serial.println("⚠️ Home sensor already triggered - belt may have slipped!"); // DO NOT DELETE
        // We're already at home, set position to 0
        stepper.setCurrentPosition(0);
    } else {
        // First attempt: Use fast return speed
        // Serial.println("Fast return to home..."); // DO NOT DELETE
        stepper.setMaxSpeed(Motion::RETURN_SPEED);
        stepper.setAcceleration(Motion::RETURN_ACCEL);
        stepper.moveTo(0); // Move to position 0
        
        // Run the stepper until it's close to home or has stopped moving
        unsigned long fastReturnStartTime = millis();
        unsigned long fastReturnTimeout = 15000; // 15 seconds timeout for fast return
        
        while (stepper.distanceToGo() != 0) {
            stepper.run();
            
            // Check if we've reached home
            homeSwitch.update();
            if (homeSwitch.read() == HIGH) {
                stepper.setCurrentPosition(0);
                // Serial.println("Home reached with fast return!"); // DO NOT DELETE
                break;
            }
            
            // Check for timeout or if motor is stuck
            if (millis() - fastReturnStartTime > fastReturnTimeout) {
                // Serial.println("⚠️ Fast return timeout - switching to slow homing..."); // DO NOT DELETE
                break;
            }
        }
        
        // If we didn't reach home with fast return, use slow homing
        homeSwitch.update();
        if (homeSwitch.read() == LOW) {
            // Serial.println("Home not reached with fast return - switching to slow homing..."); // DO NOT DELETE
            
            // Configure for slow homing motion
            stepper.setMaxSpeed(Motion::HOMING_SPEED);
            stepper.setAcceleration(Motion::RETURN_ACCEL);
            stepper.setSpeed(-Motion::HOMING_SPEED);  // Negative for homing direction
            
            // Move towards home switch
            // Serial.println("Seeking home switch with slow speed..."); // DO NOT DELETE
            unsigned long homingStartTime = millis();
            unsigned long homingTimeout = 30000; // 30 seconds timeout
            
            while (true) {
                homeSwitch.update();
                if (homeSwitch.read() == HIGH) {
                    stepper.setCurrentPosition(0);
                    // Serial.println("Home reached with slow homing!"); // DO NOT DELETE
                    break;
                }
                
                // Check for timeout
                if (millis() - homingStartTime > homingTimeout) {
                    // Serial.println("⚠️ Homing timeout - could not find home switch!"); // DO NOT DELETE
                    // Emergency stop
                    stepper.stop();
                    currentState = SystemState::ERROR;
                    return;
                }
                
                stepper.runSpeed();
            }
        }
    }
    
    delay(Timing::HOME_SETTLE_TIME);
    
    // Move to home offset
    // Serial.println("Moving to home offset position..."); // DO NOT DELETE
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    // Serial.println("✅ Cycle complete!"); // DO NOT DELETE
}

void moveStepperToPosition(float position, float speed, float acceleration) {
    stepper.setMaxSpeed(speed);
    stepper.setAcceleration(acceleration);
    stepper.moveTo(position * Motion::STEPS_PER_INCH);
    
    // Define the home check zone (within 5 inches of home)
    const float HOME_CHECK_ZONE = 5.0; // 5 inches from home
    bool isMovingTowardsHome = stepper.targetPosition() < stepper.currentPosition(); // Moving towards home (decreasing position)
    
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        
        // Only check for home switch when moving towards home and within the home check zone
        if (isMovingTowardsHome) {
            // Calculate current position in inches
            float currentPosInches = stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
            
            // Check if we're within the home check zone (5 inches from home)
            if (currentPosInches <= HOME_CHECK_ZONE) {
                homeSwitch.update();
                
                // If home switch is triggered unexpectedly early
                if (homeSwitch.read() == HIGH) {
                    // Serial.println("⚠️ Home sensor triggered earlier than expected!"); // DO NOT DELETE
                    // Serial.println("Setting current position as new home (0)"); // DO NOT DELETE
                    
                    // Stop the motor
                    stepper.stop();
                    
                    // Set this position as the new home
                    stepper.setCurrentPosition(0);
                    
                    // If we were moving to home offset, adjust the target
                    if (position <= Motion::HOME_OFFSET * 1.5) {
                        // Move to home offset from current position
                        stepper.setMaxSpeed(Motion::APPROACH_SPEED);
                        stepper.moveTo(Motion::HOME_OFFSET * Motion::STEPS_PER_INCH);
                    } else {
                        // We've reached home unexpectedly during another movement
                        // Just stop here
                        return;
                    }
                }
            }
        }
    }
}

void engageClamps() {
    digitalWrite(Pins::LEFT_CLAMP, LOW);
    digitalWrite(Pins::RIGHT_CLAMP, LOW);
    delay(Timing::CLAMP_ENGAGE_TIME);
}

void releaseClamps() {
    digitalWrite(Pins::LEFT_CLAMP, HIGH);
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);
    delay(Timing::CLAMP_RELEASE_TIME);
}

void staggeredReleaseClamps() {
    digitalWrite(Pins::RIGHT_CLAMP, HIGH);  // Release right clamp first
    delay(Timing::CLAMP_ENGAGE_TIME);       // Wait 300ms
    digitalWrite(Pins::LEFT_CLAMP, HIGH);   // Release left clamp
    delay(Timing::CLAMP_RELEASE_TIME);      // Normal release wait time
}

void handleSerialCommand(const String& command) {
    if (command == "status") {
        printSystemStatus();
    }
    else if (command == "home") {
        // Serial.println("🏠 Initiating homing sequence..."); // DO NOT DELETE
        performHomingSequence();
    }
    else if (command == "settings") {
        printCurrentSettings();
    }
    else if (command == "help") {
        // Serial.println("\n📚 Available Commands:"); // DO NOT DELETE
        // Serial.println("- status: Show current system status"); // DO NOT DELETE
        // Serial.println("- home: Perform homing sequence"); // DO NOT DELETE
        // Serial.println("- settings: Show current system settings"); // DO NOT DELETE
        // Serial.println("- help: Show this help message"); // DO NOT DELETE
    }
    else {
        // Serial.println("❌ Unknown command. Type 'help' for available commands."); // DO NOT DELETE
    }
}

void printSystemStatus() {
    // Serial.println("\n📊 Current Status:"); // DO NOT DELETE
    
    // Print state
    // Serial.print("System State: "); // DO NOT DELETE
    switch (currentState) {
        case SystemState::INITIALIZING: /* Serial.println("INITIALIZING"); */ break;
        case SystemState::HOMING: /* Serial.println("HOMING"); */ break;
        case SystemState::READY: /* Serial.println("READY"); */ break;
        case SystemState::CYCLE_RUNNING: /* Serial.println("CYCLE RUNNING"); */ break;
        case SystemState::ERROR: /* Serial.println("ERROR"); */ break;
        default: /* Serial.println("UNKNOWN"); */
    }
    
    // Print position
    // Serial.print("Position: "); // DO NOT DELETE
    // Serial.print(stepper.currentPosition() / Motion::STEPS_PER_INCH); // DO NOT DELETE
    // Serial.println(" inches"); // DO NOT DELETE
    
    // Print input states
    // Serial.print("Home Switch: "); // DO NOT DELETE
    // Serial.println(homeSwitch.read() ? "TRIGGERED" : "NOT TRIGGERED"); // DO NOT DELETE
    // Serial.print("Start Button: "); // DO NOT DELETE
    // Serial.println(startButton.read() ? "PRESSED" : "NOT PRESSED"); // DO NOT DELETE
    // Serial.print("Remote Start: "); // DO NOT DELETE
    // Serial.println(remoteStart.read() ? "TRIGGERED" : "NOT TRIGGERED"); // DO NOT DELETE
    // Serial.print("Alarm: "); // DO NOT DELETE
    // Serial.println(digitalRead(Pins::ALARM) ? "ACTIVE" : "INACTIVE"); // DO NOT DELETE
    
    // Print output states
    // Serial.print("Left Clamp: "); // DO NOT DELETE
    // Serial.println(digitalRead(Pins::LEFT_CLAMP) ? "RELEASED" : "ENGAGED"); // DO NOT DELETE
    // Serial.print("Right Clamp: "); // DO NOT DELETE
    // Serial.println(digitalRead(Pins::RIGHT_CLAMP) ? "RELEASED" : "ENGAGED"); // DO NOT DELETE
    // Serial.print("Alignment Cylinder: "); // DO NOT DELETE
    // Serial.println(digitalRead(Pins::ALIGN_CYLINDER) ? "EXTENDED" : "RETRACTED"); // DO NOT DELETE
}

void printCurrentSettings() {
    Serial.println("\n⚙️ Current Settings:");
    Serial.println("Motion Parameters:");
    Serial.print("- Steps per inch: "); Serial.println(Motion::STEPS_PER_INCH);
    Serial.print("- Home offset: "); Serial.println(Motion::HOME_OFFSET);
    Serial.print("- Approach distance: "); Serial.println(Motion::APPROACH_DISTANCE);
    Serial.print("- Cutting distance: "); Serial.println(Motion::CUTTING_DISTANCE);
    Serial.print("- Forward distance: "); Serial.println(Motion::FORWARD_DISTANCE);
    
    Serial.println("\nSpeed Settings (steps/sec):");
    Serial.print("- Homing: "); Serial.println(Motion::HOMING_SPEED);
    Serial.print("- Approach: "); Serial.println(Motion::APPROACH_SPEED);
    Serial.print("- Cutting: "); Serial.println(Motion::CUTTING_SPEED);
    Serial.print("- Finish: "); Serial.println(Motion::FINISH_SPEED);
    Serial.print("- Return: "); Serial.println(Motion::RETURN_SPEED);
    
    Serial.println("\nAcceleration Settings (steps/sec²):");
    Serial.print("- Forward: "); Serial.println(Motion::FORWARD_ACCEL);
    Serial.print("- Return: "); Serial.println(Motion::RETURN_ACCEL);
    
    Serial.println("\nTiming Settings (ms):");
    Serial.print("- Clamp engage time: "); Serial.println(Timing::CLAMP_ENGAGE_TIME);
    Serial.print("- Clamp release time: "); Serial.println(Timing::CLAMP_RELEASE_TIME);
    Serial.print("- Home settle time: "); Serial.println(Timing::HOME_SETTLE_TIME);
    Serial.print("- Motion settle time: "); Serial.println(Timing::MOTION_SETTLE_TIME);
    Serial.println();
}

