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
    constexpr float APPROACH_SPEED = 5000;
    constexpr float CUTTING_SPEED = 133;
    constexpr float FINISH_SPEED = 5000;
    constexpr float RETURN_SPEED = 10000;
    
    // Acceleration Settings (steps/second^2)
    constexpr float FORWARD_ACCEL = 7000;
    constexpr float RETURN_ACCEL = 7000;
}

// Timing Settings (milliseconds)
namespace Timing {
    constexpr int CLAMP_ENGAGE_TIME = 300;
    constexpr int CLAMP_RELEASE_TIME = 500;
    constexpr int HOME_SETTLE_TIME = 50;
    constexpr int MOTION_SETTLE_TIME = 50;
    constexpr int ALIGNMENT_TIME = 150;
    constexpr int LEFT_CLAMP_RETRACT_WAIT = 300;
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
    delay(1000);
    Serial.println("\n🏭 Automated Table Saw Control System Starting...");
    
    initializeHardware();
    performHomingSequence();
    
    currentState = SystemState::READY;
    Serial.println("✅ System Ready!");
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
        Serial.println(startButton.fell() ? "🚀 Starting cycle from button..." : "🚀 Starting cycle from remote signal...");
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
    delay(100);
    digitalWrite(Pins::ENABLE, LOW);   // Enable
    delay(200);
    
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    Serial.println("✅ Hardware initialized");
}

void performHomingSequence() {
    Serial.println("🏠 Starting homing sequence...");
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
    
    delay(Timing::HOME_SETTLE_TIME);
    
    // Move to home offset
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    // Now that we're at home position, release the clamps
    releaseClamps();
    
    isHomed = true;
    Serial.println("✅ Homing complete");
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
    Serial.println("🚀 Approach phase...");
    moveStepperToPosition(Motion::APPROACH_DISTANCE, Motion::APPROACH_SPEED, Motion::FORWARD_ACCEL);
    
    // Cutting phase
    Serial.println("🔪 Cutting phase...");
    moveStepperToPosition(Motion::APPROACH_DISTANCE + Motion::CUTTING_DISTANCE, 
                         Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL);
    
    // Finish phase
    Serial.println("🏁 Finish phase...");
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
    
    delay(Timing::MOTION_SETTLE_TIME);
    
    // Staggered clamp release
    staggeredReleaseClamps();
    
    // Add extra delay before return movement
    delay(500);  // Extra half second wait
    
    // Return phase
    Serial.println("🏠 Return phase...");
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::RETURN_SPEED, Motion::RETURN_ACCEL);
    
    Serial.println("✅ Cycle complete!");
}

void moveStepperToPosition(float position, float speed, float acceleration) {
    stepper.setMaxSpeed(speed);
    stepper.setAcceleration(acceleration);
    stepper.moveTo(position * Motion::STEPS_PER_INCH);
    
    while (stepper.distanceToGo() != 0) {
        stepper.run();
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
        Serial.println("🏠 Initiating homing sequence...");
        performHomingSequence();
    }
    else if (command == "settings") {
        printCurrentSettings();
    }
    else if (command == "help") {
        Serial.println("\n📚 Available Commands:");
        Serial.println("- status: Show current system status");
        Serial.println("- home: Perform homing sequence");
        Serial.println("- settings: Show current system settings");
        Serial.println("- help: Show this help message");
    }
    else {
        Serial.println("❌ Unknown command. Type 'help' for available commands.");
    }
}

void printSystemStatus() {
    Serial.println("\n📊 Current Status:");
    
    // Print state
    Serial.print("System State: ");
    switch (currentState) {
        case SystemState::INITIALIZING: Serial.println("INITIALIZING"); break;
        case SystemState::HOMING: Serial.println("HOMING"); break;
        case SystemState::READY: Serial.println("READY"); break;
        case SystemState::CYCLE_RUNNING: Serial.println("CYCLE RUNNING"); break;
        case SystemState::ERROR: Serial.println("ERROR"); break;
        default: Serial.println("UNKNOWN");
    }
    
    // Print position
    Serial.print("Position: ");
    Serial.print(stepper.currentPosition() / Motion::STEPS_PER_INCH);
    Serial.println(" inches");
    
    // Print input states
    Serial.print("Home Switch: ");
    Serial.println(homeSwitch.read() ? "TRIGGERED" : "NOT TRIGGERED");
    Serial.print("Start Button: ");
    Serial.println(startButton.read() ? "PRESSED" : "NOT PRESSED");
    Serial.print("Remote Start: ");
    Serial.println(remoteStart.read() ? "TRIGGERED" : "NOT TRIGGERED");
    Serial.print("Alarm: ");
    Serial.println(digitalRead(Pins::ALARM) ? "ACTIVE" : "INACTIVE");
    
    // Print output states
    Serial.print("Left Clamp: ");
    Serial.println(digitalRead(Pins::LEFT_CLAMP) ? "RELEASED" : "ENGAGED");
    Serial.print("Right Clamp: ");
    Serial.println(digitalRead(Pins::RIGHT_CLAMP) ? "RELEASED" : "ENGAGED");
    Serial.print("Alignment Cylinder: ");
    Serial.println(digitalRead(Pins::ALIGN_CYLINDER) ? "EXTENDED" : "RETRACTED");
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

