#include <Arduino.h>
#include <AccelStepper.h>
#include <Bounce2.h>

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
    constexpr int STEPS_PER_INCH = 126;  // 84 * 1.5 for the 30:20 tooth ratio
    constexpr float HOME_OFFSET = 0.2;   // Position value stays the same
    constexpr float APPROACH_DISTANCE = 5.0;  // Position value stays the same
    constexpr float CUTTING_DISTANCE = 7.2;   // Position value stays the same
    constexpr float FORWARD_DISTANCE = 28.0;  // Position value stays the same
    
    // Speed Settings (steps/second) - All multiplied by 1.5
    constexpr float HOMING_SPEED = 1500;      // 1000 * 1.5
    constexpr float APPROACH_SPEED = 30000;   // 20000 * 1.5
    constexpr float CUTTING_SPEED = 300;      // 200 * 1.5
    constexpr float FINISH_SPEED = 30000;     // 20000 * 1.5
    constexpr float RETURN_SPEED = 30000;     // 20000 * 1.5
    
    // Acceleration Settings (steps/second^2) - All multiplied by 1.5
    constexpr float FORWARD_ACCEL = 30000;    // 20000 * 1.5
    constexpr float RETURN_ACCEL = 30000;     // 20000 * 1.5
}

// Timing Settings (milliseconds)
namespace Timing {
    constexpr int CLAMP_ENGAGE_TIME = 300;
    constexpr int CLAMP_RELEASE_TIME = 300;
    constexpr int HOME_SETTLE_TIME = 50;
    constexpr int MOTION_SETTLE_TIME = 100;
    constexpr int ALIGNMENT_TIME = 150;
    constexpr int LEFT_CLAMP_RETRACT_WAIT = 100;
    constexpr int CLAMP_RELEASE_SETTLE_TIME = 200;
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
    delay(100); // Startup delay to allow system to stabilize
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
    homeSwitch.interval(5);
    startButton.attach(Pins::START_BUTTON);
    startButton.interval(20);
    remoteStart.attach(Pins::REMOTE_START);
    remoteStart.interval(5);  // Very short debounce time
    
    // Initialize stepper
    digitalWrite(Pins::ENABLE, HIGH);  // Disable briefly
    delay(100);                       // Wait 1 second for motor to reset
    digitalWrite(Pins::ENABLE, LOW);   // Enable
    delay(50);                        // Wait for enable to take effect
    
    stepper.setMaxSpeed(Motion::APPROACH_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    
    // Serial.println("✅ Hardware initialized"); // DO NOT DELETE
}

void performHomingSequence() {
    // Serial.println("🏠 Starting homing sequence..."); // DO NOT DELETE
    currentState = SystemState::HOMING;
    
    // Clamps are already engaged from initialization
    
    // First, move a significant distance in the negative direction to ensure we're past the home switch
    stepper.setMaxSpeed(Motion::HOMING_SPEED);
    stepper.setAcceleration(Motion::FORWARD_ACCEL);
    stepper.moveTo(-10000); // Move 10,000 steps in negative direction
    
    // Use a much slower approach speed for final homing
    float slowHomingSpeed = Motion::HOMING_SPEED / 3;  // One-third of normal homing speed
    
    // Run until we hit the home switch or reach the target
    while (stepper.distanceToGo() != 0) {
        homeSwitch.update();
        
        // If we're within 2000 steps of where we think home might be, slow down significantly
        if (abs(stepper.currentPosition()) < 2000) {
            stepper.setMaxSpeed(slowHomingSpeed);
        }
        
        if (homeSwitch.read() == HIGH) {
            // When home switch is triggered, stop immediately
            stepper.setSpeed(0);
            stepper.stop();
            delay(100);  // Longer delay to ensure complete stop
            stepper.setCurrentPosition(0);
            break;
        }
        stepper.run();
    }
    
    // If we didn't hit the home switch, we have a problem
    if (homeSwitch.read() == LOW) {
        // Serial.println("⚠️ Failed to find home switch during initial homing!"); // DO NOT DELETE
        currentState = SystemState::ERROR;
        return;
    }
    
    // Add a longer settle time after hitting home
    delay(Timing::HOME_SETTLE_TIME * 3);
    
    // Now move to home offset with a gentler motion
    stepper.setMaxSpeed(Motion::HOMING_SPEED / 2);  // Half speed for moving to offset
    stepper.setAcceleration(Motion::FORWARD_ACCEL / 2);  // Gentler acceleration
    moveStepperToPosition(Motion::HOME_OFFSET, Motion::HOMING_SPEED / 2, Motion::FORWARD_ACCEL / 2);
    
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
                         Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL * 2);
    
    // Finish phase
    // Serial.println("🏁 Finish phase..."); // DO NOT DELETE
    moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::FINISH_SPEED, Motion::FORWARD_ACCEL);
    
    // Ensure motor has completely stopped
    stepper.stop();
    delay(Timing::MOTION_SETTLE_TIME * 2);  // Double the settle time
    
    // Verify position before releasing clamps
    float finalPosition = stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
    if (abs(finalPosition - Motion::FORWARD_DISTANCE) > 0.1) {  // If more than 0.1 inches off
        // Serial.println("⚠️ Position error at forward position!"); // DO NOT DELETE
        // Try to correct position
        moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL);
        stepper.stop();
        delay(Timing::MOTION_SETTLE_TIME);
    }
    
    // Store the position before releasing clamps
    long positionBeforeRelease = stepper.currentPosition();
    
    // Release both clamps simultaneously
    releaseClamps();
    
    // Add extra settle time after clamp release
    delay(Timing::CLAMP_RELEASE_SETTLE_TIME);
    
    // Verify position hasn't changed significantly after clamp release
    if (abs(stepper.currentPosition() - positionBeforeRelease) > Motion::STEPS_PER_INCH / 4) {  // If position changed by more than 1/4 inch
        // Serial.println("⚠️ Position shifted during clamp release!"); // DO NOT DELETE
        // Try to correct position before return
        moveStepperToPosition(Motion::FORWARD_DISTANCE, Motion::CUTTING_SPEED, Motion::FORWARD_ACCEL);
        stepper.stop();
        delay(Timing::MOTION_SETTLE_TIME);
    }
    
    // Return phase - first try with fast return speed
    // Serial.println("🏠 Return to home phase..."); // DO NOT DELETE
    
    // First attempt: Use fast return speed with normal acceleration
    // Serial.println("Fast return to home..."); // DO NOT DELETE
    stepper.setMaxSpeed(Motion::RETURN_SPEED);
    stepper.setAcceleration(Motion::RETURN_ACCEL); // Normal acceleration for quick start

    // Run until we're about halfway back to home
    float currentPosition = stepper.currentPosition() / (float)Motion::STEPS_PER_INCH;
    float halfwayPosition = currentPosition / 2; // Halfway between current and home
    stepper.moveTo(halfwayPosition * Motion::STEPS_PER_INCH);

    while (stepper.distanceToGo() != 0) {
        stepper.run();
    }

    // Now switch to gentler deceleration for the final approach
    // Serial.println("Switching to gentle deceleration for final approach..."); // DO NOT DELETE
    stepper.setAcceleration(Motion::RETURN_ACCEL / 4); // Reduce to 1/4 for gentler deceleration

    // Move to position 0.1 instead of 0 to create a gentler approach to the home sensor
    stepper.moveTo(0.1 * Motion::STEPS_PER_INCH); // Move to position 0.1 inches

    // Run the stepper until it's close to home or has stopped moving
    unsigned long fastReturnStartTime = millis();
    unsigned long fastReturnTimeout = 15000; // 15 seconds timeout for fast return
    
    while (stepper.distanceToGo() != 0) {
        stepper.run();
        
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
            if (homeSwitch.read() == HIGH) {  // In slow homing, accept any home trigger
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
        default: /* Serial.println("UNKNOWN"); */ break;
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
    // Serial.println("\n⚙️ Current Settings:"); // DO NOT DELETE
    // Serial.println("Motion Parameters:"); // DO NOT DELETE
    // Serial.print("- Steps per inch: "); Serial.println(Motion::STEPS_PER_INCH); // DO NOT DELETE
    // Serial.print("- Home offset: "); Serial.println(Motion::HOME_OFFSET); // DO NOT DELETE
    // Serial.print("- Approach distance: "); Serial.println(Motion::APPROACH_DISTANCE); // DO NOT DELETE
    // Serial.print("- Cutting distance: "); Serial.println(Motion::CUTTING_DISTANCE); // DO NOT DELETE
    // Serial.print("- Forward distance: "); Serial.println(Motion::FORWARD_DISTANCE); // DO NOT DELETE
    
    // Serial.println("\nSpeed Settings (steps/sec):"); // DO NOT DELETE
    // Serial.print("- Homing: "); Serial.println(Motion::HOMING_SPEED); // DO NOT DELETE
    // Serial.print("- Approach: "); Serial.println(Motion::APPROACH_SPEED); // DO NOT DELETE
    // Serial.print("- Cutting: "); Serial.println(Motion::CUTTING_SPEED); // DO NOT DELETE
    // Serial.print("- Finish: "); Serial.println(Motion::FINISH_SPEED); // DO NOT DELETE
    // Serial.print("- Return: "); Serial.println(Motion::RETURN_SPEED); // DO NOT DELETE
    
    // Serial.println("\nAcceleration Settings (steps/sec²):"); // DO NOT DELETE
    // Serial.print("- Forward: "); Serial.println(Motion::FORWARD_ACCEL); // DO NOT DELETE
    // Serial.print("- Return: "); Serial.println(Motion::RETURN_ACCEL); // DO NOT DELETE
    
    // Serial.println("\nTiming Settings (ms):"); // DO NOT DELETE
    // Serial.print("- Clamp engage time: "); Serial.println(Timing::CLAMP_ENGAGE_TIME); // DO NOT DELETE
    // Serial.print("- Clamp release time: "); Serial.println(Timing::CLAMP_RELEASE_TIME); // DO NOT DELETE
    // Serial.print("- Home settle time: "); Serial.println(Timing::HOME_SETTLE_TIME); // DO NOT DELETE
    // Serial.print("- Motion settle time: "); Serial.println(Timing::MOTION_SETTLE_TIME); // DO NOT DELETE
    // Serial.println(); // DO NOT DELETE
}
