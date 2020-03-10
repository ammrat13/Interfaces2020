#include <AccelStepper.h>


// For serial
#define SERIAL_TIMEOUT (1)

// Stepper motor constants
#define STEPPER_MAX_SPEED (300)
#define STEPPER_MAX_ACCEL (STEPPER_MAX_SPEED*STEPPER_MAX_SPEED)

// For the enabled button
#define ENABLED_DEBOUNCE_T (200)


// Pin setup
#define PIN_EN_DETECT (3)
#define PIN_EN_LED (4)

#define PIN_ST0 (8)
#define PIN_ST1 (9)
#define PIN_ST2 (10)
#define PIN_ST3 (11)


boolean enabled = false;
unsigned long enabledDebounce = 0;

AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_ST0, PIN_ST1, PIN_ST2, PIN_ST3, true);


/* Handlers to take care of interrupts when the button is pushed */

void enableRisingHandler() {
    // Just log the debounce time if it changes in the wrong direction
    enabledDebounce = millis();
}

void enableFallingHandler() {
    // Check for debounce time
    if(millis() > enabledDebounce + ENABLED_DEBOUNCE_T) {
        // Toggle enable and change LED
        enabled = !enabled;
        digitalWrite(PIN_EN_LED, enabled ? HIGH : LOW);
    }

    // Remember to log the debounce time
    enabledDebounce = millis();
}


void setup()  {
    Serial.begin(115200);
    Serial.setTimeout(SERIAL_TIMEOUT);
    
    // Create the handler for enable
    pinMode(PIN_EN_DETECT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_EN_DETECT), enableRisingHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_EN_DETECT), enableFallingHandler, FALLING);

    // For the enabled LED
    pinMode(PIN_EN_LED, OUTPUT);
    digitalWrite(PIN_EN_LED, LOW);

    // Stepper setup
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_MAX_ACCEL);

    // Announce our presence
    Serial.println("stepper");
}

void loop() {
    char buf[64];
    if(Serial.readBytesUntil('\n', buf, 64)) {
        // We only ever get the target position on Serial
        // We know what we need to so
        stepper.moveTo(atoi(buf));

        // Print out the result in format <DTG,Enabled>
        Serial.print("<");
        Serial.print(stepper.distanceToGo());
        Serial.print(",");
        Serial.print(enabled ? 1 : 0);
        Serial.print(">");
        Serial.println();
    }

    // Don't forget to run the stepper
    stepper.run();
}
