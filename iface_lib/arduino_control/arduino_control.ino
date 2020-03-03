#include <PID_v1.h>
#include <AccelStepper.h>


// For serial
#define SERIAL_TIMEOUT (1)

// Drive motor constants
#define MOTOR_KP (.035)             // These constants are omega to omega
#define MOTOR_KI (.0003)
#define MOTOR_KD (.0005)
#define PULSE_TO_FREQ (2.4886e-3)   // See ArduinoControl.ino for explanation
#define W_TO_PWM (300)              // Approximation
#define PULSE_IN_TOUT (50000)       // For motor speed calculation

// Stepper motor constants
#define STEPPER_MAX_SPEED (150)
#define STEPPER_MAX_ACCEL (STEPPER_MAX_SPEED*STEPPER_MAX_SPEED)
#define NUM_STEPS_PER_CYCLE (10)

// For the enabled button
#define ENABLED_DEBOUNCE_T (200)


// Pin setup
// Depends on the board, see Google Sheet for details
// Detection from https://electronics.stackexchange.com/questions/58386/how-can-i-detect-which-arduino-board-or-which-controller-in-software/280379
#if defined(ARDUINO_AVR_MEGA2560)
    #define PIN_EN_DETECT (3)
    #define PIN_EN_LED (49)

    #define PIN_RPWM (4)
    #define PIN_LPWM (5)
    #define PIN_CRPWM (6)
    #define PIN_CLPWM (7)

    #define PIN_RDIR1 (10)
    #define PIN_RDIR2 (11)
    #define PIN_LDIR1 (12)
    #define PIN_LDIR2 (13)

    #define PIN_RENC1 (22)
    #define PIN_RENC2 (23)
    #define PIN_LENC1 (24)
    #define PIN_LENC2 (25)
    #define PIN_CRENC (26)
    #define PIN_CLENC (27)

    #define PIN_ST0 (50)
    #define PIN_ST1 (51)
    #define PIN_ST2 (52)
    #define PIN_ST3 (53)
#elif defined(ARDUINO_AVR_UNO)
    #define PIN_EN_DETECT (2)
    #define PIN_EN_LED (A5)

    #define PIN_RPWM (9)
    #define PIN_LPWM (10)
    #define PIN_CPWM (11)

    #define PIN_RDIR1 (A0)
    #define PIN_RDIR2 (A1)
    #define PIN_LDIR1 (A2)
    #define PIN_LDIR2 (A3)

    #define PIN_RENC1 (12)
    #define PIN_RENC2 (13)
    #define PIN_LENC1 (7)
    #define PIN_LENC2 (8)

    #define PIN_ST0 (3)
    #define PIN_ST1 (4)
    #define PIN_ST2 (5)
    #define PIN_ST3 (6)
#else
    #error "Only supported platforms are Uno and Mega"
#endif


boolean enabled = false;
unsigned long enabledDebounce = 0;

AccelStepper stepper(AccelStepper::FULL4WIRE, PIN_ST0, PIN_ST1, PIN_ST2, PIN_ST3, true);

double wTargR;
double wR;
double wPidR;
PID pidR(&wR, &wPidR, &wTargR, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);

double wTargL;
double wL;
double wPidL;
PID pidL(&wL, &wPidL, &wTargL, MOTOR_KP, MOTOR_KI, MOTOR_KD, DIRECT);

int pwmCR;
int pwmCL;
double wCR;
double wCL;


void split(String results[], int len, String input, char spChar) {
    String temp = input;
    for (int i = 0; i < len; i++) {
        int idx = temp.indexOf(spChar);
        results[i] = temp.substring(0,idx);
        temp = temp.substring(idx+1);
    }
}


// We have already computed `enabled` with interrupts, so just return
void Enabled() {
    Serial.println(enabled);
}


// Set the target velocities of the motors
// Parameters are right then left
void SetTargVels(String data) {
    // Get the parameters
    String ps[4];
    split(ps, 4, data, '%');
    // Write them
    wTargR = ps[0].toDouble();
    wTargL = ps[1].toDouble();
    // Collectors update immediately because no PID
    pwmCR = ps[2].toInt();
    pwmCL = ps[3].toInt();
}

// Also have a getter
void GetVels() {
    Serial.print(wR, 15);
    Serial.print(",");
    Serial.print(wL, 15);
    Serial.print(",");
    Serial.print(wCR, 15);
    Serial.print(",");
    Serial.print(wCL, 15);
    Serial.println("");
}


// We only need minimal stepper handlers
// Just get to the target position asynchronously
void STMove(String data){
    stepper.move(data.toInt());
}
void STDistanceToGo(){
    Serial.println(stepper.distanceToGo());
}


void SerialParser(void) {

    // Only do processing if there is stuff to read
    // Prevents unneeded use of the serial bus
    char readChar[64];
    if(Serial.readBytesUntil('!', readChar, 64) > 0) {

        String read_ = String(readChar);
        int idx1 = read_.indexOf('%');
        int idx2 = read_.indexOf('$');

        String cmd = read_.substring(1,idx1);
        String data = read_.substring(idx1+1,idx2);


        // Just have a case for each of the commands
        // Feed them data if needed
        if (cmd == "en") {
            Enabled();

        } else if (cmd == "tv") {
            SetTargVels(data);

        } else if (cmd == "gv") {
            GetVels();

        } else if (cmd == "smv") {
            STMove(data);

        } else if (cmd == "sdg") {
            STDistanceToGo();
        }
    }
}


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


/* Utility methods to handle motor IO */

void readMotorVels() {
    // Do the movement motors first
    // Remember to do both direction and magnitude
    // Remember to check for 0 for PulseIn -- not moving
    int pR = pulseIn(PIN_RENC2, HIGH, PULSE_IN_TOUT);
    wR = digitalRead(PIN_RENC1) == HIGH ? 1.0 : -1.0;
    wR *= pR == 0 ? 0.0 : TWO_PI / (pR * PULSE_TO_FREQ);
    int pL = pulseIn(PIN_LENC1, HIGH, PULSE_IN_TOUT);
    wL = digitalRead(PIN_LENC2) == HIGH ? 1.0 : -1.0;
    wL *= pL == 0 ? 0.0 : TWO_PI / (pL * PULSE_TO_FREQ);

    // Only do the calculation for collectors if we have encoders otherwise, 
    //  arbitrarily write 0
    // Note we don't have to do direction computations for the collectors
    #if defined(ARDUINO_AVR_MEGA2560)
        int pCR = pulseIn(PIN_CRENC, HIGH, PULSE_IN_TOUT);
        wCR = pCR == 0 ? 0.0 : TWO_PI / (pCR * PULSE_TO_FREQ);
        int pCL = pulseIn(PIN_CLENC, HIGH, PULSE_IN_TOUT);
        wCL = pCL == 0 ? 0.0 : TWO_PI / (pCL * PULSE_TO_FREQ);
    #elif defined(ARDUINO_AVR_UNO)
        wCR = 0.0;
        wCL = 0.0;
    #else
        #error "Only supported platforms are Uno and Mega"
    #endif
}

void writeMotorDirs() {
    digitalWrite(PIN_RDIR1, wTargR >= 0.0 ? HIGH : LOW);
    digitalWrite(PIN_RDIR2, wTargR >= 0.0 ? LOW : HIGH);
    digitalWrite(PIN_LDIR1, wTargL >= 0.0 ? HIGH : LOW);
    digitalWrite(PIN_LDIR2, wTargL >= 0.0 ? LOW : HIGH);
}

void writeMotorVels() {
    // Write the movement motors first
    // Remember to bound to integers 0-255
    analogWrite(PIN_RPWM, max(0, min(255, (int) abs(W_TO_PWM * wPidR))));
    analogWrite(PIN_LPWM, max(0, min(255, (int) abs(W_TO_PWM * wPidL))));

    // Different logic for collectors depending on platform
    #if defined(ARDUINO_AVR_MEGA2560)
        analogWrite(PIN_CRPWM, max(0, min(255, pwmCR)));
        analogWrite(PIN_CLPWM, max(0, min(255, pwmCL)));
    #elif defined(ARDUINO_AVR_UNO)
        // Arbitrarily take their average
        analogWrite(PIN_CPWM, max(0, min(255, (pwmCR + pwmCL) / 2)));
    #else
        #error "Only supported platforms are Uno and Mega"
    #endif
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

    // Movement motor setup
    pinMode(PIN_RPWM, OUTPUT);
    pinMode(PIN_LPWM, OUTPUT);
    pinMode(PIN_RDIR1, OUTPUT);
    pinMode(PIN_RDIR2, OUTPUT);
    pinMode(PIN_LDIR1, OUTPUT);
    pinMode(PIN_LDIR2, OUTPUT);
    // Encoders
    pinMode(PIN_RENC1, INPUT);
    pinMode(PIN_RENC2, INPUT);
    pinMode(PIN_LENC1, INPUT);
    pinMode(PIN_LENC2, INPUT);

    // Collector motor setup
    #if defined(ARDUINO_AVR_MEGA2560)
        pinMode(PIN_CRPWM, OUTPUT);
        pinMode(PIN_CLPWM, OUTPUT);
    #elif defined(ARDUINO_AVR_UNO)
        pinMode(PIN_CPWM, OUTPUT);
    #else
        #error "Only supported platforms are Uno and Mega"
    #endif
    // Encoders
    #if defined(ARDUINO_AVR_MEGA2560)
        pinMode(PIN_CRENC, INPUT);
        pinMode(PIN_CLENC, INPUT);
    #elif !defined(ARDUINO_AVR_UNO)
        #error "Only supported platforms are Uno and Mega"
    #endif

    // PID setup
    pidR.SetMode(AUTOMATIC);
    pidL.SetMode(AUTOMATIC);


    // Stepper setup
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
    stepper.setSpeed(STEPPER_MAX_SPEED);
    stepper.setAcceleration(STEPPER_MAX_ACCEL);
}

void loop() {
    readMotorVels();
    pidR.Compute();
    pidL.Compute();
    writeMotorDirs();
    writeMotorVels();

    SerialParser();

    int i = 0;
    while(stepper.distanceToGo() != 0 && i < NUM_STEPS_PER_CYCLE){
        i += stepper.runSpeedToPosition() ? 1 : 0;
    }
}
