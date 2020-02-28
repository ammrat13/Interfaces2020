#include <PID_v1.h>
#include <AccelStepper.h>


#define MOTOR_KP (.035)
#define MOTOR_KI (.0003)
#define MOTOR_KD (.0005)

#define W_TO_PWM (300)

#define STEPPER_MAX_SPEED (500)

#define ENABLED_DEBOUNCE_T (200)
#define PULSE_IN_TOUT (50000)


#define PIN_EN_DETECT ()
#define PIN_EN_LED ()

#define PIN_MEN ()
#define PIN_RENC ()
#define PIN_LENC ()
#define PIN_RPWM ()
#define PIN_RGND ()
#define PIN_LPWM ()
#define PIN_LGND ()

#define PIN_ST0 ()
#define PIN_ST1 ()
#define PIN_ST2 ()
#define PIN_ST3 ()


boolean connected = false;

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
    String ps[2];
    split(ps, 2, data, '%');
    // Write them
    wTargR = ps[0].toDouble();
    wTargL = ps[1].toDouble();
}

// Also have a getter
void GetVels() {
    Serial.print(wR, 15);
    Serial.print(",");
    Serial.println(wL, 15);
}


// We only need minimal stepper handlers
// Just get to the target position
void STRunToNewPosition(String data){
    stepper.runToNewPosition(data.toInt());
    Serial.println("D");
}
void STStop(){
    stepper.stop();
}


void SerialParser(void) {

    char readChar[64];
    Serial.readBytesUntil('!', readChar, 64);

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

    } else if (cmd == "srt") {
        STRunToNewPosition(data);

    } else if (cmd == "sst") {
        STStop();
    }

}


// Handlers to take care of interrupts when the button is pushed
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
void enableRisingHandler() {
    // Just log the debounce time if it changes in the wrong direction
    enabledDebounce = millis();
}


// Utility methods to handle motor IO
void readMotorVels() {
    // Read
    int pR = pulseIn(PIN_RENC, HIGH, PULSE_IN_TOUT);
    int pL = pulseIn(PIN_LENC, HIGH, PULSE_IN_TOUT);
    // Make sure we actually got a length of time
    // See `DeadReckoning.ino` for magic number
    wR = pR == 0 ? 0.0 : TWO_PI / (pR * 2.4886e-3);
    wL = pL == 0 ? 0.0 : TWO_PI / (pL * 2.4886e-3);
}
void writeMotorVels() {
    // Bound to integers 0 - 255
    analogWrite(PIN_RPWM, max(0, min(255, (int) (W_TO_PWM * wPidR))));
    analogWrite(PIN_LPWM, max(0, min(255, (int) (W_TO_PWM * wPidL))));
}


void setup()  {
    Serial.begin(115200);
    
    // Create the handler for enable
    pinMode(PIN_EN_DETECT, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(PIN_EN_DETECT), enableRisingHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_EN_DETECT), enableFallingHandler, FALLING);

    // For the enabled LED
    pinMode(PIN_EN_LED, OUTPUT);
    digitalWrite(PIN_EN_LED, LOW);

    // Motors
    pinMode(PIN_MEN, OUTPUT);
    pinMode(PIN_RPWM, OUTPUT);
    pinMode(PIN_LPWM, OUTPUT);
    pinMode(PIN_RGND, OUTPUT);
    pinMode(PIN_LGND, OUTPUT);
    digitalWrite(PIN_EN, HIGH);
    digitalWrite(PIN_RGND, LOW);
    digitalWrite(PIN_LGND, LOW);

    // Motor encoders
    pinMode(PIN_RENC, INPUT);
    pinMode(PIN_LENC, INPUT);

    // PID setup
    pidR.SetMode(AUTOMATIC);
    pidL.SetMode(AUTOMATIC);

    // Stepper setup
    stepper.setMaxSpeed(STEPPER_MAX_SPEED);
}

void loop() {
    readMotorVels();
    pidR.Compute();
    pidL.Compute();
    writeMotorVels();

    SerialParser();
}
