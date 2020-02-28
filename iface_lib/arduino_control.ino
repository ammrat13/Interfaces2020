#include <AccelStepper.h>

#define ENABLED_DEBOUNCE_T (200)
#define PULSE_IN_TOUT (50000)


boolean connected = false;

boolean enabled = false;
unsigned long enabledDebounce = 0;

AccelStepper stepper(AccelStepper::FULL4WIRE, 8, 9, 10, 11, true);


int str2int (String str_value) {
    char buffer[10];
    str_value.toCharArray(buffer, 10);

    return atoi(buffer);
}

void split(String results[], int len, String input, char spChar) {
    String temp = input;

    for (int i = 0; i < len; i++) {
        int idx = temp.indexOf(spChar);
        results[i] = temp.substring(0,idx);
        temp = temp.substring(idx+1);
    }
}


void DigitalHandler(int mode, String data) {
    int pin = str2int(data);
    
    if(mode <= 0) {
        Serial.println(digitalRead(pin));

    } else {
        if(pin < 0) {
            digitalWrite(-pin,LOW);
        } else {
            digitalWrite(pin,HIGH);
        }
    }
}

void AnalogHandler(int mode, String data) {
    if(mode <= 0) {
        int pin = str2int(data);
        Serial.println(analogRead(pin));

    } else {
        String sdata[2];
        split(sdata,2,data,'%');

        int pin = str2int(sdata[0]);
        int pv = str2int(sdata[1]);
        analogWrite(pin,pv);
    }
}

void ConfigurePinHandler(String data) {
    int pin = str2int(data);

    if(pin <= 0) {
        pinMode(-pin,INPUT);
    } else {
        pinMode(pin,OUTPUT);
    }
}


void PulseInHandler(String data){
    int pin = str2int(data);
    long duration;

    if(pin <= 0){
        pinMode(-pin, INPUT);
        duration = pulseIn(-pin, LOW, PULSE_IN_TOUT);

    } else {
        pinMode(pin, INPUT);
        duration = pulseIn(pin, HIGH, PULSE_IN_TOUT);      
    }

    Serial.println(duration);
}


void Enabled() {
    Serial.println(enabled);
}

void STRun(){
    stepper.run();
}

void STRunSpeed(){
    stepper.runSpeed();
}

void STSetMaxSpeed(String data){
    stepper.setMaxSpeed(str2int(data));
}

void STSetSpeed(String data){
    stepper.setSpeed(str2int(data));
}

void STMove(String data){
    stepper.move(str2int(data));
}

void STCurrentPosition(){
    Serial.println(stepper.currentPosition());
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


    if (cmd == "dw") {
        DigitalHandler(1, data);

    } else if (cmd == "dr") {
        DigitalHandler(0, data);

    } else if (cmd == "aw") {
        AnalogHandler(1, data);

    } else if (cmd == "ar") {
        AnalogHandler(0, data);

    } else if (cmd == "pm") {
        ConfigurePinHandler(data);

    } else if (cmd == "pi") {
        PulseInHandler(data);

    } else if (cmd == "en") {
        Enabled();

    } else if (cmd == "sru") {
        STRun();

    } else if (cmd == "srs") {
        STRunSpeed();

    } else if (cmd == "sms") {
        STSetMaxSpeed(data);

    } else if (cmd == "sss") {
        STSetSpeed(data);

    } else if (cmd == "smm") {
        STMove(data);

    } else if (cmd == "scp") {
        STCurrentPosition();

    } else if (cmd == "sst") {
        STStop();
    }

}


void enableFallingHandler() {
    if(millis() > enabledDebounce + ENABLED_DEBOUNCE_T){
        enabled = !enabled;
    }
    enabledDebounce = millis();
}

void enableRisingHandler() {
    enabledDebounce = millis();
}

void setup()  {
    Serial.begin(115200);
    
    // Create the handler for enable
    pinMode(2, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(2), enableRisingHandler, RISING);
    attachInterrupt(digitalPinToInterrupt(2), enableFallingHandler, FALLING);
}

void loop() {
    SerialParser();
}
