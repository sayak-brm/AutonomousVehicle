#include <Wire.h>

PROGMEM const int ENAX = 13;
PROGMEM const int IN1X = 12;
PROGMEM const int IN2X = 11;
PROGMEM const int IN3X = 9;
PROGMEM const int IN4X = 10;
PROGMEM const int ENBX = 8;
PROGMEM const int ENAY = 3;
PROGMEM const int IN1Y = 4;
PROGMEM const int IN2Y = 5;
PROGMEM const int IN3Y = 6;
PROGMEM const int IN4Y = 7;
PROGMEM const int ENBY = 2;

PROGMEM const int DRL = A0;
PROGMEM const int DIP = A1;

#define SLAVE_ADDRESS 0x01

int write_flag = -1;
byte write_data = -1;

void FRONT()
{
    digitalWrite(IN1X, HIGH);
    digitalWrite(IN2X, LOW);
    digitalWrite(IN3X, LOW);
    digitalWrite(IN4X, HIGH);
    digitalWrite(IN1Y, LOW);
    digitalWrite(IN2Y, HIGH);
    digitalWrite(IN3Y, HIGH);
    digitalWrite(IN4Y, LOW);
}

void BACK()
{
    digitalWrite(IN1X, LOW);
    digitalWrite(IN2X, HIGH);
    digitalWrite(IN3X, HIGH);
    digitalWrite(IN4X, LOW);
    digitalWrite(IN1Y, HIGH);
    digitalWrite(IN2Y, LOW);
    digitalWrite(IN3Y, LOW);
    digitalWrite(IN4Y, HIGH);
}

void RIGHT()
{
    digitalWrite(IN1X, LOW);
    digitalWrite(IN2X, HIGH);
    digitalWrite(IN3X, LOW);
    digitalWrite(IN4X, HIGH);
    digitalWrite(IN1Y, HIGH);
    digitalWrite(IN2Y, LOW);
    digitalWrite(IN3Y, HIGH);
    digitalWrite(IN4Y, LOW);
}

void LEFT()
{
    digitalWrite(IN1X, HIGH);
    digitalWrite(IN2X, LOW);
    digitalWrite(IN3X, HIGH);
    digitalWrite(IN4X, LOW);
    digitalWrite(IN1Y, LOW);
    digitalWrite(IN2Y, HIGH);
    digitalWrite(IN3Y, LOW);
    digitalWrite(IN4Y, HIGH);
}

void STOP()
{
    digitalWrite(IN1X, LOW);
    digitalWrite(IN2X, LOW);
    digitalWrite(IN3X, LOW);
    digitalWrite(IN4X, LOW);
    digitalWrite(IN1Y, LOW);
    digitalWrite(IN2Y, LOW);
    digitalWrite(IN3Y, LOW);
    digitalWrite(IN4Y, LOW);
}

// callback for received data
void receiveData(int n)
{
    Serial.print("Recv. bytes:");
    Serial.println(n);

    byte command[n] = {0};
    n = 0;
    while (Wire.available())
        command[n++] = Wire.read();

    switch (command[0])
    {
    case 0: //sonar req
        Serial.println("Recv. sonar request");
        write_flag = 0;
        break;

    case 1: //pinMode
        /*
       * INPUT        0x0
       * OUTPUT       0x1
       * INPUT_PULLUP 0x2
       */
        Serial.println("Recv. pinMode");
        Serial.print("Pin: ");
        Serial.println((int)command[1]);

        switch (command[2])
        {
        case 0: // INPUT
            pinMode((int)command[1], INPUT);
            Serial.println("Setting INPUT");
            break;
        case 1: //OUTPUT
            pinMode((int)command[1], OUTPUT);
            Serial.println("Setting OUTPUT");
            break;
        case 2: //INPUT_PULLUP
            pinMode((int)command[1], INPUT_PULLUP);
            Serial.println("Setting INPUT_PULLUP");
            break;
        }
        break;

    case 2: //digitalWrite
        /*
       * LOW          0x0
       * HIGH         0x1
       */
        Serial.println("Recv. digitalWrite");
        Serial.print("Pin: ");
        Serial.println((int)command[1]);

        switch (command[2])
        {
        case 0: //INPUT_PULLUP
            digitalWrite((int)command[1], LOW);
            Serial.println("Setting LOW");
            break;
        case 1: //INPUT_PULLUP
            digitalWrite((int)command[1], HIGH);
            Serial.println("Setting HIGH");
            break;
        }
        break;

    case 3: //digitalRead
        Serial.println("Recv. digitalRead");
        Serial.print("Pin: ");
        Serial.println((int)command[1]);
        write_flag = 1;
        write_data = command[1];
        break;

    case 4: //analogWrite
        Serial.println("Recv. analogWrite");
        Serial.print("Pin: ");
        Serial.println((int)command[1]);
        Serial.print("Level:");
        Serial.println((int)command[2]);
        analogWrite((int)command[1], (int)command[2]);
        break;

    case 5: //analogRead
        Serial.println("Recv. analogRead");
        Serial.print("Pin: ");
        Serial.println((int)command[1]);
        write_flag = 2;
        write_data = command[1];
        break;

    case 6: //drive
        Serial.println("Recv. drive");
        Serial.print("Command: ");
        Serial.println((int)command[1]);

        switch (command[1])
        {
        case 1: //front
            FRONT();
            break;
        case 2: //back
            BACK();
            break;
        case 3: //left
            LEFT();
            break;
        case 4: //right
            RIGHT();
            break;
        case 5: //stop
            STOP();
            break;
        }
        break;

    case 7: //speed
        Serial.println("Recv. speed");
        Serial.print("Speed: ");
        Serial.println((int)command[1]);

        analogWrite(ENAX, (int)command[1]);
        analogWrite(ENBX, (int)command[1]);
        analogWrite(ENAY, (int)command[1]);
        analogWrite(ENBY, (int)command[1]);
        break;
    }
}

// callback for sending data
void sendData()
{
    Serial.println("Recv. data request");
    int reading = 0;
    switch (write_flag)
    {
    case 0: //init, sonar data
        // Wire.write(sonar, 3);
        Serial.println("Sonar data transmitted");
        break;

    case 1: //digitalRead
        reading = digitalRead((int)write_data);
        Wire.write(reading);
        Serial.println("Digital data transmitted");
        Serial.print("Value:");
        Serial.println(reading);
        break;

    case 2: //analogRead
        reading = analogRead((int)write_data);
        Wire.write(highByte(reading));
        Wire.write(lowByte(reading));
        Serial.println("Analog data transmitted");
        Serial.print("Value:");
        Serial.println(reading);
        break;
    }
    write_flag = -1;
}

void setup()
{
    Serial.begin(9600); // start serial for output
    // initialize i2c as slave
    Wire.begin(SLAVE_ADDRESS);

    // define callbacks for i2c communication
    Wire.onReceive(receiveData);
    Wire.onRequest(sendData);

    pinMode(ENAX, OUTPUT);
    pinMode(ENBX, OUTPUT);
    pinMode(IN1X, OUTPUT);
    pinMode(IN2X, OUTPUT);
    pinMode(IN3X, OUTPUT);
    pinMode(IN4X, OUTPUT);
    pinMode(ENAY, OUTPUT);
    pinMode(ENBY, OUTPUT);
    pinMode(IN1Y, OUTPUT);
    pinMode(IN2Y, OUTPUT);
    pinMode(IN3Y, OUTPUT);
    pinMode(IN4Y, OUTPUT);
    analogWrite(ENAX, 255);
    analogWrite(ENBX, 255);
    analogWrite(ENAY, 255);
    analogWrite(ENBY, 255);

    pinMode(DRL, OUTPUT);
    pinMode(DIP, OUTPUT);
    digitalWrite(DRL, HIGH);
    digitalWrite(DIP, HIGH);

    Serial.println("Ready!");
}

void loop()
{
    // loop
}