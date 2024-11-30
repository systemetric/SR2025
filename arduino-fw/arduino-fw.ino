#include <Arduino.h>
//Constants

//Digital inputs for motor M0 and M1
const int signal0A = 2;
const int signal0B = 4;
const int signal1A = 3;
const int signal1B = 7;

//debugging pins
const int debugUART = 8;
const int debugInterrupt = 9;

//Interrupt functions
void CountMotor0();
void CountMotor1();


//Global variables
volatile long countM0 = 0;
volatile long countM1 = 0;

// We communicate with the Arduino at 115200 baud.
#define SERIAL_BAUD 115200

#define FW_VER 2

void setup() {
  Serial.begin(SERIAL_BAUD);

  //Mode declaration
  pinMode(signal0A, INPUT);
  pinMode(signal0B, INPUT);

  pinMode(signal1A, INPUT);
  pinMode(signal1B, INPUT);

  pinMode(debugUART, OUTPUT);
  pinMode(debugInterrupt, OUTPUT);

  //Trigger interrupts on rising edges of M0 and M1 A signals

  attachInterrupt(digitalPinToInterrupt(signal0A), CountMotor0, RISING);
  attachInterrupt(digitalPinToInterrupt(signal1A), CountMotor1, RISING);
}

int read_pin() {
  // Convert the ASCII character to a pin number.
  // a -> 0, b -> 1, c -> 2, etc.
  while (!Serial.available());
  int pin = Serial.read();
  return (int)(pin - 'a');
}

void command_read() {
  int pin = read_pin();
  // Read from the expected pin.
  int level = digitalRead(pin);
  // Send back the result indicator.
  if (level == HIGH) {
    Serial.write('h');
  } else {
    Serial.write('l');
  }
}

void command_analog_read() {
  int pin = read_pin();
  int value = analogRead(pin);
  Serial.print(value);
}

void command_write(int level) {
  int pin = read_pin();
  digitalWrite(pin, level);
}

void command_mode(int mode) {
  int pin = read_pin();
  pinMode(pin, mode);
}

void command_ultrasound() {
  int pulse = read_pin();
  int echo = read_pin();

  // config pins to correct modes
  pinMode(pulse, OUTPUT);
  pinMode(echo, INPUT);

  // provide pulse to trigger reading
  digitalWrite(pulse, LOW);
  delayMicroseconds(2);
  digitalWrite(pulse, HIGH);
  delayMicroseconds(5);
  digitalWrite(pulse, LOW);

  // measure the echo time on the echo pin
  int duration = pulseIn(echo, HIGH, 60000);
  Serial.print(microsecondsToMm(duration));
}

long microsecondsToMm(long microseconds) {
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance we need half
  // 10 x (us / 29 / 2)
  return (5 * microseconds / 29);
}

// CUSTOM MOTOR HANDLING //


//Interrupt functions
void CountMotor0()
{
  if (digitalRead(debugInterrupt))
    digitalWrite(debugInterrupt, 0);
  else
    digitalWrite(debugInterrupt, 1);
  //Check if B is high or low to decide how to change the count
  countM0 += (digitalRead(signal0B) == 0 ? 1 : -1);
}

void CountMotor1()
{
  digitalWrite(debugInterrupt, 1);
  countM1 += (digitalRead(signal1B) == 0 ? 1 : -1);
  digitalWrite(debugInterrupt, 0);
}

void motorHandleSerial() {
  if (Serial.read()) {
    //Turn of interrupts whilst reallocating the location of countM0 and countM1
    noInterrupts();
    //Store volatile variables in some cache
    long countM0Copy = countM0;
    long countM1Copy = countM1;
    interrupts();

    //This pulse allows us to see how long the message takes to be sent of the serial port
    digitalWrite(debugUART, 1);

    Serial.print(countM0Copy);
    Serial.print(";");
    Serial.print(countM1Copy);
    Serial.print("|");

    //Print out the counts to the serial port
    // One byte at a time...
    //int i;
    //for (i = 0; i < 4; i++) {
    //    Serial.write((uint8_t)countM0Copy);
    //    countM0Copy >>= 8;
    //}

    //for (i = 0; i < 4; i++) {
    //    Serial.write((uint8_t)countM1Copy);
    //    countM1Copy >>= 8;
    //}

    digitalWrite(debugUART, 0);
  }
}

// EXISTING CODE //

void loop() {
  // Fetch all commands that are in the buffer
  while (Serial.available()) {
    int selected_command = Serial.read();
    // Do something different based on what we got:
    switch (selected_command) {
      case 'a':
        command_analog_read();
        break;
      case 'r':
        command_read();
        break;
      case 'l':
        command_write(LOW);
        break;
      case 'h':
        command_write(HIGH);
        break;
      case 'i':
        command_mode(INPUT);
        break;
      case 'o':
        command_mode(OUTPUT);
        break;
      case 'p':
        command_mode(INPUT_PULLUP);
        break;
      case 'u':
        command_ultrasound();
        break;
      case 'v':
        Serial.print("SRcustom:");
        Serial.print(FW_VER);
        break;
      case 'm':
        motorHandleSerial();
        break;
      case 'c': // c to reset counts
        countM0 = 0;
        countM1 = 0;
        break;
      default:
        // A problem here: we do not know how to handle the command!
        // Just ignore this for now.
        break;
    }
    Serial.print("\n");
  }
}
