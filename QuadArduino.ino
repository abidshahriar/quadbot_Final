#include <Servo.h>

// Define the servo pins
const int servoPin1 = 10; //theta1
const int servoPin2 = 11; //2
const int servoPin3 = 6;  //3

// Create servo objects
Servo servo1;
Servo servo2;
Servo servo3;

void setup() {
  // Initialize the serial port at 9600 baud
  Serial.begin(9600);

  // Attach the servos to their pins
  servo1.attach(servoPin1);
  servo2.attach(servoPin2);
  servo3.attach(servoPin3);
}

void loop() {
  // Check if there is data available on the serial port
  if (Serial.available() > 0) {
    // Read the incoming data
    String data = Serial.readStringUntil('\n');

    // Parse the data into servo angles
    int angle1 = data.substring(0, data.indexOf(",")).toInt();
    data = data.substring(data.indexOf(",") + 1);
    int angle2 = data.substring(0, data.indexOf(",")).toInt();
    data = data.substring(data.indexOf(",") + 1);
    int angle3 = data.toInt();

    // Move the servos to the specified angles
    servo1.write(angle1);
    servo2.write(angle2);
    servo3.write(angle3);
  }
}
