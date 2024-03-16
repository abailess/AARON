int diode1 = 2;           // the PWM pin the LED is attached to
int brightness = 0;       // how bright the LED is
int fadeAmount = 10;      // how many points to fade the LED by
const int sensorPin = A6; // Define the analog input pin A6

// the setup routine runs once when you press reset:
void setup() {
  // declare pin 2 to be an output:
  pinMode(diode1, OUTPUT);
  Serial.begin(9600); // Initialize serial communication at 9600 baud rate
}

// the loop routine runs over and over again forever:
void loop() {
  unsigned long startTime = millis(); // Record the start time
  unsigned long sum = 0; // Variable to accumulate the sum of readings
  int count = 0; // Counter to keep track of the number of readings

  // Integrate the analog readings over 100ms
  while (millis() - startTime < 100) {
    sum += analogRead(sensorPin); // Add each reading to the sum
    count++; // Increment the count
  }

  // Calculate the average value
  int integratedValue = sum / count;

  // set the brightness of pin 2:
  analogWrite(diode1, brightness);

  // change the brightness for next time through the loop:
  brightness = brightness + fadeAmount;

  // reverse the direction of the fading at the ends of the fade:
  if (brightness <= 0 || brightness >= 250) {
    fadeAmount = -fadeAmount;
  }
  
  // Print integrated sensor value and LED brightness to the serial monitor
  Serial.print("Integrated Photodiode Analog: ");
  Serial.print(integratedValue);
  Serial.print("\t"); // Tab character
  Serial.print("LED Brightness: ");
  Serial.println(brightness);

  // Wait for 1 second before reading the sensor again
  delay(1000);
}
