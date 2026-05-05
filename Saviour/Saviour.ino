// Define the pins we are using
const int buttonPin = 2;    // The digital pin connected to the button
const int greenLEDPin = 3;  // The digital pin connected to the Green leg of the RGB LED
const int redLEDPin = 4;    // The digital pin connected to the Red leg of the RGB LED

// Variable to store the current state of the button
int buttonState = 0; 

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Set the LED pins as outputs
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  
  // Set the button pin as an input with the internal pull-up resistor
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // Because we used INPUT_PULLUP, the button reads LOW when pressed.
  if (buttonState == LOW) {
    // BUTTON IS PRESSED:
    // Turn the Green LED ON and the Red LED OFF
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    
    // Print to the Serial Monitor
    Serial.println("Button is PRESSED! Light is GREEN.");
    
    // A small delay prevents the serial monitor from being flooded with text instantly
    delay(100); 
  } else {
    // BUTTON IS NOT PRESSED: 
    // Turn the Red LED ON and the Green LED OFF
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
  }
}