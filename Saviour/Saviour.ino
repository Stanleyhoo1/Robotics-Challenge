// Define the pins we are using
const int buttonPin = 2;    // The digital pin connected to the button
const int greenLEDPin = 3;  // The digital pin connected to the Green leg of the RGB LED

// Variable to store the current state of the button
int buttonState = 0; 

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Set the green LED pin as an output
  pinMode(greenLEDPin, OUTPUT);
  
  // Set the button pin as an input.
  // INPUT_PULLUP turns on the Arduino's internal resistor, 
  // pulling the pin HIGH (3.3V) when the button is NOT pressed.
  pinMode(buttonPin, INPUT_PULLUP);
}

void loop() {
  // Read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // Because we used INPUT_PULLUP, the button reads LOW when pressed.
  if (buttonState == LOW) {
    // Button is pressed: turn the Green LED ON
    digitalWrite(greenLEDPin, HIGH);
    
    // Print to the Serial Monitor
    Serial.println("Button is PRESSED!");
    
    // A small delay prevents the serial monitor from being flooded with text instantly
    delay(100); 
  } else {
    // Button is not pressed: turn the Green LED OFF
    digitalWrite(greenLEDPin, LOW);
  }
}