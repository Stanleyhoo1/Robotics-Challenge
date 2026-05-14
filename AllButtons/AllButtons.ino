// Define the pins we are using
const int buttonPin1 = 2;    // The digital pin connected to the first button
const int buttonPin2 = 5;    // The digital pin connected to the second button
const int buttonPin3 = 41;    // NEW: The digital pin connected to the third button
const int greenLEDPin = 49;   // The digital pin connected to the Green leg of the RGB LED
const int redLEDPin = 43;     // The digital pin connected to the Red leg of the RGB LED

// Variables to store the current state of the buttons
int buttonState1 = 0; 
int buttonState2 = 0; 
int buttonState3 = 0;        // NEW: Variable to store the third button's state

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Set the LED pins as outputs
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  
  // Set ALL button pins as inputs with the internal pull-up resistor
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP); // NEW: Setup the third button
}

void loop() {
  // Read the state of ALL pushbuttons:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3); // NEW: Read the third button

  // Because we used INPUT_PULLUP, the buttons read LOW when pressed.
  
  // Check the 3rd button first so it takes priority
  if (buttonState3 == LOW) {
    // 3RD BUTTON IS PRESSED: 
    // Blink the Red LED
    Serial.println("Button 3 is PRESSED! LED is BLINKING RED.");
    
    digitalWrite(greenLEDPin, LOW); // Make sure Green is OFF
    digitalWrite(redLEDPin, HIGH);  // Turn Red ON
    delay(250);                     // Wait 250 milliseconds
    digitalWrite(redLEDPin, LOW);   // Turn Red OFF
    delay(250);                     // Wait 250 milliseconds
    
  } else if (buttonState1 == LOW || buttonState2 == LOW) {
    // BUTTON 1 -OR- BUTTON 2 IS PRESSED:
    // Turn the Green LED ON and the Red LED OFF
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    
    // Print to the Serial Monitor
    Serial.println("Button 1 or 2 is PRESSED! Light is GREEN.");
    
    // A small delay prevents the serial monitor from being flooded with text instantly
    delay(100); 
    
  } else {
    // NO BUTTONS ARE PRESSED: 
    // Turn the Red LED ON and the Green LED OFF
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
  }
}