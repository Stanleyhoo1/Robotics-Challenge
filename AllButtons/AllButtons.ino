// Define the pins we are using
const int buttonPin1 = 2;    // The digital pin connected to the first button
const int buttonPin2 = 5;    // The digital pin connected to the second button
const int buttonPin3 = 41;   // The digital pin connected to the third button
const int greenLEDPin = 49;  // The digital pin connected to the Green leg of the RGB LED
const int redLEDPin = 43;    // The digital pin connected to the Red leg of the RGB LED

// Variables to store the current state of the buttons
int buttonState1 = 0; 
int buttonState2 = 0; 
int buttonState3 = 0;        

// NEW VARIABLES for the toggle and blink logic
int lastButtonState3 = HIGH;     // Tracks the previous state of Button 3
bool isBlinking = false;         // Tracks whether we are currently in "Blink Mode"
unsigned long previousMillis = 0; // Stores the last time the LED was updated
const long interval = 250;       // Interval at which to blink (milliseconds)
int currentRedLedState = LOW;    // Tracks if the Red LED is currently ON or OFF during the blink

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Set the LED pins as outputs
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  
  // Set ALL button pins as inputs with the internal pull-up resistor
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP); 
}

void loop() {
  // Read the state of ALL pushbuttons:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);
  buttonState3 = digitalRead(buttonPin3);

  // --- 1. CHECK FOR BUTTON 3 TOGGLE ---
  // If the button is LOW (pressed) and it was previously HIGH (unpressed)
  if (buttonState3 == LOW && lastButtonState3 == HIGH) {
    isBlinking = !isBlinking; // Flip the state (if false make it true, if true make it false)
    delay(50); // Small debounce delay to prevent the button from registering multiple clicks instantly
  }
  
  // Save the current state for the next loop
  lastButtonState3 = buttonState3;

  // --- 2. EXECUTE LOGIC BASED ON STATE ---
  if (isBlinking) {
    // 3RD BUTTON MODE IS ACTIVE: 
    // Ignore other buttons and blink the Red LED using millis()
    digitalWrite(greenLEDPin, LOW); // Make sure Green is OFF
    
    unsigned long currentMillis = millis(); // Check the current time
    
    // If enough time has passed (250ms), toggle the red LED
    if (currentMillis - previousMillis >= interval) {
      previousMillis = currentMillis; // Save the last time you blinked the LED
      
      // Flip the LED state
      if (currentRedLedState == LOW) {
        currentRedLedState = HIGH;
      } else {
        currentRedLedState = LOW;
      }
      
      digitalWrite(redLEDPin, currentRedLedState);
    }
    
  } else {
    // 3RD BUTTON MODE IS OFF: 
    // Normal operation for Buttons 1 and 2
    
    if (buttonState1 == LOW || buttonState2 == LOW) {
      // BUTTON 1 -OR- BUTTON 2 IS PRESSED:
      digitalWrite(greenLEDPin, HIGH);
      digitalWrite(redLEDPin, LOW);
      
    } else {
      // NO BUTTONS ARE PRESSED: 
      digitalWrite(redLEDPin, HIGH);
      digitalWrite(greenLEDPin, LOW);
    }
  }
}