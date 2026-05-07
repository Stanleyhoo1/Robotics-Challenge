// Define the pins we are using
const int buttonPin1 = 2;    // The digital pin connected to the first button
const int buttonPin2 = 5;    // The digital pin connected to the second button
const int greenLEDPin = 3;   // The digital pin connected to the Green leg of the RGB LED
const int redLEDPin = 4;     // The digital pin connected to the Red leg of the RGB LED

// Variables to store the current state of the buttons
int buttonState1 = 0; 
int buttonState2 = 0; 

void setup() {
  // Initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  
  // Set the LED pins as outputs
  pinMode(greenLEDPin, OUTPUT);
  pinMode(redLEDPin, OUTPUT);
  
  // Set BOTH button pins as inputs with the internal pull-up resistor
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
}

void loop() {
  // Read the state of BOTH pushbuttons:
  buttonState1 = digitalRead(buttonPin1);
  buttonState2 = digitalRead(buttonPin2);

  // Because we used INPUT_PULLUP, the buttons read LOW when pressed.
  // The "||" symbol means "OR". This statement reads: 
  // "If button 1 is LOW -OR- button 2 is LOW..."
  if (buttonState1 == LOW || buttonState2 == LOW) {
    // AT LEAST ONE BUTTON IS PRESSED:
    // Turn the Green LED ON and the Red LED OFF
    digitalWrite(greenLEDPin, HIGH);
    digitalWrite(redLEDPin, LOW);
    
    // Print to the Serial Monitor
    Serial.println("A button is PRESSED! Light is GREEN.");
    
    // A small delay prevents the serial monitor from being flooded with text instantly
    delay(100); 
  } else {
    // NEITHER BUTTON IS PRESSED: 
    // Turn the Red LED ON and the Green LED OFF
    digitalWrite(redLEDPin, HIGH);
    digitalWrite(greenLEDPin, LOW);
  }
}