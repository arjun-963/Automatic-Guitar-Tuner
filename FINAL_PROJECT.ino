#include "arduinoFFT.h"

#define SAMPLES 128             // SAMPLES-pt FFT. Must be a base 2 number. Max 128 for Arduino Uno.
#define SAMPLING_FREQUENCY 2048 // Ts = Based on Nyquist, must be 2 times the highest expected frequency.

arduinoFFT FFT = arduinoFFT();

unsigned int samplingPeriod;
unsigned long microSeconds;
double vReal[SAMPLES]; // Create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; // Create vector of size SAMPLES to hold imaginary values

double noteFreq[] = {329.63, 246.94, 196.00, 146.83, 110.00, 82.41}; // E4, B3, G3, D3, A2, E2
String noteNames[] = {"E4", "B3", "G3", "D3", "A2", "E2"};
const int numNotes = 6;
const double tolerance = 8; // Hz

const int buttonPin = 2; // Pin connected to the push button
int currentString = -1;  // Current string being detected (-1 means no string is being detected)
bool buttonState = HIGH; // Current state of the button
bool lastButtonState = HIGH; // Previous state of the button
bool detected = false; // Flag to indicate if the current string frequency is detected

// L298N Motor Driver Pins
const int enablePin = 3;  // Enable pin
const int in1Pin = 4;    // Input pin 4
const int in2Pin = 7;    // Input pin 7
const int ledPin = 12;  // Pin connected to the LED

void setup()
{
  Serial.begin(115200); // Baud rate for the Serial Monitor
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); // Period in microseconds
  pinMode(buttonPin, INPUT_PULLUP); // Set the button pin as input with internal pull-up resistor

  pinMode(enablePin, OUTPUT);
  pinMode(in1Pin, OUTPUT);
  pinMode(in2Pin, OUTPUT);
}

void loop()
{
  buttonState = digitalRead(buttonPin); // Read the current state of the button

  // Check if the button state has changed
  if (buttonState != lastButtonState)
  {
    delay(50); // Debounce delay

    // Check if the button is pressed (falling edge)
    if (buttonState == LOW)
    {
      currentString = (currentString + 1) % numNotes; // Switch to the next string
      digitalWrite(ledPin, LOW); // Turn off the LED

      Serial.print("Detecting String: ");
      Serial.println(noteNames[currentString]);

      detected = false; // Reset the detected flag
    }
  }

  // Continue the frequency sensing process if a string is being detected
  if (currentString >= 0 && !detected)
  {
    // Sample SAMPLES times
    for (int i = 0; i < SAMPLES; i++)
    {
      microSeconds = micros(); // Returns the number of microseconds since the Arduino board began running the current script.

      vReal[i] = analogRead(0); // Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
      vImag[i] = 0;             // Makes imaginary term 0 always

      // Remaining wait time between samples if necessary
      while (micros() < (microSeconds + samplingPeriod))
      {
        // Do nothing
      }
    }

    // Perform FFT on samples
    FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

    // Find peak frequency and print detected note
    double peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);
    Serial.print("Detected Frequency: ");
    Serial.println(peak);

    // Compare detected frequency with the selected string frequency
    if (abs(peak - noteFreq[currentString]) <= tolerance)
    {
      Serial.print("Detected Note: ");
      Serial.println(noteNames[currentString]);
      detected = true; // Set the detected flag to true
      digitalWrite(ledPin, HIGH); // Turn on the LED
    }
    else
    {
      double stringThreshold = noteFreq[currentString]; // Threshold frequency for the current string

      // Compare detected frequency with the threshold frequency
      if (peak < stringThreshold)
      {
        Serial.println("Tune Up");
        // Drive the motor clockwise
        digitalWrite(in1Pin, LOW);
        digitalWrite(in2Pin, HIGH);
      }
      else
      {
        Serial.println("Tune Down");
        // Drive the motor counterclockwise
        digitalWrite(in1Pin, HIGH);
        digitalWrite(in2Pin, LOW);
      }
      digitalWrite(enablePin, HIGH); // Enable the motor
    }
  }
  else
  {
    // Stop the motor if no string is being detected
    digitalWrite(in1Pin, LOW);
    digitalWrite(in2Pin, LOW);
    digitalWrite(enablePin, LOW); // Disable the motor
  }

  lastButtonState = buttonState; // Store the current button state for comparison in the next iteration
}
