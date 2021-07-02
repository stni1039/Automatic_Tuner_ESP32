/* Projekt: Automatisches Stimmgerät für die Violine
 * Sommersemester 2021 (18.06.2021), Hochschule Karlsruhe
 * stni1039 <stni1039@h-ka.de>
 * 
 * Programm zur automatischen Erkennung des gespielten Tons und 
 * Stimmung durch einen Schrittmotor auf die richtige Tonhoehe
 */
#include "arduinoFFT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Stepper.h>

//Definition of String Frequencies and params for better Frequency recognition
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define E5 659.3
#define A4 440
#define D4 293.6
#define G3 196
#define OFFSETFREQ 130  //if measured Frequency is above OFFSETFREQ, OFFSET is applied
#define OFFSET 1.5 //OFFSET ab OFFSETFREQ  (Ausgabe=Gemessen-OFFSET)
#define RANGE 1.5  //Range around Tone for displaying "perfect" (Precision)
#define SAMPLES 512             //SAMPLES-pt FFT. Must be a base 2 number.
#define SAMPLING_FREQUENCY 2048 //Ts = Based on Nyquist, must be 2 times the highest expected frequency.

//Declaration of Variables
const int stepsPerRevolution = 2048;
unsigned int samplingPeriod;
unsigned long microSeconds;
double SumSamples;
int MaxAnalogRead;
char Note;
double vReal[SAMPLES]; //create vector of size SAMPLES to hold real values
double vImag[SAMPLES]; //create vector of size SAMPLES to hold imaginary values
double peak = 0;
double peak_old = 0;

//Declaration of functions
void up();
void down();
void correct();
char FindClosestNoteAndDisplay(int);
//Setup display and motor
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
arduinoFFT FFT = arduinoFFT();
Stepper motor = Stepper(stepsPerRevolution, 12 , 27, 14, 26); //PIN1, PIN3,PIN2,PIN4

void setup()
{
  motor.setSpeed(15);
  Serial.begin(115200); //Baud rate for the Serial Monitor
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY)); //Period in microseconds
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
}

void loop()
{
  MaxAnalogRead = 0;
  SumSamples = 0;
  for (int i = 0; i < SAMPLES; i++)
  {
    microSeconds = micros();    //Returns the number of microseconds since the Arduino board began running the current script

    vReal[i] = analogRead(A0); //Reads the value from analog pin 0 (A0), quantize it and save it as a real term.
    vImag[i] = 0; //Makes imaginary term 0 always
    //remaining wait time between samples if necessary
    while (micros() < (microSeconds + samplingPeriod));
    //determine maximum AnalogRead for Sensor sensitivity Setup
    SumSamples += vReal[i];
    if (vReal[i] > MaxAnalogRead) {
      MaxAnalogRead = vReal[i];
    }
    //remaining wait time between samples if necessary
    while (micros() < (microSeconds + samplingPeriod));
  }
  //Perform FFT on samples
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);
  //Find peak frequency and print peak
  peak = FFT.MajorPeak(vReal, SAMPLES, SAMPLING_FREQUENCY);

  display.clearDisplay();

  //  executed only with relatively constant frequency measurement (avoid wrong measurement)
  if (peak > peak_old - 3 && peak < peak_old + 3) {
  //Apply offset if necessary
    if (peak >= OFFSETFREQ)
      peak = peak - OFFSET;
    display.setCursor(0, 0);
    display.setTextSize(1);
    display.print("MicMax: ");
    display.println(MaxAnalogRead);
    display.print("MicAvg: ");
    display.print((int)SumSamples / SAMPLES);
    display.setCursor(40, 20);
    display.setTextSize(2);
    display.print((int)peak);
    display.print(" Hz");
    display.setCursor(5, 20);
    
    //assigning measured frequency to char "Note"
    Note = FindClosestNoteAndDisplay(peak);
    display.print(Note);
    
    //tuning the instrument to the string closest to detected peak frequency
    switch (Note) {
      case 'E':
        if (peak < (E5 - RANGE))
          up();
        else if (peak > (E5 + RANGE))
          down();
        else
          correct();
        break;
      case 'A':
        if (peak < (A4 - RANGE))
          up();
        else if (peak > (A4 + RANGE))
          down();
        else
          correct();
        break;
      case 'D':
        if (peak < (D4 - RANGE))
          up();
        else if (peak > (D4 + RANGE))
          down();
        else
          correct();
        break;
      case 'G':
        if (peak < (G3 - RANGE))
          up();
        else if (peak > (G3 + RANGE))
          down();
        else
          correct();
        break;
    }

    display.display();
  }
  peak_old = peak;
}


void down() {
  display.setCursor(40, 40);
  display.println("down");
  motor.step(stepsPerRevolution / 4); //Quarter Rotation counter-clockwise --> tune down
  }

void up() {
  display.setCursor(40, 40);
  display.println("up");
  motor.step(-stepsPerRevolution / 4); //Quarter Rotation clockwise --> tune up
}

void correct() {
  display.setCursor(20, 40);
  display.println("perfect!");
}

char FindClosestNoteAndDisplay(int inputFrequency) {
  if (inputFrequency >= (E5 - (E5 - A4) / 2))
    Note = 'E';
  else if (inputFrequency > (A4 - (A4 - D4) / 2))
    Note = 'A';
  else if (inputFrequency > (D4 - (D4 - G3) / 2))
    Note = 'D';
  else if (inputFrequency <= (G3 + (D4 - G3) / 2))
    Note = 'G';
  return Note;
}
