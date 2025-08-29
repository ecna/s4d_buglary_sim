/*
 * s4d_breadboard.h
 * version 3
 *  Created on: Jul 3, 2024
 *      Author: Robert Holwerda and Richard Holleman
 */

//#define SIMULATION
//#define HW_VERSION2
//#define NO_OLED

#ifndef S4D_BREADBOARD_H
#define S4D_BREADBOARD_H

#ifdef SIMULATION
#define NO_OLED
#define NO_VOLUMESENSOR
#endif

#ifndef NO_OLED
#include <U8g2lib.h>
void initializeOLED();
#endif


/* * * * * * * * * * * * * * * * * * *
   
    Below the pin names for the
    S4D board.
    Use these pin names with
    Arduino commands like
    analogRead(), digitalRead(),
    analogWrite() and digitalWrite()
                                    
 * * * * * * * * * * * * * * * * * * */

#ifdef HW_VERSION2
const int MAGNETSENSOR = A0;   // analog input
const int POTENTIOMETER = A3;  // analog input
const int LIGHTSENSOR = A1;    // analog input

const int BUZZER = 10;  // digital output

const int LED_BLUE = 6;     // analog and digital output
const int LED_GREEN = 5;    // analog and digital output
const int LED_YELLOW = 11;  // analog and digital output
const int LED_RED = 3;      // analog and digital output
#else
const int MAGNETSENSOR = A1;   // analog input
const int POTENTIOMETER = A2;  // analog input
const int LIGHTSENSOR = A3;    // analog input

const int BUZZER = 6;  // digital output

const int LED_BLUE = 5;     // analog and digital output
const int LED_GREEN = 9;    // analog and digital output
const int LED_YELLOW = 10;  // analog and digital output
const int LED_RED = 11;     // analog and digital output
#endif

/* * * * * * * * * * * * * * * * * * *

  Use the following code to initialize
  the breadboard.

    initializeBreadboard()

  Always call initializeBreadboard()
  in your setup() function when using
  the breadboard.

* * * * * * * * * * * * * * * * * * * */

void initializeBreadboard() {
  pinMode(BUZZER, OUTPUT);
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  pinMode(LED_YELLOW, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(MAGNETSENSOR, INPUT);
  pinMode(POTENTIOMETER, INPUT);
  pinMode(LIGHTSENSOR, INPUT);

  digitalWrite(LED_BLUE, LOW);
  digitalWrite(LED_GREEN, LOW);
  digitalWrite(LED_YELLOW, LOW);
  digitalWrite(LED_RED, LOW);

  Serial.begin(9600);
  tone(BUZZER, 440);
  delay(50);
  noTone(BUZZER);
#ifndef NO_OLED
  initializeOLED();
#endif
}

/* * * * * * * * * * * * * * * * * * *

  The code below creates the following
  command:
  
  Button.Button1Pressed()
  Button.Button2Pressed()
  
 * * * * * * * * * * * * * * * * * * */

class ButtonClass {

#ifdef HW_VERSION2
  const int BUTTON1 = 13;  // digital input
  const int BUTTON2 = 7;   // digital input
#else
  const int BUTTON1 = 4;    // digital input
  const int BUTTON2 = 3;    // digital input
#endif

public:

  ButtonClass() {
#ifdef HW_VERSION2
    pinMode(BUTTON1, INPUT_PULLUP);
#else
    pinMode(BUTTON1, INPUT);
#endif
    pinMode(BUTTON2, INPUT);
  }

  int button1Pressed() {
#ifdef HW_VERSION2
    return (!digitalRead(BUTTON1));
#else
    return (digitalRead(BUTTON1));
#endif
  }

  int button2Pressed() {
    return digitalRead(BUTTON2);
  }
};

ButtonClass Button;


/* * * * * * * * * * * * * * * * * * *

  The code below creates the following
  commands:
    
    OLED.clear()                      // Removes all pixels from the screen.
    OLED.copyToSerial()               // Everything printed to OLED will also be sent to the
                                      // Serial Monitor.
    
    OLED.print( value )               // Shows a value (text, number) on the screen.
    OLED.print( string, value )       // You can add a label or message before the value.

    OLED.printTop( value )            // Prints a smaller line in the upper half of the screen.
    OLED.printTop( string, value )    // You can add a label or message before the value.

    OLED.printBottom( value )         // Prints a smaller line in the lower half of the screen.
    OLED.printBottom( string, value ) // You can add a label or message before the value.

  The value for OLED.print() can be 
  a text or a number.

 * * * * * * * * * * * * * * * * * * *

  If you don't use the OLED, define
  the NO_OLED macro just before 
  including this header file. 
  Like this:

  #define NO_OLED
  #include "s4d_breadboard.h"

  This will exclude all OLED code 
  from the program, which can speed 
  up compile times, and save a lot 
  of memory.
  The OLED commands above will still
  work. They'll simply print to the
  Serial Monitor, as if 
  OLED.copyToSerial() had been called.
      
 * * * * * * * * * * * * * * * * * * */

#ifndef NO_OLED
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))

// This is some ugly code, required to get the display to work.
U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0,
                                            /* reset=*/U8X8_PIN_NONE,
                                            /* clock=*/SCL,
                                            /* data=*/SDA);
#endif

class OledClass {

private:
#ifndef NO_OLED
  bool mustCopyToSerial = false;
#else
  bool mustCopyToSerial = true;            // if OLED disabled, send OLED output
                                           // to Serial Monitor.
#endif
  bool lastPrintWasSmall = false;  // if last print was half screen,
                                   // next small print should clear
                                   // only half the screen.
  String lastLines[2] = { "", "" };

public:

  OledClass() {
  }

  void copyToSerial() {
    mustCopyToSerial = true;
  }

  void printToSerial() {
    if (!mustCopyToSerial) {
      return;
    }
    int boxLength = max(lastLines[0].length(), lastLines[1].length()) + 2;
    Serial.print("+");
    for (int i = 0; i < boxLength; i++) {
      Serial.print('-');
    }
    Serial.println("+");
    Serial.print("| ");
    Serial.print(lastLines[0]);
    for (int i = 0; i < boxLength - int(lastLines[0].length()) - 1; i++) {
      Serial.print(' ');
    }
    Serial.println("|");
    if (lastPrintWasSmall) {
      Serial.print("| ");
      Serial.print(lastLines[1]);
      for (int i = 0; i < boxLength - int(lastLines[0].length()) - 1; i++) {
        Serial.print(' ');
      }
      Serial.println("|");
    }
    Serial.print("+");
    for (int i = 0; i < boxLength; i++) {
      Serial.print('-');
    }
    Serial.println("+");
  }

  void print(String text) {
#ifndef NO_OLED
    char tempCharBuffer[20];
    text.toCharArray(tempCharBuffer, 20);
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_helvR14_tr);
    u8g2.setFontPosBaseline();
    u8g2.drawStr(0, 20, tempCharBuffer);
    u8g2.sendBuffer();
#endif
    lastPrintWasSmall = false;
    lastLines[0] = text;
    lastLines[1] = "";
    printToSerial();
  }

  void print(int number) {
    print(String(number));
  }

  void print(long number) {
    print(String(number));
  }

  void print(unsigned long number) {
    print(String(number));
  }

  void print(double number) {
    print(String(number));
  }

  void print(String label, String value) {
    String text = label + " " + value;
    print(text);
  }

  void print(String label, int number) {
    print(label, String(number));
  }

  void print(String label, long number) {
    print(label, String(number));
  }

  void print(String label, unsigned long number) {
    print(label, String(number));
  }

  void print(String label, double number) {
    print(label, String(number));
  }

  void printSmallLine(String text, int line) {  // line=0: top & line=1: bottom
#ifndef NO_OLED
    char tempCharBuffer[30];
    text.toCharArray(tempCharBuffer, 30);
    if (lastPrintWasSmall) {
      // just clear the line we're going to print on
      u8g2.setDrawColor(0);
      u8g2.drawBox(0, 16 * line, 128, 16);
      u8g2.setDrawColor(1);
    } else {
      // clear the entire screen
      u8g2.clearBuffer();
    }
    u8g2.setFont(u8g2_font_helvR10_tr);
    u8g2.setFontPosBottom();
    int drawPos = line ? 32 : 15;
    u8g2.drawStr(0, drawPos, tempCharBuffer);
    u8g2.sendBuffer();
#endif
    lastLines[line] = text;
    if (!lastPrintWasSmall) {
      lastLines[1 - line] = "";
    }
    lastPrintWasSmall = true;
    printToSerial();
  }

  void printTop(String text) {
    printSmallLine(text, 0);
  }

  void printTop(int number) {
    printSmallLine(String(number), 0);
  }

  void printTop(long number) {
    printSmallLine(String(number), 0);
  }

  void printTop(unsigned long number) {
    printSmallLine(String(number), 0);
  }

  void printTop(double number) {
    printSmallLine(String(number), 0);
  }

  void printTop(String label, String value) {
    String text = label + " " + value;
    printSmallLine(text, 0);
  }

  void printTop(String label, int number) {
    printTop(label, String(number));
  }

  void printTop(String label, long number) {
    printTop(label, String(number));
  }

  void printTop(String label, unsigned long number) {
    printTop(label, String(number));
  }

  void printTop(String label, double number) {
    printTop(label, String(number));
  }

  void printBottom(String text) {
    printSmallLine(text, 1);
  }

  void printBottom(int number) {
    printSmallLine(String(number), 1);
  }

  void printBottom(long number) {
    printSmallLine(String(number), 1);
  }

  void printBottom(unsigned long number) {
    printSmallLine(String(number), 1);
  }

  void printBottom(double number) {
    printSmallLine(String(number), 1);
  }

  void printBottom(String label, String value) {
    String text = label + " " + value;
    printSmallLine(text, 1);
  }

  void printBottom(String label, int number) {
    printBottom(label, String(number));
  }

  void printBottom(String label, long number) {
    printBottom(label, String(number));
  }

  void printBottom(String label, unsigned long number) {
    printBottom(label, String(number));
  }

  void printBottom(String label, double number) {
    printBottom(label, String(number));
  }

  void clear() {
    print("");
  }
};

OledClass OLED;

void initializeOLED() {
#ifndef NO_OLED
  u8g2.begin();
#endif
  OLED.clear();
}


/* * * * * * * * * * * * * * * * * * *

  The code below creates the following
  command:
  
  VolumeSensor.read()
  
 * * * * * * * * * * * * * * * * * * */
class VolumeSensorClass {
private:

#ifndef NO_VOLUMESENSOR
  const int sampleWindow = 50;  // Sample window width in mS (50 mS = 20Hz)
  unsigned int sample;
#endif

#ifdef HW_VERSION2
  const int VOLUMESENSOR = 2;  // analog input
#else
  const int VOLUMESENSOR = 0;              // analog input
#endif

public:

  VolumeSensorClass() {
    pinMode(VOLUMESENSOR, INPUT);
  }

  //Method returns volume.
  int read() {
#ifdef NO_VOLUMESENSOR
    int sensorValue = analogRead(VOLUMESENSOR);
    if (sensorValue != 0) {
      return sensorValue;
    }
#else
    unsigned long startMillis = millis();  // Start of sample window
    unsigned int peakToPeak = 0;           // peak-to-peak level

    unsigned int signalMax = 0;
    unsigned int signalMin = 1024;

    // collect data for 50 mS and then plot data
    while (millis() - startMillis < sampleWindow) {
      sample = analogRead(VOLUMESENSOR);
      if (sample < 1024)  // toss out spurious readings
      {
        if (sample > signalMax) {
          signalMax = sample;  // save just the max levels
        } else if (sample < signalMin) {
          signalMin = sample;  // save just the min levels
        }
      }
    }
    peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
    //  double volts = ((peakToPeak * 5) / 1024) * 0.707;  // convert to RMS voltage
    //  double first = log10(volts/0.00631)*20;
    //  double second = first + 94 - 44 - 25;
    return ((20. * log(10)) * (peakToPeak / 5.) / 100) + 15;  //convert to dB
#endif
  }
};

VolumeSensorClass VolumeSensor;


/* * * * * * * * * * * * * * * * * * *

  The code below creates the following
  command:

  playTone( frequency, duration )   // Play a tone on the buzzer. 
                                    // The frequency detemines how high the tone wil sound.
                                    //    A frequency of 0 produces silence.
                                    //    See https://pages.mtu.edu/~suits/notefreqs.html for 
                                    //    frequencies of musical notes.
                                    // Duration specifies the how long the note will sound (in
                                    //    milliseconds).
                                    // In contrast to the tone() function in Arduino library, this 
                                    //    function does not interfere with PWM on digital pins. But 
                                    //    it will only return after the duretion is complete, like delay().
    
* * * * * * * * * * * * * * * * * * */

void playTone(int frequency, int duration) {
  if (frequency == 0) {  // PAUSE
    delay(duration);
  } else {
    long wavePeriodInMicroSecs = 1.0 / frequency * 1000 * 1000;
    const long pauseDuration = 10000;
    for (long i = 0; i < duration * 1000L - pauseDuration; i += wavePeriodInMicroSecs) {
      digitalWrite(BUZZER, HIGH);
      delayMicroseconds(wavePeriodInMicroSecs / 2);
      digitalWrite(BUZZER, LOW);
      delayMicroseconds(wavePeriodInMicroSecs / 2);
    }
    delayMicroseconds(pauseDuration);  // very short pause so the listener can tell notes apart.
  }
}

#endif  // S4D_BREADBOARD_H