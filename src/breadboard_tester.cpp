 /* * * * * * * * * * * * * * * * * * * * *
    
  How to test the breadboard:

  - Press buttons to turn the LEDs on or off.
  - Turn the potmeter to dim the LEDs.

 * * * * * * * * * * * * * * * * * * * * */

#include "s4d_breadboard.h"

// frequencies of musical notes taken from: https://pages.mtu.edu/~suits/notefreqs.html
const int REST = 0; 
const int NOTE_G_low = 196; 
const int NOTE_C = 262;
const int NOTE_E = 330;
const int NOTE_G = 392;

bool LEDsRunning = false;

void showSensorValues();
void printSensorsToSerial();
void showButtonStates();
void animateLEDs();
void simpleMelody();
void print4chars(int);
void buttonPressed();
bool isPressed(int);
bool bothButtonsPressed();
void switchToLED(int);


void setup() {
  initializeBreadboard();
  Serial.println("setup");
  OLED.print("running \"setup\"");
  delay(750);
  OLED.printTop("Press button 1 to");
  OLED.printBottom("turn LEDs on.");
  simpleMelody();
  delay(200);
  OLED.print("starting \"loop\"");
  delay(750);
}

void loop() {
  showButtonStates();
  showSensorValues();
  animateLEDs();
  printSensorsToSerial();
}

void showSensorValues() {
  OLED.printTop("potentiometer:", analogRead(POTENTIOMETER));
  OLED.printBottom("magnet sensor:", analogRead(MAGNETSENSOR));
}

void printSensorsToSerial() {
  // SL, 2023-02-13
  //  Removed spaces to make output Arduino IDE – Serial Plotter compatible.
  //  See: https://www.diyrobocars.com/2020/05/04/arduino-serial-plotter-the-missing-manual/

  Serial.print("button-1:");      Serial.print( Button.button1Pressed() /* == HIGH ? "HIGH" : "LOW" */); Serial.print( "\t" );
  Serial.print("button-2:");      Serial.print( Button.button2Pressed() /* == HIGH ? "HIGH" : "LOW" */); Serial.print( "\t" );
  Serial.print("potmeter:");      Serial.print( analogRead(POTENTIOMETER) );                          Serial.print( "\t" );
  Serial.print("magnet-sensor:"); Serial.print( analogRead(MAGNETSENSOR) );                           Serial.println();
  delay(50);
}

void print4chars(int value) {
  String result = String(value);
  if(value < 10)   { Serial.print(" "); } 
  if(value < 100)  { Serial.print(" "); } 
  if(value < 1000) { Serial.print(" "); } 
  Serial.print(result);
}

void showButtonStates() {
  if( bothButtonsPressed() ) {
    playTone(NOTE_G ,20);
    OLED.print("Both buttons");
    delay(20);
    // wait for any button release
    while( bothButtonsPressed() ) { 
      /* do nothing except: */ 
      printSensorsToSerial();
    }
  }
  else if( isPressed(1) ) {
    playTone(NOTE_G, 20);
    LEDsRunning = true;
    animateLEDs();
    OLED.print("LEDs on");
    delay(20);
    // wait for button 1 release
    while( isPressed(1) ) { 
      /* do nothing except: */ 
      printSensorsToSerial();
    }
    playTone(NOTE_C ,20);
  }
  else if( isPressed(2) ) {
    playTone(NOTE_G, 20);
    LEDsRunning = false;
    OLED.print("LEDs off");
    animateLEDs();
    delay(20);
    // wait for button 2 release
    while( isPressed(2) ) { 
      /* do nothing except: */ 
      printSensorsToSerial();
    }
    playTone(NOTE_C ,20);
  }
}

bool isPressed(int buttonPin) {
  bool pressed = false;
  if(buttonPin == 1){
    if(Button.button1Pressed() == 1){
      pressed = true;
    }
  }
  else{
    if(Button.button2Pressed() == 1){
      pressed = true;
    }
  }
  return pressed;
}

bool bothButtonsPressed() {
  bool pressed = false;
  if(Button.button1Pressed() == 1 && Button.button2Pressed() == 1){
    pressed = true;
  }
  return pressed;
}

const int LED_ALL  = 100;
const int LED_NONE = 101;

void switchToLED( int ledPin ) {
  int brightness = analogRead(POTENTIOMETER) / 4;
  // SL, 2023-02-13: keep minimum brightness, otherwise leds may seem to fail
  brightness = max(brightness, 127);

  analogWrite(LED_GREEN,  ledPin==LED_GREEN  || ledPin==LED_ALL ? brightness : 0 );
  analogWrite(LED_BLUE,   ledPin==LED_BLUE   || ledPin==LED_ALL ? brightness : 0 );
  analogWrite(LED_YELLOW, ledPin==LED_YELLOW || ledPin==LED_ALL ? brightness : 0 );
  analogWrite(LED_RED,    ledPin==LED_RED    || ledPin==LED_ALL ? brightness : 0 );
}

void animateLEDs() {
  if( !LEDsRunning ) {
    switchToLED(LED_NONE);
    return;
  }
  static int prevPhase = 0;
  long phase = (millis() / 150) % 6;
  if( phase == prevPhase ) {
    return;
  }
  prevPhase = phase;
  switch(phase) {
    case 0:
      switchToLED( LED_BLUE );
      break;
    case 1:
    case 5:
      switchToLED( LED_GREEN );
      break;
    case 2:
    case 4:
      switchToLED( LED_YELLOW );
      break;
    case 3:
      switchToLED( LED_RED );
      break;
    default:
      switchToLED(LED_ALL);
  }
}

void simpleMelody() {
  playTone( NOTE_G_low, 120 );
  playTone( REST,        30 );
  playTone( NOTE_C,     120 );
  playTone( REST,        30 );
  playTone( NOTE_E,     150 );
  playTone( NOTE_G,     300 );
  playTone( REST,        30 );
  playTone( NOTE_E,     120 );
  playTone( NOTE_G,     750 );
}
