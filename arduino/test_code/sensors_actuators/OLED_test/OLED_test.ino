/*
 Based on the LiquidCrystal Library - Hello World
 
 Demonstrates the use a Winstar 16x2 OLED display.  These are similar, but
 not quite 100% compatible with the Hitachi HD44780 based LCD displays. 
 
 This sketch prints "Hello OLED World" to the LCD
 and shows the time in seconds since startup.
 
  The circuit:
 * LCD RS pin to digital pin A3
 * LCD R/W pin to digital pin A4
 * LCD Enable pin to digital pin A5
 * LCD D4 pin to digital pin A6
 * LCD D5 pin to digital pin A7
 * LCD D6 pin to digital pin A8
 * LCD D7 pin to digital pin A9

 There is no need for the contrast pot as used in the LCD tutorial
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 Library & Example Converted for OLED
 by Bill Earl 30 Jun 2012
 
 This example code is in the public domain.
 */

// include the library code:
#include <Adafruit_CharacterOLED.h>

// initialize the library with the numbers of the interface pins
Adafruit_CharacterOLED lcd(A3, A4, A5, A6, A7, A8, A9);

void setup() 
{
    // Print a message to the LCD.
  pinMode(A3, OUTPUT);     
  pinMode(A4, OUTPUT);   
  pinMode(A5, OUTPUT);   
  pinMode(A6, OUTPUT);   
  pinMode(A7, OUTPUT);   
  pinMode(A8, OUTPUT);   
  pinMode(A9, OUTPUT);   
  lcd.begin(16, 2);
  lcd.print("hello OLED World");
}

void loop() 
{
  // set the cursor to column 0, line 1
  // (note: line 1 is the second row, since counting begins with 0):
  lcd.setCursor(0, 1);
  // print the number of seconds since reset:
  lcd.print(millis()/1000);
 }

