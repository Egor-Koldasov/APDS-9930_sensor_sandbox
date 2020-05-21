#include "colors_16.h"
#include "Adafruit_ILI9340.h"
#include "Adafruit_GFX.h"

#define CS   10
#define DC   8
#define RESET  9

#define COLOR_LABEL COLOR_EMBER
#define COLOR_VALUE COLOR_GREEN

Adafruit_ILI9340 screen = Adafruit_ILI9340(CS, DC, RESET);

unsigned int fontSize = 2;

void display_setup() {
  screen.begin();
  screen.fillScreen(COLOR_BLACK);
  screen.setRotation(0);
  screen.setTextSize(fontSize);
}

unsigned int xPos;
unsigned int yPos;
unsigned int yPosI;

void readyCursor() {
  screen.setCursor(xPos, yPosI * 8 * fontSize);
}

void display_resetCursor() {
  unsigned int xPosA = 0x3c; //analogRead(A2);
  unsigned int yPosA = 6; //analogRead(A3);
  if (xPosA != xPos || yPosA != yPos) {
    xPos = xPosA;
    yPos = yPosA;
    screen.fillScreen(COLOR_BLACK);
  }
  yPosI = yPos;
  readyCursor();
  //display_println("xPos", xPos, "yPos", yPos);
}

void display_print(String label, String value) {
  screen.setTextColor(COLOR_LABEL, COLOR_BLACK);
  screen.print(label + ": ");
  screen.setTextColor(COLOR_VALUE, COLOR_BLACK);
  screen.print(" " + value);
}

void display_println(String label, String value) {
  readyCursor();
  display_print(label, value);
  screen.print("\n\n");
  yPosI += 2;
}
void display_println(String label1, String value1, String label2, String value2) {
  readyCursor();
  display_print(label1, value1);
  screen.print("\n");
  yPosI++;
  readyCursor();
  display_print(label2, value2);
  screen.print("\n\n");
  yPosI += 2;
}
void display_println(String label1, int value1, String label2, int value2) {
  display_println(label1, String(value1, HEX), label2, String(value2, HEX));
}
