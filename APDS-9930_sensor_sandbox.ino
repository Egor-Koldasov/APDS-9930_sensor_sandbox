
#include <Wire.h>

#define I2C_ADDR 57 // APDS-9930 device I2C address

// protocol
// command [7] type [6 5] register/special function [4 3 2 1 0]
#define COMMAND_MASK 10000000 // 1
#define AUTO_INCREMENT_MASK 00100000 // 01
#define REGISTER_MASK (COMMAND_MASK | AUTO_INCREMENT_MASK)

// registers [default value]
#define ENABLE_REG 0 // power on, setting features r/w [0]
// timings, ms value is calculated from byte by [2.73 ms * (256 – byte)]
#define ATIME_REG 1 // ALS integration time r/w [255]
#define PTIME_REG 2 // Prox inegration time r/w [255]
#define WTIME_REG 3 // wait time  r/w [255]
#define CONFIG_REG 0x0d // config (timings)
#define CONTROL_REG 0x0f // LEDs and gain
#define ID_REG 18 // device id  r
#define STATUS_REG 19 // device status  [0]

/*
In order to reject 50/60-Hz ripple strongly present in fluorescent lighting, the integration time needs to be programmed
in multiples of 10 / 8.3 ms or the half cycle time. Both frequencies can be rejected with a programmed value of 50 ms
(ATIME = 0xED) or multiples of 50 ms (i.e. 100, 150, 200, 400, 600).
 */
#define ALSIT 50 // ALS integration time     ALSIT = 2.73 ms * (256 – ATIME)
#define ATIME ((int) (256 - ALSIT / 2.73)) // ALS Timing register value      ATIME = 256 – ALSIT / 2.73 ms

#define INT // interupt
#define SAI // sleep after interrupt
#define WEN 0 // wait enabled
#define WTIME 0xff // wait time
#define WLONG 0  // Wait Long. When asserted, the wait cycles are increased by a factor 12x from that programmed in the WTIME register.

#define PEN 0 // Prox enabled

// Abmient Light Sensor
#define AEN 1 // ALS enabled
#define AGAIN 0 // ALS gain, 2 bits     1, 8, 16, or 120
// ALS gain level. When asserted, the 1× and 8× ALS gain (AGAIN) modes are scaled by 0.16.
// Otherwise,AGAIN is scaled by 1. Do not use with AGAIN greater than 8×.
#define AGL 0
#define Ch0DATAL 0x14 // Ch0 lower byte
#define Ch0DATAH 0x15 // Ch0 upper byte
#define Ch1DATAL 0x16 // Ch1 lower byte
#define Ch1DATAH 0x17 // Ch1 upper byte

#define LUX_DF 52 // device factor      52 for APDS-9930
// Coefficients
#define LUX_GA 0.49 // Glass (or Lens) Attenuation Factor 
#define LUX_B 1.862
#define LUX_C 0.746
#define LUX_D 1.291

#define PDL 0 // Proximity drive level. When asserted, the proximity LDR drive current is reduced by 9.
#define PGAIN 0 // Prox gain, 2 bits     1x, 2x, 4x, or 8x

char aGain = AGAIN;

void callRegister(char reg) {
  Wire.beginTransmission(I2C_ADDR);
  Wire.write(REGISTER_MASK | reg);
}

void endTransmissionSafe() {
  char transmissionRes = Wire.endTransmission();
  if(transmissionRes != 0) {
    Serial.println(String("Wire.endTransmission error: ") + String(transmissionRes, HEX));
  }
}

void readRegisterByte(char reg, char &output) {
  callRegister(reg);
  endTransmissionSafe();
  Wire.requestFrom(I2C_ADDR, 1);
  while (Wire.available()) {
    output = (char) Wire.read();
  }
}

void writeRegisterByte(char reg, char input) {
  callRegister(reg);
  Wire.write(input);
  endTransmissionSafe();
}

void checkRegister(char reg, String label) {
  char id;
  readRegisterByte(reg, id);
  Serial.println("REG " + label + ": " + String(id, HEX) + "[" + String(id, BIN) + "]");
}

void updateControl() {
  writeRegisterByte(CONTROL_REG, 0b1110 | (PGAIN << 2) | aGain); 
}

void startI2C() {
  Wire.begin();
  checkRegister(ID_REG, "id");
  writeRegisterByte(ENABLE_REG, 0);
  writeRegisterByte(ATIME_REG, ATIME);
  writeRegisterByte(WTIME_REG, WTIME);
  writeRegisterByte(CONFIG_REG, ((AGL << 2) | (WLONG << 1) | PDL));
  updateControl();
  writeRegisterByte(ENABLE_REG, (WEN << 3) | (PEN << 2) | (AEN << 1) | 1);
  checkRegister(STATUS_REG, "status");
  checkRegister(STATUS_REG, "status");
}

void setup() {
  Serial.begin(9600);
  startI2C();
}

double getAGainValue() {
  int values[4] = {1, 8, 16, 120};
  int gainValue = values[aGain];
  return AGL ? gainValue * 0.16 : gainValue;
}

double calculateLux(int ch0, int ch1) {
  double IAC1 = ch0 - LUX_B * ch1;
  double IAC2 = LUX_C * ch0 - LUX_D * ch1;
  double IAC = max(max(IAC1, IAC2), 0);
  double LPC = LUX_GA * LUX_DF / (ALSIT * getAGainValue());
  double lux = IAC * LPC;
  return lux;
}

long bindNumber(long number, long minValue, long maxValue) {
  return max(minValue, min(maxValue, number));
}

long mapSafe(long x, long in_min, long in_max, long out_min, long out_max) {
  return (bindNumber(x, in_min, in_max) - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void loop() {
  char ch0L, ch0H, ch1L, ch1H;
  readRegisterByte(Ch0DATAL, ch0L);
  readRegisterByte(Ch0DATAH, ch0H);
  readRegisterByte(Ch1DATAL, ch1L);
  readRegisterByte(Ch1DATAH, ch1H);

  unsigned int ch0 = ch0L + 256 * ch0H;
  unsigned int ch1 = ch1L + 256 * ch1H;
  double lux = calculateLux(ch0, ch1);

  unsigned int ch0A = mapSafe(ch0, 0, 50, 0, 255);
  unsigned int ch1A = mapSafe(ch1, 0, 50, 0, 255);
  unsigned int luxA = mapSafe(lux, 0, 50, 0, 255);
  analogWrite(11, ch0A);
  analogWrite(10, ch1A);
  analogWrite(9, luxA);
  unsigned int aGainInput = analogRead(A0);
  int aGainInputMap = map(aGainInput, 0, 1023, 0, 3);
  if (aGainInputMap != aGain) {
    aGain = aGainInputMap;
    updateControl();
  }
  Serial.println("ch0: " + String(ch0, HEX) + "\tch1: " + String(ch1, HEX) + "\t aGain: " + String(aGain, DEC) + "\tlux: " + String(lux, DEC));

}
