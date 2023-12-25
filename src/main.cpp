#include <Arduino.h>
#include <ESP32Servo.h>
#include "TimedBlink.h"
#include "CRSFforArduino.hpp"

const uint8_t RxPin = 4;
const uint8_t TxPin = 6;

CRSFforArduino crsf = CRSFforArduino(RxPin, TxPin);
const int channelCount = crsfProtocol::RC_CHANNEL_COUNT;
const int armChannel = 5;
const int steeringChannel = 1;
const int driveChannel = 2;
const int lightChannel = 3;
const int turnLightChannel = 4;
const int hazLightChannel = 6;
const int breakChannel = 7;

const int boardStatusLedPin = 15;

const int driveChannelPin = 1;
const int steeringChannelPin = 2;
const int headLightPin = 40;
const int tailLightPin = 38;
const int leftTurnlightPin = 36;
const int rightTurnlightPin = 34;
const int breakLightPin = 21;
const int reverseLightPin = 17;

const int minServoTime = 988;
const int maxServoTime = 2012;
const int midServoTime = minServoTime + (maxServoTime - minServoTime) / 2;

Servo streetingServo;
Servo driveServo;
TimedBlink leftTurnLightBlink(leftTurnlightPin);
TimedBlink rightTurnLightBlink(rightTurnlightPin);
TimedBlink headLightBlink(headLightPin);
TimedBlink tailLightBlink(tailLightPin);
TimedBlink boardStatusBlink(boardStatusLedPin);

void updateTurnSignals(TimedBlink &signalLight, bool enabled);
void printChannelValue(uint8_t channelId);
uint16_t readChannel(uint8_t channelId);
void updateServo(uint8_t channelId, Servo &servo);
void updateMainlights();

void setup()
{
  pinMode(boardStatusLedPin, OUTPUT);

  Serial.begin(460800);

  Serial1.begin(420000, SERIAL_8N1, RxPin, TxPin);
  while (!crsf.begin())
  {
    Serial.println("CRSF for Arduino initialization failed!");
    delay(100);
    digitalWrite(boardStatusLedPin, digitalRead(boardStatusLedPin) == LOW ? HIGH : LOW);
  }

  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  pinMode(driveChannelPin, OUTPUT);
  pinMode(steeringChannelPin, OUTPUT);
  driveServo.setPeriodHertz(50);
  driveServo.attach(driveChannelPin, minServoTime, maxServoTime);
  driveServo.writeMicroseconds(midServoTime);
  streetingServo.setPeriodHertz(50);
  streetingServo.attach(steeringChannelPin, midServoTime - 200, midServoTime + 200);
  streetingServo.writeMicroseconds(midServoTime);

  pinMode(headLightPin, OUTPUT);
  digitalWrite(headLightPin, LOW);
  pinMode(tailLightPin, OUTPUT);
  digitalWrite(tailLightPin, LOW);
  pinMode(leftTurnlightPin, OUTPUT);
  digitalWrite(leftTurnlightPin, LOW);
  pinMode(rightTurnlightPin, OUTPUT);
  digitalWrite(rightTurnlightPin, LOW);
  pinMode(breakLightPin, OUTPUT);
  digitalWrite(breakLightPin, LOW);
  pinMode(reverseLightPin, OUTPUT);
  digitalWrite(reverseLightPin, LOW);

  Serial.println("Ready");
  digitalWrite(boardStatusLedPin, LOW);

  boardStatusBlink.setBlinkCount(3);
  boardStatusBlink.blink(100, 100);
  boardStatusBlink.blinkSync();
}

volatile bool lastArmedState = false;
void loop()
{
  crsf.update();
  uint16_t armChannelValue = readChannel(armChannel);
  bool armed = armChannelValue > 1500;
  headLightBlink.loop();
  boardStatusBlink.loop();
  tailLightBlink.loop();
  leftTurnLightBlink.loop();
  rightTurnLightBlink.loop();
  // Serial.print("leftTurnLightBlink: ");
  // Serial.print(leftTurnLightBlink.isBlinking());
  // Serial.print(", rightTurnLightBlink: ");
  // Serial.print(rightTurnLightBlink.isBlinking());
  // Serial.println("");
  if (armed)
  {
    if (!lastArmedState)
    {
      boardStatusBlink.blink(100, 100);
      boardStatusBlink.setBlinkCount(2);
      headLightBlink.blink(100, 100);
      headLightBlink.setBlinkCount(2);
    }
    lastArmedState = true;
    updateServo(steeringChannel, streetingServo);
    updateServo(driveChannel, driveServo);
    updateMainlights();

    uint16_t driveInput = readChannel(driveChannel);

    if (driveInput < midServoTime + 50)
    {
      digitalWrite(breakLightPin, HIGH);
    }
    else
    {
      digitalWrite(breakLightPin, LOW);
    }

    if (driveInput < midServoTime - 50)
    {
      digitalWrite(reverseLightPin, HIGH);
    }
    else
    {
      digitalWrite(reverseLightPin, LOW);
    }

    uint16_t turnChannelValue = readChannel(turnLightChannel);
    uint16_t hazChannelValue = readChannel(hazLightChannel);
    if (hazChannelValue > 1300)
    {
      if (!leftTurnLightBlink.isBlinking() || !rightTurnLightBlink.isBlinking())
      {
        leftTurnLightBlink.blinkOff();
        rightTurnLightBlink.blinkOff();
        leftTurnLightBlink.blink(500, 500);
        rightTurnLightBlink.blink(500, 500);
      }
    }
    else if (turnChannelValue < 1300)
    {
      if (!leftTurnLightBlink.isBlinking())
      {
        leftTurnLightBlink.blinkOff();
        leftTurnLightBlink.blink(500, 500);
      }
    }
    else if (turnChannelValue > 1700)
    {
      if (!rightTurnLightBlink.isBlinking())
      {
        rightTurnLightBlink.blinkOff();
        rightTurnLightBlink.blink(500, 500);
      }
    }
    else if (abs(turnChannelValue - midServoTime) < 200)
    {
      leftTurnLightBlink.blinkOff();
      rightTurnLightBlink.blinkOff();
    }
  }
  else
  {
    if (lastArmedState)
    {
      boardStatusBlink.blink(100, 100);
      boardStatusBlink.setBlinkCount(3);
      digitalWrite(headLightPin, LOW);
      digitalWrite(tailLightPin, LOW);
      headLightBlink.blink(100, 100);
      headLightBlink.setBlinkCount(3);
    }
    lastArmedState = false;
    uint16_t steeringInput = readChannel(steeringChannel);
    uint16_t driveInput = readChannel(driveChannel);
    if (abs(steeringInput - midServoTime) > 50 || abs(driveInput - midServoTime) > 50)
    {
      headLightBlink.blink(150, 150);
      headLightBlink.setBlinkCount(2);
      tailLightBlink.blink(150, 150);
      tailLightBlink.setBlinkCount(2);
    }

    streetingServo.writeMicroseconds(midServoTime);
    driveServo.writeMicroseconds(midServoTime);
  }
}

uint16_t readChannel(uint8_t channelId)
{
  return crsf.rcToUs(crsf.getChannel(channelId));
}

void updateTurnSignals(TimedBlink &signalLight, bool enabled)
{
  if (enabled)
  {
    signalLight.blink(500, 500);
  }
  else
  {
    signalLight.blinkOff();
  }
  signalLight.loop();
}

volatile int mainLightsState = LOW;
void updateMainlights()
{
  uint16_t value = readChannel(lightChannel);
  if (value < 1000 && mainLightsState == HIGH)
  {
    Serial.println("Main lights off");
    digitalWrite(headLightPin, LOW);
    digitalWrite(tailLightPin, LOW);
    mainLightsState = LOW;
  }
  else if (value >= 1000 && mainLightsState == LOW)
  {
    Serial.println("Main lights on");
    digitalWrite(headLightPin, HIGH);
    digitalWrite(tailLightPin, HIGH);
    mainLightsState = HIGH;
  }
}

void updateServo(uint8_t channelId, Servo &servo)
{
  servo.writeMicroseconds(readChannel(channelId));
}

void printChannelValue(uint8_t channelId)
{
  Serial.print(channelId);
  Serial.print(":");
  Serial.print(readChannel(channelId));
  Serial.print("\t");
}
