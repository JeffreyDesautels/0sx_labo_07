#include "Alarm.h"
#include <Arduino.h>

Alarm::Alarm(int rPin, int gPin, int bPin, int buzzerPin, float* distancePtr)
  : _rPin(rPin), _gPin(gPin), _bPin(bPin), _buzzerPin(buzzerPin), _distance(distancePtr) {
  pinMode(_rPin, OUTPUT);
  pinMode(_gPin, OUTPUT);
  pinMode(_bPin, OUTPUT);
  pinMode(_buzzerPin, OUTPUT);
}

void Alarm::update() {
  _currentTime = millis();

  switch (_state) {
    case OFF:
      _offState();
      break;

    case WATCHING:
      _watchState();
      break;

    case ON:
      _onState();
      break;

    case TESTING:
      _testingState();
      break;
  }
}

void Alarm::setColourA(int r, int g, int b) {
  _colA[0] = r;
  _colA[1] = g;
  _colA[2] = b;
}

void Alarm::setColourB(int r, int g, int b) {
  _colB[0] = r;
  _colB[1] = g;
  _colB[2] = b;
}

void Alarm::setVariationTiming(unsigned long ms) {
  _variationRate = ms;
}

void Alarm::setDistance(float d) {
  _distanceTrigger = d;
}

void Alarm::setTimeout(unsigned long ms) {
  _timeoutDelay = ms;
}

void Alarm::turnOff() {
  _turnOffFlag = true;
}

void Alarm::turnOn() {
  _turnOnFlag = true;
}

void Alarm::test() {
  _state = TESTING;
  _testStartTime = _currentTime;
}

void Alarm::_offState() {
  bool transition = _turnOnFlag;

  if (transition) {
    _turnOnFlag = false;
    _state = WATCHING;
  }
}

void Alarm::_watchState() {
  bool transitionOn = (*_distance) < _distanceTrigger;
  bool transitionOff = _turnOffFlag;

  if (transitionOn) {
    _state = ON;
  } else if (transitionOff) {
    _turnOffFlag = false;
    _state = OFF;
  }
}

void Alarm::_onState() {
  bool transitionWatching = (*_distance) >= _distanceTrigger && (_currentTime - _lastDetectedTime) > _timeoutDelay;
  bool transitionOff = _turnOffFlag;

  if (transitionWatching) {
    _state = WATCHING;
  } else if (transitionOff) {
    _turnOffFlag = false;
    _state = OFF;
  }
}

void Alarm::_testingState() {
  bool transitionOff = (_currentTime - _testStartTime) > _timeoutDelay;

  if (transitionOff) {
    _state = OFF;
  }
}
