#include "ViseurAutomatique.h"
#include <Arduino.h>

#define MOTOR_INTERFACE_TYPE 4

ViseurAutomatique::ViseurAutomatique(int p1, int p2, int p3, int p4, float& distanceRef)
  : _stepper(MOTOR_INTERFACE_TYPE, p1, p3, p2, p4), _distance(distanceRef) {

  _stepper.setMaxSpeed(500);
  _stepper.setAcceleration(100);
  _stepper.setSpeed(500);
  _stepper.setCurrentPosition(0);
}

void ViseurAutomatique::update() {
  _currentTime = millis();

  switch (_etat) {
    case INACTIF:
      _inactifState(_currentTime);
      break;

    case SUIVI:
      _suiviState(_currentTime);
      break;

    case REPOS:
      _reposState(_currentTime);
      break;
  }
}

void ViseurAutomatique::setAngleMin(float angle) {
  _angleMin = angle;
}

void ViseurAutomatique::setAngleMax(float angle) {
  _angleMax = angle;
}

void ViseurAutomatique::setPasParTour(int steps) {
  _stepsPerRev = steps;
}

void ViseurAutomatique::setDistanceMinSuivi(float distanceMin) {
  _distanceMinSuivi = distanceMin;
}

void ViseurAutomatique::setDistanceMaxSuivi(float distanceMax) {
  _distanceMaxSuivi = distanceMax;
}

float ViseurAutomatique::getAngle() const {
  return (_stepper.currentPosition() * 360.0) / _stepsPerRev;
}

void ViseurAutomatique::activer() {
  _etat = REPOS;
}

void ViseurAutomatique::desactiver() {
  _etat = INACTIF;
}

const char* ViseurAutomatique::getEtatTexte() const {
  if (_etat == INACTIF) return "INACTIF"; // fort probable de changer pour return une lettre (char*)
  else if (_etat == SUIVI) return "SUIVI";
  else if (_etat == REPOS) return "REPOS";
}




void ViseurAutomatique::_inactifState(unsigned long cT) {
  _stepper.disableOutputs();
}

void ViseurAutomatique::_suiviState(unsigned long cT) {
  if(_stepper.distanceToGo() == 0) {
    _stepper.disableOutputs();
  }
}

void ViseurAutomatique::_reposState(unsigned long cT) {
}

long ViseurAutomatique::_angleEnSteps(float angle) const {
}