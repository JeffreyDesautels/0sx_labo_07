// Fichier : ViseurAutomatique.h

#pragma once


#include <AccelStepper.h>


enum EtatViseur {

  INACTIF,

  SUIVI,

  REPOS

};


class ViseurAutomatique {

public:

  // Constructeur

  // p1 à p4 : broches IN1 à IN4 du ULN2003

  // distanceRef : référence à la distance détectée

  ViseurAutomatique(int p1, int p2, int p3, int p4, float& distanceRef);


  // Doit être appelée continuellement dans loop()

  void update();


  // Régle l’angle minimal (position la plus à gauche)

  void setAngleMin(float angle);


  // Régle l’angle maximal (position la plus à droite)

  void setAngleMax(float angle);


  // Régle le nombre de pas par tour du moteur

  void setPasParTour(int steps);


  // Définit la distance minimale à partir de laquelle le viseur commence à suivre

  void setDistanceMinSuivi(float distanceMin);

  // Retourne la distance minimale à partir de laquelle le viseur commence à suivre

  float getDistanceMinSuivi();

  // Définit la distance maximale jusqu'où le viseur peut suivre

  void setDistanceMaxSuivi(float distanceMax);

  // Retourne la distance maximale jusqu'où le viseur peut suivre

  float getDistanceMaxSuivi();


  // Retourne l’angle actuel du viseur

  float getAngle() const;


  // Active le viseur en le mettant en état repos

  void activer();


  // Désactive le viseur en le mettant en état inactif

  void desactiver();


  // Retourne l’état actuel du viseur sous forme de texte

  const char* getEtatTexte() const;


private:

  AccelStepper _stepper;


  float& _distance;


  float _angleMin = 0.0;

  float _angleMax = 180.0;

  int _stepsPerRev = 2038;

  float _minStep = 0;

  float _maxStep = 0;


  float _distanceMinSuivi = 30.0;

  float _distanceMaxSuivi = 60.0;


  float _currentPosition = 0;


  unsigned long _currentTime = 0;


  EtatViseur _etat = INACTIF;

  // --- Indicateurs de transition ---

  bool _transitionRepos;

  bool _transitionSuivi;

  // bool _activerFlag = false;

  // bool _desactiverFlag = false;


  // États

  void _inactifState(unsigned long cT);

  void _suiviState(unsigned long cT);

  void _reposState(unsigned long cT);

  long _angleEnSteps(float angle) const;
};