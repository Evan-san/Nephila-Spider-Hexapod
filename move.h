
#include <math.h>

// --- Neutral pose 
int neutralCoxa  = 90;
int neutralFemur = 90;   // 90
int neutralTibia = 45;   // 45

// --- Gait tuning parameters (ADJUST THESE ON THE REAL ROBOT) ---
int stepAngle   = 30;   // how far the coxa swings forward/back each step (deg)
int liftFemur   = 40;   // how much femur lifts the leg off the ground (deg)
int liftTibia   = 30;   // how much tibia curls to help lift (deg)
int stepDelay   = 200;  // ms — total time budget for ONE tripod half-step (swing+push)
int smoothSteps = 20;   // resolution of the interpolation; higher = smoother but slower to update

//     coxa "forward" to mean opposite rotation directions. ---
int coxaDir[6] = { 1, 1, 1, -1, -1, -1 };

// --- Turning direction split (tune independently from coxaDir) ---
int turnDir[6] = { -1, -1, -1, -1, -1, -1 };
int turnAngle  = 15; // how far each leg swings per turn step (deg)

// --- Track the last commanded pose of every leg so we can always
int currentCoxa[6], currentFemur[6], currentTibia[6];
bool poseInitialized = false;

void initPoseIfNeeded() {
  if (poseInitialized) return;
  for (int i = 0; i < 6; i++) {
    currentCoxa[i]  = neutralCoxa;
    currentFemur[i] = neutralFemur;
    currentTibia[i] = neutralTibia;
  }
  poseInitialized = true;
}

float lerp(float a, float b, float t) {
  return a + (b - a) * t;
}


void setLeg(int legNum, int coxa, int femur, int tibia) {
  currentCoxa[legNum - 1]  = coxa;
  currentFemur[legNum - 1] = femur;
  currentTibia[legNum - 1] = tibia;

  switch (legNum) {
    case 1: leg1(coxa, femur, tibia); break;
    case 2: leg2(coxa, femur, tibia); break;
    case 3: leg3(coxa, femur, tibia); break;
    case 4: leg4(coxa, femur, tibia); break;
    case 5: leg5(coxa, femur, tibia); break;
    case 6: leg6(coxa, femur, tibia); break;
  }
}


void tripodPhase(int swingLegs[3], int stanceLegs[3], int coxaOffset, int dirTable[6]) {
  initPoseIfNeeded();

  // Remember start pose for the 6 legs involved (some may not move, that's fine)
  int startCoxa[6], startFemur[6], startTibia[6];
  for (int i = 0; i < 6; i++) {
    startCoxa[i]  = currentCoxa[i];
    startFemur[i] = currentFemur[i];
    startTibia[i] = currentTibia[i];
  }

  int perStepDelay = max(2, stepDelay / smoothSteps);

  for (int s = 1; s <= smoothSteps; s++) {
    float t = (float)s / smoothSteps;

    // Sine arc: 0 -> 1 -> 0 Simple Inverse Kinematic half circle cycle
    float liftArc = sin(t * PI);
    int femurLifted = neutralFemur - (int)(liftFemur * liftArc);
    int tibiaLifted  = neutralTibia + (int)(liftTibia * liftArc);

    // Swinging tripod: coxa sweeps from -offset to +offset, foot lifts mid-swing
    for (int i = 0; i < 3; i++) {
      int leg = swingLegs[i];
      int targetCoxa = neutralCoxa + dirTable[leg - 1] * lerp(-coxaOffset, coxaOffset, t);
      setLeg(leg, targetCoxa, femurLifted, tibiaLifted);
    }


    for (int i = 0; i < 3; i++) {
      int leg = stanceLegs[i];
      int targetCoxa = neutralCoxa + dirTable[leg - 1] * lerp(coxaOffset, -coxaOffset, t);
      setLeg(leg, targetCoxa, neutralFemur, neutralTibia);
    }

    delay(perStepDelay);
  }
}

// One full walking cycle
void walkForwardStep() {
  int tripodA[3] = {1, 3, 5};
  int tripodB[3] = {2, 4, 6};
  tripodPhase(tripodA, tripodB, stepAngle, coxaDir); // A swings forward, B pushes
  tripodPhase(tripodB, tripodA, stepAngle, coxaDir); // B swings forward, A pushes
}

void walkBackwardStep() {
  int tripodA[3] = {1, 3, 5};
  int tripodB[3] = {2, 4, 6};
  tripodPhase(tripodA, tripodB, -stepAngle, coxaDir);
  tripodPhase(tripodB, tripodA, -stepAngle, coxaDir);
}

// --- Turning
void turnRightStep() {
  int tripodA[3] = {1, 3, 5};
  int tripodB[3] = {2, 4, 6};
  tripodPhase(tripodA, tripodB, turnAngle, turnDir);
  tripodPhase(tripodB, tripodA, turnAngle, turnDir);
}

void turnLeftStep() {
  int tripodA[3] = {1, 3, 5};
  int tripodB[3] = {2, 4, 6};
  tripodPhase(tripodA, tripodB, -turnAngle, turnDir);
  tripodPhase(tripodB, tripodA, -turnAngle, turnDir);
}

void turnRight(int steps) {
  for (int i = 0; i < steps; i++) turnRightStep();
}

void turnLeft(int steps) {
  for (int i = 0; i < steps; i++) turnLeftStep();
}

void walkForward(int steps) {
  for (int i = 0; i < steps; i++) walkForwardStep();
}

void walkBackward(int steps) {
  for (int i = 0; i < steps; i++) walkBackwardStep();
}


void standNeutralGroup(int legs[3]) {
  int startCoxa[3], startFemur[3], startTibia[3];
  for (int i = 0; i < 3; i++) {
    int leg = legs[i];
    startCoxa[i]  = currentCoxa[leg - 1];
    startFemur[i] = currentFemur[leg - 1];
    startTibia[i] = currentTibia[leg - 1];
  }

  int perStepDelay = max(2, stepDelay / smoothSteps);
  for (int s = 1; s <= smoothSteps; s++) {
    float t = (float)s / smoothSteps;

    // Same lift-arc shape as tripodPhase(): 0 -> 1 -> 0
    float liftArc = sin(t * PI);
    int femurLifted = neutralFemur - (int)(liftFemur * liftArc);
    int tibiaLifted = neutralTibia + (int)(liftTibia * liftArc);

    for (int i = 0; i < 3; i++) {
      int leg = legs[i];
      int targetCoxa = lerp(startCoxa[i], neutralCoxa, t);
      setLeg(leg, targetCoxa, femurLifted, tibiaLifted);
    }
    delay(perStepDelay);
  }

  // Make sure it lands exactly on neutral, feet planted
  for (int i = 0; i < 3; i++) {
    setLeg(legs[i], neutralCoxa, neutralFemur, neutralTibia);
  }
}


void standNeutral() {
  initPoseIfNeeded();

  int tripodA[3] = {1, 3, 5};
  int tripodB[3] = {2, 4, 6};
  standNeutralGroup(tripodA);
  standNeutralGroup(tripodB);
}