/*

MIT License
Copyright (c) 2025 Hochschule Schmalkalden Robotics Lab

Authors: Venkata Prashanth Uppalapati <venkataprashanth.u@gmail.com>

*/

#include "PID.h"

void PID::init(float kp, float ki, float kd, int minlimit, int maxlimit,
               float filterAlpha) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->minlimit = minlimit;
  this->maxlimit = maxlimit;
  velResponseLimit = maxlimit * 0.05; // Taking 5% steady state error
  this->filterAlpha = filterAlpha;
}

int PID::update(float velReq, float velActual, float dt, bool coastFlag) {
  unsigned long currentTime = micros();

  // if  motor stops to respond when a PID input is given conditions like stall,
  // emergency stop and power loss
  if (!((abs(velActual) < (velResponseLimit * dt)) && (abs(velReq) > 0))) {
    noResponseTime = currentTime;
  }

  float velFiltered =
      filterAlpha * (velActual) + (1 - filterAlpha) * filterPrevValue;
  filterPrevValue = velFiltered;

  float error = velReq - velActual;

  if ((currentTime - noResponseTime) > 1000000) {
    errorIntegral = 0;
    errorDifferential = 0;
  } else {
    errorIntegral = errorIntegral + error * (dt);
    errorDifferential = (error - errorPrev) / (dt);
  }

  float value =
      (kp * error) + (ki * (errorIntegral)) + (kd * (errorDifferential));
  errorPrev = error;
  errorIntegral = constrain(errorIntegral, minlimit, maxlimit);
  if (coastFlag == 1) {
    errorIntegral = 0;
    errorDifferential = 0;
    return 0;
  } else {
    return constrain(value, minlimit, maxlimit);
  }
}