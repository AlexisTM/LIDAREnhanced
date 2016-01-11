#include "KFilter.h"
#include <Arduino.h>

#define DEBUG false

void KFilter::begin(double _initialValue, double _initialSpeed, double _maximalSpeed, double _linearity, double _offset){
  outlierCount = 0;

  factors[0] = _linearity; // X, linearity
  factors[1] = _offset;// offset

  maximalSpeed = _maximalSpeed/100;

  lastPosition = _initialValue;
  lastTimestamp = micros();
  lastSpeed = _initialSpeed;
}

inline bool KFilter::isOutlier(double actualPosition, double predictedPosition, double dt){
  double maximalDeplacement = maximalSpeed*dt;
  #if DEBUG
  Serial.print("\t max dep:");
  Serial.print(maximalDeplacement);
  #endif 
  if(actualPosition < 10)
    return true;
  // If we are further than 1 meter from the predicted value
  if(abs(predictedPosition - actualPosition) > maximalDeplacement)
    return true;
  return false;
}

double KFilter::filter(double actualPosition, bool * isItAnOutlier){
  // Predict 
  // lastPosition 
  // lastTimestamp
  // lastSpeed
  // Max difference
  double actualSpeed, predictedPosition;
  long timestamp = micros();

  // Prediction 
  double dt = double(timestamp - lastTimestamp)/1000;
  predictedPosition = lastPosition + lastSpeed * dt;
  actualSpeed = (lastPosition - actualPosition)/dt;
  
  #if DEBUG
  Serial.print("\t dt ");
  Serial.print(dt);
  Serial.print("\t Last position: ");
  Serial.print(lastPosition);
  Serial.print("\t Predicted position: ");
  Serial.print(predictedPosition);
  Serial.print("\t Actual Speed: ");
  Serial.print(actualSpeed);
  #endif

  // Outlier detection
  if(isOutlier(actualPosition, predictedPosition, dt)){

    // Return predition ? or keep last position ? 
    /* 
    lastPosition = predictedPosition;
    actualSpeed = lastSpeed;
    lastTimestamp = timestamp;
    */
    *isItAnOutlier = true;
    outlierCount++;
    #if DEBUG
    Serial.print("\t OUTLIEEEEEEER: ");
    #endif
  } else {
    *isItAnOutlier = false;
    lastPosition = actualPosition;
    lastSpeed = actualSpeed;
    lastTimestamp = timestamp;
  }

  // Prepare for next step -- depends on the case Outlier or not
  return lastPosition;
}


