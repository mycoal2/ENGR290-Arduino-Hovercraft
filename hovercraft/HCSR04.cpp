/*
  HCSR04 - Library for arduino, for HC-SR04 ultrasonic distance sensor.
  Created by Martin Sosic, June 11, 2016.
*/

#include "Arduino.h"
#include "HCSR04.h"

UltraSonicDistanceSensor::UltraSonicDistanceSensor(
        byte triggerPin, byte echoPin, unsigned short maxDistanceCm, unsigned long maxTimeoutMicroSec) {
    this->triggerPin = triggerPin;
    this->echoPin = echoPin;
    this->maxDistanceCm = maxDistanceCm;
    this->maxTimeoutMicroSec = maxTimeoutMicroSec;
    pinMode(triggerPin, OUTPUT); // 12 --> PB4
    pinMode(echoPin, INPUT);     // 8 --> PB0
}

float UltraSonicDistanceSensor::measure_distance_cm() {
    //Using the approximate formula 19.307°C results in roughly 343m/s which is the commonly used value for air.
    return measure_distance_cm(19.307);
}

float UltraSonicDistanceSensor::measure_distance_cm(float temperature) {
    unsigned long maxDistanceDurationMicroSec;

    // Make sure that trigger pin is LOW.
    digitalWrite(triggerPin, LOW); // 12 --> PB4
    _delay_us(2);
    // Hold trigger for 10 microseconds, which is signal for sensor to measure distance.
    digitalWrite(triggerPin, HIGH); // 12 --> PB4
    _delay_us(10);
    digitalWrite(triggerPin, LOW);
    

    float speedOfSoundInCmPerMicroSec = 0.03313 + 0.0000606 * temperature; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s

    // Compute max delay based on max distance with 25% margin in microseconds
    maxDistanceDurationMicroSec = 2.5 * maxDistanceCm / speedOfSoundInCmPerMicroSec;
    if (maxTimeoutMicroSec > 0) {
    	maxDistanceDurationMicroSec = min(maxDistanceDurationMicroSec, maxTimeoutMicroSec);
    }

    // Measure the length of echo signal, which is equal to the time needed for sound to go there and back.
      unsigned long durationMicroSec = pulseIn(echoPin, HIGH, maxDistanceDurationMicroSec); // can't measure beyond max distance
//    avg = avg/5;

    float distanceCm = durationMicroSec / 2.0 * speedOfSoundInCmPerMicroSec;

    return distanceCm;
}
