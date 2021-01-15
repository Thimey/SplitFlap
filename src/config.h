#pragma once

#include <Arduino.h>

const String WORD_SUB_TOPIC = "splitFlap1/word";
const String RESET_SUB_TOPIC = "splitFlap1/reset";

const int NUMBER_OF_SPLIT_FLAPS = 3;
const int MAX_SPLIT_FLAPS = 8;
const int NUMBER_OF_FLAPS = 50;
const int STEPS_PER_REVOLUTION = 200;

// Pinouts
const int SR_LATCH_PIN = 21;
const int SR_CLOCK_PIN = 23;
const int SR_DATA_PIN = 22;
const int SENSOR_PIN = 19;
const int DIR_PIN = 18;
const int ENABLE_PIN = 16;

const uint8_t flaps[NUMBER_OF_FLAPS] = {
    ' ',
    'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm',
    'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z',
    '0', '1', '2', '3', '4', '5', '6', '7', '8', '9',
    '.', ',', ';', ':', '?', '-', '!', '$', '%', '(', ')', '\'', '#'
};

const int PULSE_DELAY = 5000;

