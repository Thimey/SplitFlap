#pragma once

#include <Arduino.h>

const String DISPLAY_SUB_TOPIC = "splitFlap1/display";
const String RESET_SUB_TOPIC = "splitFlap1/reset";
const String DISABLE_MOTORS_TOPIC = "splitFlap1/disableMotors";
const String ENABLE_MOTORS_TOPIC = "splitFlap1/enableMotors";
const String GET_SENSOR_INPUT_TOPIC = "splitFlap1/getSensorInput";

// Be sure to adjust the driver stepper micro stepping pins to match
const int MICRO_STEPS = 16;

const int NUMBER_OF_SPLIT_FLAPS = 8;
const int MAX_SPLIT_FLAPS = 8;
const int NUMBER_OF_FLAPS = 50;
const int STEPS_PER_REVOLUTION = MICRO_STEPS * 200;
const int MAX_CHARACTER_DISPLAY_QUEUE = 20;
const int DEFAULT_DISPLAY_PAUSE_US = 4000000;
const unsigned long STEP_SPEED = 1000;

// Pinouts
const int SR_DRIVER_LATCH_PIN = 22; // YELLOW
const int SR_DRIVER_CLOCK_PIN = 23; // BLUE
const int SR_DRIVER_DATA_PIN = 21; // GREEN

const int SR_SENSOR_LOAD_PIN = 25;
const int SR_SENSOR_CLOCK_PIN = 14;
const int SR_SENSOR_DATA_PIN = 26;
const int SR_SENSOR_CLOCK_ENABLE_PIN = 27;

const int DIR_PIN = 18;
const int ENABLE_PIN = 16;

const uint8_t flaps[NUMBER_OF_FLAPS] = {
    ' ', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i',
    'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's',
    't', 'u', 'v', 'w', 'x', 'y', 'z', '0', '1', '2',
    '3', '4', '5', '6', '7', '8', '9', '(', ')', '.',
    ':', '\'', '=', '-', '+', '?', '$', '#', '!', '%'
};

const int DEFAULT_PULSE_DELAY = 500;

