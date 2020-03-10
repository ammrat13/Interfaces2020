/********************************************************
 * DiffDrive Basic Example
 * See defines for pin wirings.
 ********************************************************/

#include "DiffDrive.h"

// Motor Interfaces
#define PWM_LEFT     (3)
#define DIR1_LEFT    (6) //IN3
#define DIR2_LEFT    (7) //IN4
#define ENCA_LEFT    (16)
#define ENCB_LEFT    (17)

#define PWM_RIGHT    (2)
#define DIR1_RIGHT   (4) //IN1
#define DIR2_RIGHT   (5) //IN2
#define ENCA_RIGHT   (8)
#define ENCB_RIGHT   (9)

// Robot Params
#define CNTS_PER_REV (3591.84) // https://www.pololu.com/product/4866
#define R            (0.045)
#define D            (0.2427)
#define L            (0.1)

// PID Constants
#define KP_CONS      (6.0f)
#define KI_CONS      (0.00001f)
#define KD_CONS      (0.0f)
#define KP_AGG       (12.0f)
#define KI_AGG       (0.00001f)
#define KD_AGG       (0.1f)

DiffDrive<PWM_LEFT, DIR1_LEFT, DIR2_LEFT, ENCA_LEFT, ENCB_LEFT,
          PWM_RIGHT, DIR1_RIGHT, DIR2_RIGHT, ENCA_RIGHT, ENCB_RIGHT> robot(CNTS_PER_REV, R, D, L, KP_CONS, KI_CONS, KD_CONS,
                                                                           KP_AGG, KI_AGG, KD_AGG);

void setup() {
}

void loop() {
    robot.loop();
}
