/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

// if 0 - 0 obstacles version, 1 - 1 obstactle version
#define NN_OBS 0


// marwz
#define NN_SIZE 50

#if NN_OBS == 0
#define NN_INPUT_SIZE 6
#else
#define NN_INPUT_SIZE 10
#endif

#define NN_OUTPUT_SIZE 2
// rows x col

//#define REL_U(in) (if (in>0) return in else return 0)
#define REL_U(in) ((in>0)?in:0)

typedef struct nn_inputs_s {
  float x1[NN_INPUT_SIZE];
  float z;
  float vz;
  uint32_t updated;
  uint32_t thrust;
} nn_inputs_t;

typedef struct nn_s {
  uint32_t timestamp;  // Timestamp when the data was computed
  uint32_t dt;
  uint32_t thrust;
  float weights1[NN_SIZE][NN_INPUT_SIZE]; //50x6
  float weights2[NN_SIZE][NN_SIZE]; // 50x50
  float weights3[NN_OUTPUT_SIZE][NN_SIZE]; //2x50
  float b1 [NN_SIZE]; // 6x1
  float b2 [NN_SIZE]; //50x1
  float b3 [NN_OUTPUT_SIZE]; //2x1
  float x1 [NN_INPUT_SIZE]; // 6x1
  float x1_raw [NN_INPUT_SIZE]; // 6x1
  float x2 [NN_SIZE]; // 50 first layer output
  float x3 [NN_SIZE]; // 50 second layer output
  float y [NN_OUTPUT_SIZE]; // 2
  float std_x [NN_INPUT_SIZE]; // 6
  float mean_x [NN_INPUT_SIZE]; // 6 
  float std_y [NN_OUTPUT_SIZE]; // 2
  float mean_y [NN_OUTPUT_SIZE]; // 2
} nn_t;

// for python nn formating static float R[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
// f = REL_U
// normalize x1: x1 = (x1 - mean_x) / std_x  ; std_x and mean_x precalculated
// x2 = f(weights1*x1 + b1)
// x3 = f(weights2*x2 + b2)
// y = weigths2 * x3 + b3
// normalize y: y = y * std_y + mean_y; std_y and mean_y precalculated


/** Attitude in euler angle form */
typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* x,y,z vector */
struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;

/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef struct toaMeasurement_s {
  int8_t senderId;
  float x, y, z;
  int64_t rx, tx;
} toaMeasurement_t;

typedef struct tdoaMeasurement_s {
  toaMeasurement_t measurement[2];
  float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s {
  float pressure;
  float temperature;
  float asl;
} baro_t;

typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
} positionMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct sensorData_s {
  Axis3f acc;
  Axis3f gyro;
  Axis3f mag;
  baro_t baro;
  point_t position;
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;
  quaternion_t attitudeQuaternion;
  point_t position;
  velocity_t velocity;
  acc_t acc;
} state_t;

typedef struct control_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
} control_t;

typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} mode_t;

typedef struct setpoint_s {
  attitude_t attitude;
  attitude_t attitudeRate;
  float thrust;
  point_t position;
  velocity_t velocity;

  struct {
    mode_t x;
    mode_t y;
    mode_t z;
    mode_t roll;
    mode_t pitch;
    mode_t yaw;
  } mode;
} setpoint_t;

/** Estimate of position */
typedef struct estimate_s {
  uint32_t timestamp; // Timestamp when the data was computed

  point_t position;
} estimate_t;

/** Setpoint for althold */
typedef struct setpointZ_s {
  float z;
  bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#ifdef ESTIMATOR_TYPE_kalman
#define RATE_MAIN_LOOP RATE_1000_HZ
#else
#define RATE_MAIN_LOOP RATE_1000_HZ
#endif

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#endif
