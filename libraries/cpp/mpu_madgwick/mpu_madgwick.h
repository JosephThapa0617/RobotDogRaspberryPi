#include "MPU9250.h"
#include "MadgwickAHRS.h"



#define GRAVITY 9.802 // The gravity acceleration in New York City

// Calibration outcomes
#define GYRO_X_OFFSET 0.0003547
#define GYRO_Y_OFFSET 0.0002904
#define GYRO_Z_OFFSET 0.0000442

#define ACCEL_X_OFFSET -1.7587781
#define ACCEL_Y_OFFSET -8.6502850
#define ACCEL_Z_OFFSET -9.2377464

const float magn_ellipsoid_center[3] = {-16.444, 123.131, -60.0703};
const float magn_ellipsoid_transform[3][3] = {{0.887227, 0.0186386, 0.0175599}, {0.0186386, 0.878481, 0.0445495}, {0.0175599, 0.0445495, 0.978254}};

// Set structs for converting result from Quaternion to Euler angles
struct Quaternion {
    float w, x, y, z;
};

struct EulerAngles {
    float roll_e, pitch_e, yaw_e;
};

struct YPR {
  float yaw, pitch, roll;
};

struct Gravity {
  double x,y,z;
};

/*
extern float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).

extern float gyro[3];
*/

extern float magnetom[3];

extern Quaternion qua;
//extern EulerAngles eul;
extern YPR ypr;
//extern Gravity gravity;


void read_sensors();
void compensate_sensor_errors();
void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
void sendToPC(float* data1, float* data2, float* data3);
EulerAngles ToEulerAngles(Quaternion q);
void toYPR(Quaternion q);
void imu_setup();
void update_data();
