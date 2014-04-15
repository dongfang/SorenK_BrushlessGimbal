// **************************************************
// Simplified IMU based on "Complementary Filter"
// Inspired by http://starlino.com/imu_guide.html
//
// adapted by ziss_dm : http://www.multiwii.com/forum/viewtopic.php?f=8&t=198
//
// The following ideas was used in this project:
// 1) Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
// 2) Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// 3) C. Hastings approximation for atan2()
// 4) Optimization tricks: http://www.hackersdelight.org/
//
// Currently Magnetometer uses separate CF which is used only
// for heading approximation.
//
// **************************************************

//******  advanced users settings *******************
/* Set the Low Pass Filter factor for ACC */
/* Increasing this value would reduce ACC noise (visible in GUI), but would increase ACC lag time*/
/* Comment this if  you do not want filter at all.*/
#ifndef ACC_LPF_FACTOR
  #define ACC_LPF_FACTOR 40
#endif

#define ACC_1G 16384.0f

// Small angle approximation
#define ssin(val) (val)
#define scos(val) 1.0f

// set default sensor orientation (sensor upside)
void initSensorOrientationFaceUp() {
  // This one may not work. not tested in this version!
  // channel assignment
  sensorDef.Gyro[ROLL].idx = 0;
  sensorDef.Gyro[PITCH].idx = 1;
  sensorDef.Gyro[YAW].idx = 2;

  sensorDef.Acc[ROLL].idx = 1;     // y
  sensorDef.Acc[PITCH].idx = 0;    // x
  sensorDef.Acc[YAW].idx = 2;      // z

  // direction
  sensorDef.Gyro[ROLL].dir = 1;
  sensorDef.Gyro[PITCH].dir = -1;
  sensorDef.Gyro[YAW].dir = 1;

  sensorDef.Acc[ROLL].dir = 1;
  sensorDef.Acc[PITCH].dir = 1;
  sensorDef.Acc[YAW].dir = 1;
}

// set sensor orientation (chip end up)
void initSensorOrientationChipTextRightSideUp() {
  // channel assignment
  sensorDef.Gyro[ROLL].idx = 0;
  sensorDef.Gyro[PITCH].idx = 2;
  sensorDef.Gyro[YAW].idx = 1;
   
  sensorDef.Acc[ROLL].idx = 1;
  sensorDef.Acc[PITCH].idx = 2;
  sensorDef.Acc[YAW].idx = 0;

  // direction
  sensorDef.Gyro[ROLL].dir = 1;
  sensorDef.Gyro[PITCH].dir = -1;
  sensorDef.Gyro[YAW].dir = -1;

  sensorDef.Acc[ROLL].dir = 1;
  sensorDef.Acc[PITCH].dir = 1;
  sensorDef.Acc[YAW].dir = -1;
}

// set sensor orientation (chip side up)
void initSensorOrientationChipTextStandingOnEnd() {
  // channel assignment
  sensorDef.Gyro[ROLL].idx = 1;
  sensorDef.Gyro[PITCH].idx = 2;
  sensorDef.Gyro[YAW].idx = 0;
   
  sensorDef.Acc[ROLL].idx = 2;
  sensorDef.Acc[PITCH].idx = 1;
  sensorDef.Acc[YAW].idx = 0;

  // direction
  sensorDef.Gyro[ROLL].dir = 1;
  sensorDef.Gyro[PITCH].dir = -1;
  sensorDef.Gyro[YAW].dir = 1;

  sensorDef.Acc[ROLL].dir = 1;
  sensorDef.Acc[PITCH].dir = 1;
  sensorDef.Acc[YAW].dir = 1;
}

void initSensorOrientationDefault(uint8_t majorAxis) {
  switch(majorAxis) {
    case 1:  initSensorOrientationChipTextRightSideUp(); break;
    case 2:  initSensorOrientationChipTextStandingOnEnd(); break;
    default: initSensorOrientationFaceUp(); break;
  }
}

// set sensor orientation according config
//
//   config.axisReverseZ
//        false ... sensor mounted on top
//        true  ... sensor mounted upside down
//   config.axisSwapXY
//        false ... default XY axes
//        true  ... swap XY (means exchange Roll/Pitch)

void initSensorOrientation() {
  
  initSensorOrientationDefault(config.majorAxis);
  
  if (config.axisReverseZ) {
    // flip over roll
    sensorDef.Acc[YAW].dir *= -1;
    sensorDef.Acc[ROLL].dir *= -1;
    sensorDef.Gyro[PITCH].dir *= -1;
    sensorDef.Gyro[YAW].dir *= -1;
  }

  if (config.axisSwapXY) {
    // swap gyro axis
    swap_char(&sensorDef.Gyro[ROLL].idx, &sensorDef.Gyro[PITCH].idx); 
    swap_int(&sensorDef.Gyro[ROLL].dir, &sensorDef.Gyro[PITCH].dir); sensorDef.Gyro[PITCH].dir *= -1;   // try and error ;-)
    // swap acc axis
    swap_char(&sensorDef.Acc[ROLL].idx, &sensorDef.Acc[PITCH].idx);
    swap_int(&sensorDef.Acc[ROLL].dir, &sensorDef.Acc[PITCH].dir); sensorDef.Acc[ROLL].dir *= -1;
  }
}

void setACCFastMode (bool fastMode) {
  if (fastMode) {
    AccComplFilterConst = (float)DT_FLOAT/(2.0 + DT_FLOAT); // 2 sec
  } else {
    AccComplFilterConst = (float)DT_FLOAT/(config.accTimeConstant + DT_FLOAT);
  }
}

void initIMU() {
  // resolutionDivider=131, scale = 0.000133
  // 102us
  gyroScale =  1.0 / resolutionDivider / 180.0 * 3.14159265359;  // convert to radians
  setACCFastMode(false);
  
  uint8_t axis;

  for (axis = 0; axis < 3; axis++) {
    readACC((axisDef)axis);
    accLPF[axis] = accADC[axis];
    EstG[axis] = accADC[0];
  }

  accMag = 100;
}

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
// needs angle in radian units !
void rotateV(float* v, float* delta) {
  float v_tmp[3] = {v[X],v[Y],v[Z]};
  v[Z] -= delta[ROLL]  * v_tmp[X] + delta[PITCH] * v_tmp[Y];
  v[X] += delta[ROLL]  * v_tmp[Z]-  delta[YAW]   * v_tmp[Y];
  v[Y] += delta[PITCH] * v_tmp[Z] + delta[YAW]   * v_tmp[X];
}

void readGyros() {
  int16_t axisRot[3];
  uint8_t idx;
  // 414 us

  // read gyros
  mpu.getRotation(&axisRot[0], &axisRot[1], &axisRot[2]);
  idx = sensorDef.Gyro[0].idx;
  gyroADC[ROLL] = axisRot[idx]-gyroOffset[idx];
  gyroADC[ROLL] *= sensorDef.Gyro[0].dir;

  idx = sensorDef.Gyro[1].idx;
  gyroADC[PITCH] = axisRot[idx]-gyroOffset[idx];
  gyroADC[PITCH] *= sensorDef.Gyro[1].dir;

  idx = sensorDef.Gyro[2].idx;
  gyroADC[YAW] = axisRot[idx]-gyroOffset[idx];  
  gyroADC[YAW] *= sensorDef.Gyro[2].dir;
}

// Called from slow-loop in BruGi.
void readACC(axisDef axis) {
  // get acceleration
  // 382 us
  uint8_t idx;
  int16_t val;
  idx = sensorDef.Acc[axis].idx;
  val = mpu.getAccelerationN(idx);  // TODO: 370us 
  val *= sensorDef.Acc[axis].dir;
  accADC[axis] = val;
}

void updateGyroAttitude(){
  uint8_t axis;
  
  float deltaGyroAngle[3];

  for (axis = 0; axis < 3; axis++) {
    deltaGyroAngle[axis] = gyroADC[axis]  * gyroScale * DT_FLOAT;
  }
  // 168 us
  rotateV(EstG, deltaGyroAngle);
}

void updateACC(){
  uint8_t axis;

  // 179 us
  accMag = 0;
  for (axis = 0; axis < 3; axis++) {
    accLPF[axis] = accLPF[axis] * (1.0f - (1.0f/ACC_LPF_FACTOR)) + accADC[axis] * (1.0f/ACC_LPF_FACTOR);
    accSmooth[axis] = accLPF[axis];
    accMag += (int32_t)accSmooth[axis]*accSmooth[axis] ;
  }

  // accMag = accMag*100/((int32_t)ACC_1G*ACC_1G); 
  // 130 us
  // split operation to avoid 32-bit overflow, TODO: no division may happen !!!
  accMag = accMag/(int32_t)ACC_1G;
  accMag = accMag*100;
  accMag = accMag/(int32_t)ACC_1G;

  // 11 us
  if ( abs(accSmooth[ROLL])<acc_25deg && abs(accSmooth[PITCH])<acc_25deg && accSmooth[YAW]>0) {
    flags.SMALL_ANGLES_25 = 1;
  } else {
    flags.SMALL_ANGLES_25 = 0;
  }
}

void updateACCAttitude(){
  uint8_t axis;
  
  // 255 us
  // Apply complimentary filter (Gyro drift correction)
  // If accel magnitude >1.4G or <0.6G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
  // To do that, we just skip filter, as EstV already rotated by Gyro
  if ((36 < accMag && accMag < 196) || flags.SMALL_ANGLES_25) {
    for (axis = 0; axis < 3; axis++) {
      int32_t acc = accSmooth[axis];
      EstG[axis] = EstG[axis] * (1.0 - AccComplFilterConst) + acc * AccComplFilterConst; // note: this is different from MultiWii (wrong brackets postion in MultiWii ??.
    } 
  }
}

void updateAttitudeAngles() {
  // attitude of the estimated vector  
  // 272 us
  angle[ROLL]  = (config.angleOffsetRoll * 10) +  Rajan_FastArcTan2_deg1000(EstG[X] , sqrt(EstG[Z]*EstG[Z]+EstG[Y]*EstG[Y]));
  // 192 us
  angle[PITCH] = (config.angleOffsetPitch * 10) + Rajan_FastArcTan2_deg1000(EstG[Y] , EstG[Z]);  
}
