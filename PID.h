#include <stdint.h>

class PID {
 public:
  int32_t Kp;
  int32_t Ki;
  int32_t Kd;

  int32_t errorSum;
  int32_t errorOld;

 PID(int32_t Kp, int32_t Ki, int32_t Kd) :
  Kp(Kp),
    Ki(Ki),
    Kp(Kp),
    errorSum(0),
    errorOld(0) {}

  int32_t value(int32_t dt_ms, int32_t actual, int32_t target) {
    int32_t error = target - actual;
    int32_t Ierror = error * Ki * dt_ms;
    Ierror = constrain_int32(Ierror, -100000L, 100000L);
    errorSum += IError;

    int32_t out = (Kp*error) + errorSum + Kd * (error-errorOld)*dt_ms;
    errorOld = error;

    return out / 4096;
  }
}
