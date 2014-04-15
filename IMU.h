
/*
 * IMU is always used like:
 *
    // update IMU data
    readGyros();   // t=386us

    updateGyroAttitude();  // t=260us
    updateACCAttitude();   // t=146us

    getAttitudeAngles(); // t=468us
 *
 */

class IMU {
private:
	void readGyros();
	void updateGyroAttitude();
	void updateACCAttitude();
	void updateAttitudeAngles();

	// swap two char items
	inline void swap_char(char * a, char * b) {
	  char tmp = *a;
	  *a = *b;
	  *b = tmp;
	}

	// swap two int items
	inline void swap_int(int * a, int * b) {
	  int tmp = *a;
	  *a = *b;
	  *b = tmp;
	}
};
