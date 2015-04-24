#ifndef LTATune_H
#define LTATune_H
//Linear Three Axis Tune
// a = (a+shift)*scalar
//shift should be applied before scalar

struct LTATune{
	union{
		float params[2][3];
		float raw[6];
		struct {
			float shift[3];
			float scalar[3];
		};
	};
	LTATune(): raw{0,0,0,1,1,1} {}
    inline void calibrate(float& value, uint8_t axis){
        value = (value+shift[axis])*scalar[axis];
    }
    inline void calibrate(float (&values)[3]){
        calibrate(values[0], 0);
        calibrate(values[1], 1);
        calibrate(values[2], 2);
    }
    template <typename T>
    void inline calibrate(T (&data)[3], float(&calibrated)[3]){
        calibrated[0] = (float) data[0];
        calibrated[1] = (float) data[1];
        calibrated[2] = (float) data[2];
        calibrate(calibrated);
    }
    inline float apply(float value, uint8_t axis){//axis X,Y,Z
        calibrate(value, axis);
        return value;
    }
};
#endif
