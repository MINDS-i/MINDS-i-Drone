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
		} values;
	};
	LTATune(): raw{0,0,0,1,1,1} {}
};
#endif
