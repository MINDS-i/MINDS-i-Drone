#ifndef OUTPUT_DEVICE_H
#define OUTPUT_DEVICE_H
class OutputDevice{
public:
	virtual void    startArming()    = 0;
	virtual boolean continueArming(uint32_t dt) = 0; //rtn true when done arming
	virtual void    startCalibrate() = 0;
	virtual boolean continueCalibrate(uint32_t dt) = 0; //rtn true when done
	virtual void    set(float in)    = 0;
	virtual void    stop() = 0;
	virtual float   get() = 0;
};
#endif
