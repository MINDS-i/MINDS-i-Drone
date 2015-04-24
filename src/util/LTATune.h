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
    //takes an array of 6 triplets of measured values taken
    //uniformly from a 3d ellipse
    static LTATune FitEllipsoid(float (&values)[6][3]){
        /**
         * Equations taken from
         * "Performance comparison of accelerometer calibration algorithms
         * based on 3D-ellipsoid fitting methods"
         */
        LTATune tune;

        //get centers
        float E=0,Vx=0,Vy=0,Vz=0;
        float Vxx=0, Vyy=0, Vzz=0;
        float Vxy=0, Vxz=0, Vyz=0;
        float Ex=0,  Ey=0,  Ez=0;
        for(int i=0; i<6; i++){
            float Ei = -1*(values[i][0]*values[i][0]+
                           values[i][1]*values[i][1]+
                           values[i][2]*values[i][2] );
            E   += Ei;
            Ex  += Ei*values[i][0];
            Ey  += Ei*values[i][1];
            Ez  += Ei*values[i][2];

            Vx  += values[i][0];
            Vy  += values[i][1];
            Vz  += values[i][2];

            Vxx += (values[i][0])*(values[i][0]);
            Vyy += (values[i][1])*(values[i][1]);
            Vzz += (values[i][2])*(values[i][2]);

            Vxy += (values[i][0])*(values[i][1]);
            Vxz += (values[i][0])*(values[i][2]);
            Vyz += (values[i][1])*(values[i][2]);
        }
        float matrix[4][5] = {
            {6 ,  Vx,  Vy,  Vz, E }, //hint: there are 6 measurements
            {Vx, Vxx, Vxy, Vxz, Ex},
            {Vy, Vxy, Vyy, Vyz, Ey},
            {Vz, Vxz, Vyz, Vzz, Ez}
        };
        rowReduce<4,5>(matrix);

        if (matrix[1][1] != 0) tune.shift[0] = (matrix[1][4]/matrix[1][1])/-2.0;
        else                   tune.shift[0] = 0;
        if (matrix[2][2] != 0) tune.shift[1] = (matrix[2][4]/matrix[2][2])/-2.0;
        else                   tune.shift[1] = 0;
        if (matrix[3][3] != 0) tune.shift[2] = (matrix[3][4]/matrix[3][3])/-2.0;
        else                   tune.shift[2] = 0;

        //find scale factors
        float  X=0,  Y=0,  Z=0;
        float XX=0, YY=0, ZZ=0;
        float XY=0, XZ=0, YZ=0;
        for(int i=0; i<6; i++){
            float x = values[i][0]-tune.shift[0]; x = x*x;
            float y = values[i][1]-tune.shift[1]; y = y*y;
            float z = values[i][2]-tune.shift[2]; z = z*z;
            X  += x;
            Y  += y;
            Z  += z;
            XX += x*x;
            YY += y*y;
            ZZ += z*z;
            XY += x*y;
            XZ += x*z;
            YZ += y*z;
        }
        float sMatrix[3][4] = {
            { XX, XY, XZ,  X},
            { XY, YY, YZ,  Y},
            { XZ, YZ, ZZ,  Z}
        };
        rowReduce<3,4>(sMatrix);

        float tx = sMatrix[0][3]/sMatrix[0][0];
        float ty = sMatrix[1][3]/sMatrix[1][1];
        float tz = sMatrix[2][3]/sMatrix[2][2];
        tune.scalar[0] = sqrt(1.0/tx);
        tune.scalar[1] = sqrt(1.0/ty);
        tune.scalar[2] = sqrt(1.0/tz);

        return tune;
    }
    //takes an array of 3 pairs of values, being the extremes
    //seen when measuring a vector with and against each axis
    static LTATune SimpleExtremes(float (&values)[3][2]){
        LTATune tune;
        for(int i=0; i<3; i++){
            float sum        = (values[i][0]+values[i][1]);
            float difference = (values[i][0]-values[i][1]);
            tune.shift[i]    = -sum/2.f;
            tune.scalar[i]   = (2.f)/difference;
        }
        return tune;
    }

    template <size_t rows, size_t cols>
    static void rowReduce(float (&matrix)[rows][cols] ){
        //iterate down diagonol, zeroing up and down
        for(int d=0; d<min(rows,cols); d++){
            float m = matrix[d][d];
            if(m==0) continue;
            //iterate along rows that are not the row of interest
            for(int r=0; r<rows; r++){
                if(r==d) continue;
                float factor = -matrix[r][d]/m;
                //iterate along row doing calculations
                for(int i=d; i<cols; i++){
                    matrix[r][i] += factor*matrix[d][i];
                }
            }
        }
    }
};
#endif
