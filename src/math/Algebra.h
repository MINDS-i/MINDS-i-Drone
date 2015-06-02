#ifndef ALGEBRA_H
#define ALGEBRA_H
template <size_t rows, size_t cols>
static void rowReduce(float (&matrix)[rows][cols]){
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
float sqrtCurve(float input){
    float res = sqrt(fabs(input));
    return copysign(res, input);
}
float squareCurve(float input){
    float res = input*input;
    return copysign(res, input);
}
#endif
