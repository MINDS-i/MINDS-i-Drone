#ifndef AxisTranslator_H
#define AxisTranslator_H

typedef Vec3 (*Translator)(float (&in)[3]);
namespace Translators{
    template<bool xs, int x,
             bool ys, int y,
             bool zs, int z>
    Vec3 write(float (&in)[3]){
        return Vec3( (xs)? -in[x] : in[x],
                     (ys)? -in[y] : in[y],
                     (zs)? -in[z] : in[z] );
    }
    static const Translator identity = &write<false, 0,
                                              false, 1,
                                              false, 2 >;
    //APM onboard compass orientaiton - note that guided calibration
    //will use the calibration to make it match the APM_MPU frame
    static const Translator APM_HMC = &write<true, 1,
                                             true, 0,
                                             true, 2 >;
    //frame translation from onboard MPU to NED on APM 2.5+ boards
    static const Translator APM_MPU = &write<false, 1,
                                             false, 0,
                                              true, 2 >;
}
#endif
