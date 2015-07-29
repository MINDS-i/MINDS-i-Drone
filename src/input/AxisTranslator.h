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
    static const Translator APM = &write<false, 1,
                                         false, 0,
                                          true, 2 >;
}
#endif
