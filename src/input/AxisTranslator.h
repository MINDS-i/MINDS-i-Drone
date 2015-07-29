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
    static const Translator ident = &write<false,false,false,0,1,2>;
    static const Translator APM = &write<false,false,true,1,0,2>;
}
#endif
