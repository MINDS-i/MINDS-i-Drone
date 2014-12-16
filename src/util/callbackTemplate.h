#ifndef CALLBACKGEN_H
#define CALLBACKGEN_H
//used to generate callbacks to member function of global class instances
//usage: callback<foo, &fooInst, &foo::func>
template<typename C, C *inst, void (C::*func)(float)>
void callback(float a){
	(inst->*func)(a);
}
#endif
