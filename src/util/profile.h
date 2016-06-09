#ifndef PROFILE_H
#define PROFILE_H
#if DEBUG
const int NUM_PROFILES = 16;
volatile uint32_t profile[NUM_PROFILES];
void __attribute__((always_inline)) inline tic(int i){
	profile[i] = -micros();
}
void __attribute__((always_inline)) inline toc(int i){
	profile[i] += micros();
}
#endif
#endif
