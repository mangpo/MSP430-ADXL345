/*
 * fixed-point.h
 *
 *  Created on: Apr 12, 2016
 *      Author: mangpo
 */

#ifndef FIXED_POINT_H_
#define FIXED_POINT_H_


#define FP_BITS 16ull       // Total bits per fixed-point number.
#define FP_P 1ull           // integer bits.
#define FP_Q 15ull          // fractional bits.

#define FP_F (1ull << FP_Q)  // pow(2, FP_Q).

typedef int fp_t;
typedef long fp_t2;

inline fp_t
d2fp(double n){ return n*FP_F; }

inline double
fp2d(fp_t n){ return (double)n/FP_F; }

inline fp_t
fp_add(fp_t x, fp_t y) { return x + y; }

inline fp_t
fp_sub(fp_t x, fp_t y) { return x - y; }

//inline
fp_t fp_mul(fp_t x, fp_t y){
  fp_t2 _x = (x >= 0 ? x : -x);
  fp_t2 _y = (y >= 0 ? y : -y);
  fp_t xy = (_x * _y)/ FP_F;
  if ((x >= 0) != (y >= 0)){
    xy = -xy;
  }
  return xy;
}

/* inline fp_t */
/* __fp_div(fp_t x, fp_t y){ */
/*   fp_t _x = (x >= 0 ? x : -x); */
/*   fp_t _y = (y >= 0 ? y : -y); */
/*   fp_t xy = (fp_t) _x * FP_F / _y; */
/*   if ((x >= 0) != (y >= 0)){ */
/*     xy = -xy; */
/*   } */
/*   return xy; */
/* } */

/* inline fp_t */
/* fp_inv(fp_t n){ return __fp_div(d2fp(1.0), n);} */

inline char // Return -1,0,1 if <,==,>
fp_cmp(fp_t x, fp_t y){
  return x < y ? 2 : (x > y ? 1 : 0);
}



#endif /* FIXED_POINT_H_ */
