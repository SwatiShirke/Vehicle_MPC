/* This file was automatically generated by CasADi 3.6.6.
 *  It consists of: 
 *   1) content generated by CasADi runtime: not copyrighted
 *   2) template code copied from CasADi source: permissively licensed (MIT-0)
 *   3) user code: owned by the user
 *
 */
#ifdef __cplusplus
extern "C" {
#endif

/* How to prefix internal symbols */
#ifdef CASADI_CODEGEN_PREFIX
  #define CASADI_NAMESPACE_CONCAT(NS, ID) _CASADI_NAMESPACE_CONCAT(NS, ID)
  #define _CASADI_NAMESPACE_CONCAT(NS, ID) NS ## ID
  #define CASADI_PREFIX(ID) CASADI_NAMESPACE_CONCAT(CODEGEN_PREFIX, ID)
#else
  #define CASADI_PREFIX(ID) Spatialbicycle_model_constr_h_fun_jac_uxt_zt_ ## ID
#endif

#include <math.h>

#ifndef casadi_real
#define casadi_real double
#endif

#ifndef casadi_int
#define casadi_int int
#endif

/* Add prefix to internal symbols */
#define casadi_clear CASADI_PREFIX(clear)
#define casadi_copy CASADI_PREFIX(copy)
#define casadi_f0 CASADI_PREFIX(f0)
#define casadi_fill CASADI_PREFIX(fill)
#define casadi_s0 CASADI_PREFIX(s0)
#define casadi_s1 CASADI_PREFIX(s1)
#define casadi_s2 CASADI_PREFIX(s2)
#define casadi_s3 CASADI_PREFIX(s3)
#define casadi_s4 CASADI_PREFIX(s4)
#define casadi_s5 CASADI_PREFIX(s5)
#define casadi_s6 CASADI_PREFIX(s6)
#define casadi_s7 CASADI_PREFIX(s7)
#define casadi_sq CASADI_PREFIX(sq)

/* Symbol visibility in DLLs */
#ifndef CASADI_SYMBOL_EXPORT
  #if defined(_WIN32) || defined(__WIN32__) || defined(__CYGWIN__)
    #if defined(STATIC_LINKED)
      #define CASADI_SYMBOL_EXPORT
    #else
      #define CASADI_SYMBOL_EXPORT __declspec(dllexport)
    #endif
  #elif defined(__GNUC__) && defined(GCC_HASCLASSVISIBILITY)
    #define CASADI_SYMBOL_EXPORT __attribute__ ((visibility ("default")))
  #else
    #define CASADI_SYMBOL_EXPORT
  #endif
#endif

void casadi_clear(casadi_real* x, casadi_int n) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = 0;
  }
}

void casadi_fill(casadi_real* x, casadi_int n, casadi_real alpha) {
  casadi_int i;
  if (x) {
    for (i=0; i<n; ++i) *x++ = alpha;
  }
}

casadi_real casadi_sq(casadi_real x) { return x*x;}

void casadi_copy(const casadi_real* x, casadi_int n, casadi_real* y) {
  casadi_int i;
  if (y) {
    if (x) {
      for (i=0; i<n; ++i) *y++ = *x++;
    } else {
      for (i=0; i<n; ++i) *y++ = 0.;
    }
  }
}

static const casadi_int casadi_s0[3] = {0, 2, 5};
static const casadi_int casadi_s1[3] = {1, 3, 6};
static const casadi_int casadi_s2[10] = {6, 1, 0, 6, 0, 1, 2, 3, 4, 5};
static const casadi_int casadi_s3[6] = {2, 1, 0, 2, 0, 1};
static const casadi_int casadi_s4[3] = {0, 0, 0};
static const casadi_int casadi_s5[9] = {5, 1, 0, 5, 0, 1, 2, 3, 4};
static const casadi_int casadi_s6[16] = {8, 5, 0, 2, 5, 6, 7, 8, 5, 6, 5, 6, 7, 3, 6, 7};
static const casadi_int casadi_s7[3] = {5, 0, 0};

/* Spatialbicycle_model_constr_h_fun_jac_uxt_zt:(i0[6],i1[2],i2[],i3[])->(o0[5],o1[8x5,8nz],o2[5x0]) */
static int casadi_f0(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem) {
  casadi_real *rr, *ss;
  const casadi_int *cii;
  casadi_real w0, w1, w2, w3, w4, w5, w6, w7, w8, w9, w10, w11, w12, w13, w14, w15, w16, w17, w18, w19, *w20=w+20, *w21=w+28, *w24=w+34, *w25=w+37, *w27=w+40, *w28=w+42;
  /* #0: @0 = 0.28 */
  w0 = 2.8000000000000003e-01;
  /* #1: @1 = 0.05 */
  w1 = 5.0000000000000003e-02;
  /* #2: @2 = input[0][3] */
  w2 = arg[0] ? arg[0][3] : 0;
  /* #3: @3 = (@1*@2) */
  w3  = (w1*w2);
  /* #4: @0 = (@0-@3) */
  w0 -= w3;
  /* #5: @3 = input[0][4] */
  w3 = arg[0] ? arg[0][4] : 0;
  /* #6: @4 = (@0*@3) */
  w4  = (w0*w3);
  /* #7: @5 = 0.006 */
  w5 = 6.0000000000000001e-03;
  /* #8: @6 = (@5*@2) */
  w6  = (w5*w2);
  /* #9: @7 = (@6*@2) */
  w7  = (w6*w2);
  /* #10: @4 = (@4-@7) */
  w4 -= w7;
  /* #11: @7 = 0.011 */
  w7 = 1.0999999999999999e-02;
  /* #12: @8 = 5 */
  w8 = 5.;
  /* #13: @9 = (@8*@2) */
  w9  = (w8*w2);
  /* #14: @9 = tanh(@9) */
  w9 = tanh( w9 );
  /* #15: @10 = (@7*@9) */
  w10  = (w7*w9);
  /* #16: @4 = (@4-@10) */
  w4 -= w10;
  /* #17: @10 = 0.043 */
  w10 = 4.2999999999999997e-02;
  /* #18: @10 = (@4/@10) */
  w10  = (w4/w10);
  /* #19: output[0][0] = @10 */
  if (res[0]) res[0][0] = w10;
  /* #20: @10 = 15.5 */
  w10 = 1.5500000000000000e+01;
  /* #21: @11 = (@10*@2) */
  w11  = (w10*w2);
  /* #22: @12 = (@11*@2) */
  w12  = (w11*w2);
  /* #23: @13 = input[0][5] */
  w13 = arg[0] ? arg[0][5] : 0;
  /* #24: @14 = (@12*@13) */
  w14  = (w12*w13);
  /* #25: @15 = 0.5 */
  w15 = 5.0000000000000000e-01;
  /* #26: @16 = (@15*@13) */
  w16  = (w15*w13);
  /* #27: @17 = sin(@16) */
  w17 = sin( w16 );
  /* #28: @18 = (@4*@17) */
  w18  = (w4*w17);
  /* #29: @19 = 0.043 */
  w19 = 4.2999999999999997e-02;
  /* #30: @18 = (@18/@19) */
  w18 /= w19;
  /* #31: @14 = (@14+@18) */
  w14 += w18;
  /* #32: output[0][1] = @14 */
  if (res[0]) res[0][1] = w14;
  /* #33: @14 = input[0][1] */
  w14 = arg[0] ? arg[0][1] : 0;
  /* #34: output[0][2] = @14 */
  if (res[0]) res[0][2] = w14;
  /* #35: output[0][3] = @3 */
  if (res[0]) res[0][3] = w3;
  /* #36: output[0][4] = @13 */
  if (res[0]) res[0][4] = w13;
  /* #37: @20 = zeros(8x5,8nz) */
  casadi_clear(w20, 8);
  /* #38: @14 = 23.2558 */
  w14 = 2.3255813953488374e+01;
  /* #39: @21 = ones(8x1,6nz) */
  casadi_fill(w21, 6, 1.);
  /* #40: {NULL, NULL, NULL, @18, NULL, @19, NULL, NULL} = vertsplit(@21) */
  w18 = w21[3];
  w19 = w21[5];
  /* #41: @5 = (@5*@19) */
  w5 *= w19;
  /* #42: @5 = (@2*@5) */
  w5  = (w2*w5);
  /* #43: @6 = (@6*@19) */
  w6 *= w19;
  /* #44: @5 = (@5+@6) */
  w5 += w6;
  /* #45: @5 = (-@5) */
  w5 = (- w5 );
  /* #46: @1 = (@1*@19) */
  w1 *= w19;
  /* #47: @3 = (@3*@1) */
  w3 *= w1;
  /* #48: @5 = (@5-@3) */
  w5 -= w3;
  /* #49: @3 = 1 */
  w3 = 1.;
  /* #50: @9 = sq(@9) */
  w9 = casadi_sq( w9 );
  /* #51: @3 = (@3-@9) */
  w3 -= w9;
  /* #52: @8 = (@8*@19) */
  w8 *= w19;
  /* #53: @3 = (@3*@8) */
  w3 *= w8;
  /* #54: @7 = (@7*@3) */
  w7 *= w3;
  /* #55: @5 = (@5-@7) */
  w5 -= w7;
  /* #56: @7 = (@14*@5) */
  w7  = (w14*w5);
  /* #57: @10 = (@10*@19) */
  w10 *= w19;
  /* #58: @2 = (@2*@10) */
  w2 *= w10;
  /* #59: @11 = (@11*@19) */
  w11 *= w19;
  /* #60: @2 = (@2+@11) */
  w2 += w11;
  /* #61: @13 = (@13*@2) */
  w13 *= w2;
  /* #62: @2 = 23.2558 */
  w2 = 2.3255813953488374e+01;
  /* #63: @5 = (@17*@5) */
  w5  = (w17*w5);
  /* #64: @5 = (@2*@5) */
  w5  = (w2*w5);
  /* #65: @13 = (@13+@5) */
  w13 += w5;
  /* #66: @22 = 00 */
  /* #67: @23 = 00 */
  /* #68: @24 = vertcat(@7, @13, @18, @22, @23) */
  rr=w24;
  *rr++ = w7;
  *rr++ = w13;
  *rr++ = w18;
  /* #69: @25 = @24[:3] */
  for (rr=w25, ss=w24+0; ss!=w24+3; ss+=1) *rr++ = *ss;
  /* #70: (@20[0, 2, 5] = @25) */
  for (cii=casadi_s0, rr=w20, ss=w25; cii!=casadi_s0+3; ++cii, ++ss) rr[*cii] = *ss;
  /* #71: @7 = ones(8x1,1nz) */
  w7 = 1.;
  /* #72: {NULL, NULL, NULL, NULL, NULL, NULL, @13, NULL} = vertsplit(@7) */
  w13 = w7;
  /* #73: @0 = (@0*@13) */
  w0 *= w13;
  /* #74: @14 = (@14*@0) */
  w14 *= w0;
  /* #75: @17 = (@17*@0) */
  w17 *= w0;
  /* #76: @17 = (@2*@17) */
  w17  = (w2*w17);
  /* #77: @22 = 00 */
  /* #78: @23 = 00 */
  /* #79: @25 = vertcat(@14, @17, @22, @13, @23) */
  rr=w25;
  *rr++ = w14;
  *rr++ = w17;
  *rr++ = w13;
  /* #80: @24 = @25[:3] */
  for (rr=w24, ss=w25+0; ss!=w25+3; ss+=1) *rr++ = *ss;
  /* #81: (@20[1, 3, 6] = @24) */
  for (cii=casadi_s1, rr=w20, ss=w24; cii!=casadi_s1+3; ++cii, ++ss) rr[*cii] = *ss;
  /* #82: @22 = 00 */
  /* #83: @14 = ones(8x1,1nz) */
  w14 = 1.;
  /* #84: {NULL, NULL, NULL, NULL, NULL, NULL, NULL, @17} = vertsplit(@14) */
  w17 = w14;
  /* #85: @12 = (@12*@17) */
  w12 *= w17;
  /* #86: @16 = cos(@16) */
  w16 = cos( w16 );
  /* #87: @15 = (@15*@17) */
  w15 *= w17;
  /* #88: @16 = (@16*@15) */
  w16 *= w15;
  /* #89: @4 = (@4*@16) */
  w4 *= w16;
  /* #90: @2 = (@2*@4) */
  w2 *= w4;
  /* #91: @12 = (@12+@2) */
  w12 += w2;
  /* #92: @23 = 00 */
  /* #93: @26 = 00 */
  /* #94: @27 = vertcat(@22, @12, @23, @26, @17) */
  rr=w27;
  *rr++ = w12;
  *rr++ = w17;
  /* #95: @28 = @27[:2] */
  for (rr=w28, ss=w27+0; ss!=w27+2; ss+=1) *rr++ = *ss;
  /* #96: (@20[4:10:3] = @28) */
  for (rr=w20+4, ss=w28; rr!=w20+10; rr+=3) *rr = *ss++;
  /* #97: output[1][0] = @20 */
  casadi_copy(w20, 8, res[1]);
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt(const casadi_real** arg, casadi_real** res, casadi_int* iw, casadi_real* w, int mem){
  return casadi_f0(arg, res, iw, w, mem);
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_alloc_mem(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_init_mem(int mem) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Spatialbicycle_model_constr_h_fun_jac_uxt_zt_free_mem(int mem) {
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_checkout(void) {
  return 0;
}

CASADI_SYMBOL_EXPORT void Spatialbicycle_model_constr_h_fun_jac_uxt_zt_release(int mem) {
}

CASADI_SYMBOL_EXPORT void Spatialbicycle_model_constr_h_fun_jac_uxt_zt_incref(void) {
}

CASADI_SYMBOL_EXPORT void Spatialbicycle_model_constr_h_fun_jac_uxt_zt_decref(void) {
}

CASADI_SYMBOL_EXPORT casadi_int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_n_in(void) { return 4;}

CASADI_SYMBOL_EXPORT casadi_int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_n_out(void) { return 3;}

CASADI_SYMBOL_EXPORT casadi_real Spatialbicycle_model_constr_h_fun_jac_uxt_zt_default_in(casadi_int i) {
  switch (i) {
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Spatialbicycle_model_constr_h_fun_jac_uxt_zt_name_in(casadi_int i) {
  switch (i) {
    case 0: return "i0";
    case 1: return "i1";
    case 2: return "i2";
    case 3: return "i3";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const char* Spatialbicycle_model_constr_h_fun_jac_uxt_zt_name_out(casadi_int i) {
  switch (i) {
    case 0: return "o0";
    case 1: return "o1";
    case 2: return "o2";
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Spatialbicycle_model_constr_h_fun_jac_uxt_zt_sparsity_in(casadi_int i) {
  switch (i) {
    case 0: return casadi_s2;
    case 1: return casadi_s3;
    case 2: return casadi_s4;
    case 3: return casadi_s4;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT const casadi_int* Spatialbicycle_model_constr_h_fun_jac_uxt_zt_sparsity_out(casadi_int i) {
  switch (i) {
    case 0: return casadi_s5;
    case 1: return casadi_s6;
    case 2: return casadi_s7;
    default: return 0;
  }
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_work(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 9;
  if (sz_res) *sz_res = 11;
  if (sz_iw) *sz_iw = 0;
  if (sz_w) *sz_w = 44;
  return 0;
}

CASADI_SYMBOL_EXPORT int Spatialbicycle_model_constr_h_fun_jac_uxt_zt_work_bytes(casadi_int *sz_arg, casadi_int* sz_res, casadi_int *sz_iw, casadi_int *sz_w) {
  if (sz_arg) *sz_arg = 9*sizeof(const casadi_real*);
  if (sz_res) *sz_res = 11*sizeof(casadi_real*);
  if (sz_iw) *sz_iw = 0*sizeof(casadi_int);
  if (sz_w) *sz_w = 44*sizeof(casadi_real);
  return 0;
}


#ifdef __cplusplus
} /* extern "C" */
#endif