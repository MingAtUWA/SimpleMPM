#ifndef _TEST_SIM_CORE_H_
#define _TEST_SIM_CORE_H_

void test_hdf5_resultfile(void);
void test_text_resultfile(void);

void display_model(void);
void test_mesh(void);
void test_cal_shape_func(void);

void test_me_mpm1(void);
void test_me_mpm2(void);
void test_me_mpm3(void);
void test_me_mpm4(void);
void test_me_mpm5(void);

void test_me_kindamp_mpm1(void);
void test_me_kindamp_mpm2(void);

void test_me_bspline_mpm1(void);

void test_chm_mpm1(void);
void test_chm_mpm2(void);

void test_chm_visdamp_mpm1(void);
void test_chm_visdamp_mpm2(void);
void test_chm_visdamp_mpm3(void);
void test_chm_visdamp_mpm4(void);

//void test_chm_kindamp_mpm1(void);
void test_chm_kindamp_mpm2(void);

// multiple objects
void test_multi_object_me1(void);

#endif