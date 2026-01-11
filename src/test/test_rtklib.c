#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "minunit.h"
#include "../rtklib.h"

int tests_run = 0;

/* Helper function for double comparison */
int is_close(double a, double b, double epsilon) {
    return fabs(a - b) < epsilon;
}

/* Test: Time conversion */
static char * test_time_conversion() {
    gtime_t t0, t1;
    double ep[6] = {2023, 10, 25, 12, 0, 0};
    double ep_out[6];
    
    t0 = epoch2time(ep);
    time2epoch(t0, ep_out);
    
    mu_assert("error, year mismatch", is_close(ep[0], ep_out[0], 1e-6));
    mu_assert("error, month mismatch", is_close(ep[1], ep_out[1], 1e-6));
    mu_assert("error, day mismatch", is_close(ep[2], ep_out[2], 1e-6));
    mu_assert("error, hour mismatch", is_close(ep[3], ep_out[3], 1e-6));
    mu_assert("error, min mismatch", is_close(ep[4], ep_out[4], 1e-6));
    mu_assert("error, sec mismatch", is_close(ep[5], ep_out[5], 1e-6));
    
    return 0;
}

/* Test: Coordinate conversion (ECEF to POS) */
static char * test_coordinate_conversion() {
    double pos[3], r[3], pos_out[3];
    
    /* Shin-yokohama (approx) */
    pos[0] = 35.5 * D2R;
    pos[1] = 139.6 * D2R;
    pos[2] = 10.0;
    
    pos2ecef(pos, r);
    ecef2pos(r, pos_out);
    
    mu_assert("error, lat mismatch", is_close(pos[0], pos_out[0], 1e-8));
    mu_assert("error, lon mismatch", is_close(pos[1], pos_out[1], 1e-8));
    mu_assert("error, hgt mismatch", is_close(pos[2], pos_out[2], 1e-4));
    
    return 0;
}

/* Test: Matrix operations */
static char * test_matrix_ops() {
    double A[4] = {1.0, 2.0, 3.0, 4.0};
    double B[4] = {0};
    double I[4] = {1.0, 0.0, 0.0, 1.0};
    
    matcpy(B, A, 2, 2);
    mu_assert("error, matcpy 0", is_close(B[0], 1.0, 1e-9));
    mu_assert("error, matcpy 3", is_close(B[3], 4.0, 1e-9));
    
    /* Invert identity matrix */
    matinv(I, 2);
    mu_assert("error, matinv 0", is_close(I[0], 1.0, 1e-9));
    mu_assert("error, matinv 3", is_close(I[3], 1.0, 1e-9));
    
    return 0;
}

static char * all_tests() {
    mu_run_test(test_time_conversion);
    mu_run_test(test_coordinate_conversion);
    mu_run_test(test_matrix_ops);
    return 0;
}

int main(int argc, char **argv) {
    char *result = all_tests();
    if (result != 0) {
        printf("TEST FAILED: %s\n", result);
    } else {
        printf("ALL TESTS PASSED\n");
    }
    printf("Tests run: %d\n", tests_run);
    
    return result != 0;
}
