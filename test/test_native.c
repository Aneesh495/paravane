/*
 * test/test_native.c
 * Verification suite for native C acceleration routines.
 */

#include "../c/paravane_native.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <assert.h>

static int g_tests_run = 0;
static int g_tests_passed = 0;

#define TEST_ASSERT(cond, msg) do { \
    g_tests_run++; \
    if (cond) { \
        g_tests_passed++; \
    } else { \
        fprintf(stderr, "FAIL: %s (line %d)\n", msg, __LINE__); \
        exit(1); \
    } \
} while (0)

static void test_crc16_ccitt(void) {
    /* Standard test vector: "123456789" -> 0x29B1 */
    const uint8_t vec[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
    uint16_t crc = paravane_crc16_ccitt(vec, sizeof(vec));
    TEST_ASSERT(crc == 0x29B1, "CRC-16-CCITT for '123456789' must equal 0x29B1");
}

static void test_mat15_ops(void) {
    double id[225];
    double a[225];
    double out[225];

    for (int i = 0; i < 225; i++) {
        id[i] = (i % 16 == 0) ? 1.0 : 0.0;
        a[i] = (double)(i + 1);
    }

    paravane_mat15_mul(a, id, out);
    int match = 1;
    for (int i = 0; i < 225; i++) {
        if (fabs(out[i] - a[i]) > 1e-9) {
            match = 0;
            break;
        }
    }
    TEST_ASSERT(match == 1, "Matrix multiplication by 15x15 identity must preserve matrix");

    paravane_mat15_transpose(a, out);
    TEST_ASSERT(fabs(out[0 * 15 + 1] - a[1 * 15 + 0]) < 1e-9, "Transpose element (0,1) must equal (1,0)");
    TEST_ASSERT(fabs(out[3 * 15 + 7] - a[7 * 15 + 3]) < 1e-9, "Transpose element (3,7) must equal (7,3)");
}

static void test_point_cloud(void) {
    ParavanePoint3D pts[4] = {
        {0.0, 0.0, 0.0},
        {2.0, 0.0, 0.0},
        {0.0, 2.0, 0.0},
        {2.0, 2.0, 0.0}
    };
    ParavanePoint3D centroid;
    double count = paravane_point_cloud_centroid(pts, 4, &centroid);
    TEST_ASSERT(fabs(count - 4.0) < 1e-9, "Centroid count must equal 4");
    TEST_ASSERT(fabs(centroid.x - 1.0) < 1e-9, "Centroid X must equal 1.0");
    TEST_ASSERT(fabs(centroid.y - 1.0) < 1e-9, "Centroid Y must equal 1.0");
    TEST_ASSERT(fabs(centroid.z - 0.0) < 1e-9, "Centroid Z must equal 0.0");

    ParavanePoint3D targets[4] = {
        {0.1, 0.1, 0.0},
        {2.1, 0.1, 0.0},
        {0.1, 2.1, 0.0},
        {2.1, 2.1, 0.0}
    };
    ParavanePoint3D paired[4];
    double mean_d2 = paravane_icp_nearest_neighbors(pts, 4, targets, 4, paired);
    TEST_ASSERT(mean_d2 > 0.0 && mean_d2 < 0.1, "Nearest neighbor mean squared distance must be small");
}

static void test_acoustic_ray_tracing(void) {
    double depths[5] = {0.0, 500.0, 1000.0, 1500.0, 2000.0};
    double speeds[5] = {1520.0, 1500.0, 1485.0, 1495.0, 1515.0};
    ParavaneRayPoint pts[1000];
    ParavaneRaySummary summary;

    int rc = paravane_trace_ray_native(
        depths, speeds, 5,
        1000.0, 0.05,
        5000.0, 5.0,
        2500.0,
        pts, 1000,
        &summary
    );

    TEST_ASSERT(rc == 0, "Ray trace must return 0 on success");
    TEST_ASSERT(summary.count > 10, "Ray trace must generate at least 10 trajectory points");
    TEST_ASSERT(summary.final_x > 1000.0, "Ray must propagate forward beyond 1000m");
    TEST_ASSERT(summary.travel_time > 0.5, "Travel time must be positive and non-trivial");
}

int main(void) {
    printf("==> Running Paravane C Native Acceleration tests...\n");
    test_crc16_ccitt();
    test_mat15_ops();
    test_point_cloud();
    test_acoustic_ray_tracing();
    printf("    All %d native tests passed successfully.\n", g_tests_passed);
    return 0;
}
