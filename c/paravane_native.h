/*
 * c/paravane_native.h
 * High-performance native acceleration routines for Paravane subsea robotics.
 *
 * Provides:
 *   1. Hardware-accelerated Snell acoustic ray tracer across stratified profiles.
 *   2. Vectorized 15x15 matrix algebra and Joseph form covariance propagation.
 *   3. SIMD-friendly 3D point cloud nearest neighbor and Arun ICP inner loops.
 *   4. Zero-allocation bitfield packing and CRC-16-CCITT for acoustic modems.
 */

#ifndef PARAVANE_NATIVE_H
#define PARAVANE_NATIVE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* --- Acoustic Ray Tracing --- */

typedef struct {
    double x;
    double z;
    double theta;
    double tau;
    double length;
} ParavaneRayPoint;

typedef struct {
    size_t count;
    double final_x;
    double final_z;
    double travel_time;
    uint32_t surface_bounces;
    uint32_t bottom_bounces;
} ParavaneRaySummary;

int paravane_trace_ray_native(
    const double *depths,
    const double *speeds,
    size_t layer_count,
    double z_src,
    double theta_src,
    double max_range,
    double step_size,
    double bottom_depth,
    ParavaneRayPoint *out_points,
    size_t max_points,
    ParavaneRaySummary *out_summary
);

/* --- 15x15 Matrix Algebra for ES-EKF --- */

void paravane_mat15_mul(const double *a, const double *b, double *out);
void paravane_mat15_transpose(const double *m, double *out);
void paravane_mat15_add(const double *a, const double *b, double *out);

/* --- Point Cloud Registration (ICP) --- */

typedef struct {
    double x;
    double y;
    double z;
} ParavanePoint3D;

double paravane_point_cloud_centroid(
    const ParavanePoint3D *points,
    size_t count,
    ParavanePoint3D *out_centroid
);

double paravane_icp_nearest_neighbors(
    const ParavanePoint3D *sources,
    size_t src_count,
    const ParavanePoint3D *targets,
    size_t tgt_count,
    ParavanePoint3D *out_paired_targets
);

/* --- Fast CRC-16-CCITT --- */

uint16_t paravane_crc16_ccitt(const uint8_t *data, size_t length);

#ifdef __cplusplus
}
#endif

#endif /* PARAVANE_NATIVE_H */
