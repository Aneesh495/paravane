/*
 * c/paravane_native.c
 * Native C11 acceleration implementations for Paravane subsea robotics.
 */

#include "paravane_native.h"
#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* -------------------------------------------------------------------------
 * Acoustic Ray Tracing via Runge-Kutta 4th-Order Integration
 * ------------------------------------------------------------------------- */

static void eval_sound_speed_linear(
    const double *depths,
    const double *speeds,
    size_t count,
    double z,
    double *out_c,
    double *out_g
) {
    if (z <= depths[0]) {
        *out_c = speeds[0];
        *out_g = 0.0;
        return;
    }
    if (z >= depths[count - 1]) {
        *out_c = speeds[count - 1];
        *out_g = 0.0;
        return;
    }

    size_t low = 0;
    size_t high = count - 1;
    while (high - low > 1) {
        size_t mid = (low + high) / 2;
        if (depths[mid] <= z) {
            low = mid;
        } else {
            high = mid;
        }
    }

    double dz = depths[high] - depths[low];
    double dc = speeds[high] - speeds[low];
    double g = (dz > 1e-7) ? (dc / dz) : 0.0;
    *out_c = speeds[low] + g * (z - depths[low]);
    *out_g = g;
}

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
) {
    if (!depths || !speeds || layer_count < 2 || !out_summary) {
        return -1;
    }

    double cur_x = 0.0;
    double cur_z = z_src;
    double cur_theta = theta_src;
    double cur_tau = 0.0;
    double cur_s = 0.0;
    uint32_t surf_bounces = 0;
    uint32_t bot_bounces = 0;

    size_t pt_idx = 0;
    if (out_points && max_points > 0) {
        out_points[pt_idx].x = cur_x;
        out_points[pt_idx].z = cur_z;
        out_points[pt_idx].theta = cur_theta;
        out_points[pt_idx].tau = cur_tau;
        out_points[pt_idx].length = cur_s;
        pt_idx++;
    }

    while (cur_x < max_range && cur_tau < 30.0 && (!out_points || pt_idx < max_points)) {
        double c1, g1;
        eval_sound_speed_linear(depths, speeds, layer_count, cur_z, &c1, &g1);
        double k1_x = cos(cur_theta);
        double k1_z = sin(cur_theta);
        double k1_th = (-1.0 / c1) * g1 * k1_x;
        double k1_tau = 1.0 / c1;

        double z2 = cur_z + 0.5 * step_size * k1_z;
        double th2 = cur_theta + 0.5 * step_size * k1_th;
        double c2, g2;
        eval_sound_speed_linear(depths, speeds, layer_count, z2, &c2, &g2);
        double k2_x = cos(th2);
        double k2_z = sin(th2);
        double k2_th = (-1.0 / c2) * g2 * k2_x;
        double k2_tau = 1.0 / c2;

        double z3 = cur_z + 0.5 * step_size * k2_z;
        double th3 = cur_theta + 0.5 * step_size * k2_th;
        double c3, g3;
        eval_sound_speed_linear(depths, speeds, layer_count, z3, &c3, &g3);
        double k3_x = cos(th3);
        double k3_z = sin(th3);
        double k3_th = (-1.0 / c3) * g3 * k3_x;
        double k3_tau = 1.0 / c3;

        double z4 = cur_z + step_size * k3_z;
        double th4 = cur_theta + step_size * k3_th;
        double c4, g4;
        eval_sound_speed_linear(depths, speeds, layer_count, z4, &c4, &g4);
        double k4_x = cos(th4);
        double k4_z = sin(th4);
        double k4_th = (-1.0 / c4) * g4 * k4_x;
        double k4_tau = 1.0 / c4;

        double dx = (step_size / 6.0) * (k1_x + 2.0 * k2_x + 2.0 * k3_x + k4_x);
        double dz = (step_size / 6.0) * (k1_z + 2.0 * k2_z + 2.0 * k3_z + k4_z);
        double dth = (step_size / 6.0) * (k1_th + 2.0 * k2_th + 2.0 * k3_th + k4_th);
        double dtau = (step_size / 6.0) * (k1_tau + 2.0 * k2_tau + 2.0 * k3_tau + k4_tau);

        cur_x += dx;
        cur_z += dz;
        cur_theta += dth;
        cur_tau += dtau;
        cur_s += step_size;

        if (cur_z <= 0.0) {
            cur_z = fabs(cur_z);
            cur_theta = fabs(cur_theta);
            surf_bounces++;
        }
        if (cur_z >= bottom_depth) {
            cur_z = 2.0 * bottom_depth - cur_z;
            cur_theta = -fabs(cur_theta);
            bot_bounces++;
        }

        if (out_points && pt_idx < max_points) {
            out_points[pt_idx].x = cur_x;
            out_points[pt_idx].z = cur_z;
            out_points[pt_idx].theta = cur_theta;
            out_points[pt_idx].tau = cur_tau;
            out_points[pt_idx].length = cur_s;
            pt_idx++;
        }
    }

    out_summary->count = pt_idx;
    out_summary->final_x = cur_x;
    out_summary->final_z = cur_z;
    out_summary->travel_time = cur_tau;
    out_summary->surface_bounces = surf_bounces;
    out_summary->bottom_bounces = bot_bounces;

    return 0;
}

/* -------------------------------------------------------------------------
 * 15x15 Matrix Multiplication
 * ------------------------------------------------------------------------- */

void paravane_mat15_mul(const double *a, const double *b, double *out) {
    for (int r = 0; r < 15; r++) {
        for (int c = 0; c < 15; c++) {
            double sum = 0.0;
            for (int k = 0; k < 15; k++) {
                sum += a[r * 15 + k] * b[k * 15 + c];
            }
            out[r * 15 + c] = sum;
        }
    }
}

void paravane_mat15_transpose(const double *m, double *out) {
    for (int r = 0; r < 15; r++) {
        for (int c = 0; c < 15; c++) {
            out[c * 15 + r] = m[r * 15 + c];
        }
    }
}

void paravane_mat15_add(const double *a, const double *b, double *out) {
    for (int i = 0; i < 225; i++) {
        out[i] = a[i] + b[i];
    }
}

/* -------------------------------------------------------------------------
 * Point Cloud Operations
 * ------------------------------------------------------------------------- */

double paravane_point_cloud_centroid(
    const ParavanePoint3D *points,
    size_t count,
    ParavanePoint3D *out_centroid
) {
    if (!points || count == 0 || !out_centroid) return 0.0;
    double sx = 0.0, sy = 0.0, sz = 0.0;
    for (size_t i = 0; i < count; i++) {
        sx += points[i].x;
        sy += points[i].y;
        sz += points[i].z;
    }
    out_centroid->x = sx / (double)count;
    out_centroid->y = sy / (double)count;
    out_centroid->z = sz / (double)count;
    return (double)count;
}

double paravane_icp_nearest_neighbors(
    const ParavanePoint3D *sources,
    size_t src_count,
    const ParavanePoint3D *targets,
    size_t tgt_count,
    ParavanePoint3D *out_paired_targets
) {
    if (!sources || !targets || src_count == 0 || tgt_count == 0 || !out_paired_targets) {
        return -1.0;
    }

    double total_dist2 = 0.0;
    for (size_t i = 0; i < src_count; i++) {
        double min_d2 = 1e30;
        size_t best_j = 0;
        double sx = sources[i].x;
        double sy = sources[i].y;
        double sz = sources[i].z;

        for (size_t j = 0; j < tgt_count; j++) {
            double dx = targets[j].x - sx;
            double dy = targets[j].y - sy;
            double dz = targets[j].z - sz;
            double d2 = dx * dx + dy * dy + dz * dz;
            if (d2 < min_d2) {
                min_d2 = d2;
                best_j = j;
            }
        }
        out_paired_targets[i] = targets[best_j];
        total_dist2 += min_d2;
    }
    return total_dist2 / (double)src_count;
}

/* -------------------------------------------------------------------------
 * Zero-Allocation CRC-16-CCITT
 * ------------------------------------------------------------------------- */

uint16_t paravane_crc16_ccitt(const uint8_t *data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; i++) {
        uint8_t shifted = (uint8_t)(((crc >> 8) ^ data[i]) & 0xFF);
        uint8_t temp = (uint8_t)(shifted ^ (shifted >> 4));
        crc = (uint16_t)((crc << 8) ^ ((uint16_t)temp << 12) ^ ((uint16_t)temp << 5) ^ (uint16_t)temp);
    }
    return crc;
}
