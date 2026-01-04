#ifndef STEERING_GAIN_CALIBRATION_H
#define STEERING_GAIN_CALIBRATION_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Maximum calibration points */
#define MAX_STEERING_GAIN_POINTS 1000
#define MIN_VALID_VELOCITY 0.1
#define MAX_VALID_VELOCITY 25.0
#define MAX_STEERING_ANGLE 0.7854  /* pi/4 radians */
#define MAD_OUTLIER_THRESHOLD 2.5
#define MAD_SEVERE_OUTLIER_THRESHOLD 3.5
#define DEFAULT_WHEELBASE 2.7

/* Single calibration point */
typedef struct {
    double timestamp;
    double steering_angle;
    double heading_error;
    double velocity;
    double rtk_quality;
    double trajectory_distance;
    uint8_t is_outlier;
    double computed_weight;
    double residual;
} SteeringGainCalibrationPoint_t;

/* Calibration manager */
typedef struct {
    SteeringGainCalibrationPoint_t points[MAX_STEERING_GAIN_POINTS];
    uint32_t point_count;
    
    double estimated_gain;
    double gain_bias;
    double gain_uncertainty;
    
    double r_squared;
    double weighted_std_dev;
    double confidence;
    
    uint32_t valid_point_count;
    uint32_t outlier_count;
    double average_weight;
    double total_weight;
    
    uint8_t is_valid;
    double min_velocity;
    double max_velocity;
    double min_steering_angle;
    double max_steering_angle;
} SteeringGainManager_t;

/* Core functions */
int steering_gain_manager_init(SteeringGainManager_t *manager);

int steering_gain_add_point(
    SteeringGainManager_t *manager,
    double steering_angle,
    double heading_error,
    double velocity,
    double rtk_quality,
    double trajectory_distance,
    double timestamp
);

double steering_gain_compute_weight(
    double steering_angle,
    double velocity,
    double rtk_quality,
    double trajectory_distance
);

int steering_gain_detect_outliers(
    SteeringGainManager_t *manager,
    uint8_t *outlier_flags,
    uint32_t *outlier_count
);

int steering_gain_estimate(
    SteeringGainManager_t *manager,
    uint32_t *outlier_indices,
    uint32_t *outlier_count
);

double steering_gain_get_estimated_gain(const SteeringGainManager_t *manager);
double steering_gain_get_uncertainty(const SteeringGainManager_t *manager);
double steering_gain_get_confidence(const SteeringGainManager_t *manager);
int steering_gain_is_valid(const SteeringGainManager_t *manager);

void steering_gain_print_diagnostics(const SteeringGainManager_t *manager);
void steering_gain_reset(SteeringGainManager_t *manager);

uint32_t steering_gain_get_point_count(const SteeringGainManager_t *manager);

int steering_gain_get_point(const SteeringGainManager_t *manager, uint32_t index,
                           SteeringGainCalibrationPoint_t *point);

int steering_gain_load_csv(SteeringGainManager_t *manager, const char *filename);
int steering_gain_save_csv(const SteeringGainManager_t *manager, const char *filename);
int steering_gain_save_diagnostics_csv(const SteeringGainManager_t *manager,
                                     const char *filename);

#ifdef __cplusplus
}
#endif

#endif
