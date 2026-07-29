#ifndef EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_
#define EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_

#include <cmath>

namespace ego_bridge {

class PlatformLandingDetector {
public:
    PlatformLandingDetector(double minimum_detection_time_s,
                            double position_error_threshold_m,
                            double vertical_speed_threshold_mps,
                            double hold_time_s)
        : minimum_detection_time_s_(minimum_detection_time_s),
          position_error_threshold_m_(position_error_threshold_m),
          vertical_speed_threshold_mps_(vertical_speed_threshold_mps),
          hold_time_s_(hold_time_s) {}

    bool update(double elapsed_s, double position_error_z_m,
                double vertical_speed_mps) {
        const bool contact_condition =
            elapsed_s >= minimum_detection_time_s_
            && position_error_z_m < position_error_threshold_m_
            && std::abs(vertical_speed_mps) < vertical_speed_threshold_mps_;
        if (!contact_condition) {
            active_ = false;
            return false;
        }
        if (!active_) {
            active_ = true;
            condition_start_s_ = elapsed_s;
            return hold_time_s_ <= 0.0;
        }
        return elapsed_s - condition_start_s_ >= hold_time_s_;
    }

    void reset() {
        active_ = false;
        condition_start_s_ = 0.0;
    }

private:
    double minimum_detection_time_s_;
    double position_error_threshold_m_;
    double vertical_speed_threshold_mps_;
    double hold_time_s_;
    bool active_ = false;
    double condition_start_s_ = 0.0;
};

}  // namespace ego_bridge

#endif  // EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_
