#ifndef EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_
#define EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_

#include <cmath>

namespace ego_bridge {

class PlatformLandingDetector {
public:
    PlatformLandingDetector(double vertical_speed_threshold_mps,
                            double hold_time_s)
        : vertical_speed_threshold_mps_(vertical_speed_threshold_mps),
          hold_time_s_(hold_time_s) {}

    bool update(double elapsed_s, double vertical_speed_mps) {
        const bool contact_condition =
            std::abs(vertical_speed_mps) < vertical_speed_threshold_mps_;
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
    double vertical_speed_threshold_mps_;
    double hold_time_s_;
    bool active_ = false;
    double condition_start_s_ = 0.0;
};

}  // namespace ego_bridge

#endif  // EGO_BRIDGE_PLATFORM_LANDING_DETECTOR_H_
