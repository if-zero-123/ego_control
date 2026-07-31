#include "d_task_uav_control/detection_source_selector.h"

#include <cmath>

namespace d_task_uav_control {
namespace {

bool fresh(bool received, double received_s, double now_s, double timeout_s) {
    return received
        && std::isfinite(received_s)
        && received_s <= now_s + 1e-6
        && now_s - received_s <= timeout_s;
}

}  // namespace

DetectionSource selectDetectionSource(
    double now_s,
    double timeout_s,
    bool apriltag_enabled,
    bool yolo_received,
    bool yolo_found,
    double yolo_received_s,
    bool apriltag_received,
    bool apriltag_found,
    double apriltag_received_s) {
    if (!std::isfinite(now_s) || !std::isfinite(timeout_s)
        || timeout_s <= 0.0) {
        return DetectionSource::NONE;
    }
    const bool yolo_fresh =
        fresh(yolo_received, yolo_received_s, now_s, timeout_s);
    const bool apriltag_fresh =
        apriltag_enabled
        && fresh(apriltag_received, apriltag_received_s, now_s, timeout_s);

    if (apriltag_fresh && apriltag_found) {
        return DetectionSource::APRILTAG;
    }
    if (yolo_fresh && yolo_found) {
        return DetectionSource::YOLO;
    }
    if (apriltag_fresh
        && (!yolo_fresh || apriltag_received_s >= yolo_received_s)) {
        return DetectionSource::APRILTAG;
    }
    if (yolo_fresh) {
        return DetectionSource::YOLO;
    }
    return DetectionSource::NONE;
}

const char* detectionSourceName(DetectionSource source) {
    switch (source) {
        case DetectionSource::YOLO:
            return "YOLO";
        case DetectionSource::APRILTAG:
            return "APRILTAG";
        case DetectionSource::NONE:
        default:
            return "NONE";
    }
}

}  // namespace d_task_uav_control
