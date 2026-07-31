#ifndef D_TASK_UAV_CONTROL_DETECTION_SOURCE_SELECTOR_H_
#define D_TASK_UAV_CONTROL_DETECTION_SOURCE_SELECTOR_H_

#include <cstdint>

namespace d_task_uav_control {

enum class DetectionSource : uint8_t {
    NONE = 0,
    YOLO,
    APRILTAG,
};

DetectionSource selectDetectionSource(
    double now_s,
    double timeout_s,
    bool apriltag_enabled,
    bool yolo_received,
    bool yolo_found,
    double yolo_received_s,
    bool apriltag_received,
    bool apriltag_found,
    double apriltag_received_s);

const char* detectionSourceName(DetectionSource source);

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_DETECTION_SOURCE_SELECTOR_H_
