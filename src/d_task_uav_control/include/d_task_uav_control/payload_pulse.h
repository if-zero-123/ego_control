#ifndef D_TASK_UAV_CONTROL_PAYLOAD_PULSE_H_
#define D_TASK_UAV_CONTROL_PAYLOAD_PULSE_H_

namespace d_task_uav_control {

enum class PayloadPulseCommand {
    NONE,
    RELEASE,
    NEUTRAL,
};

class PayloadPulse {
public:
    explicit PayloadPulse(double pulse_duration_s);

    bool trigger(double now_s);
    PayloadPulseCommand update(double now_s);
    PayloadPulseCommand reset();

private:
    double pulse_duration_s_ = 0.0;
    bool triggered_ = false;
    bool active_ = false;
    bool neutral_pending_ = false;
    double release_started_s_ = 0.0;
};

}  // namespace d_task_uav_control

#endif  // D_TASK_UAV_CONTROL_PAYLOAD_PULSE_H_
