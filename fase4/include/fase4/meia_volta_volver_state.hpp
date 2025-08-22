#include <Eigen/Eigen>
#include "fsm/fsm.hpp"
#include "drone/Drone.hpp"
#include "fase4/PidController.hpp"
#include <chrono>
#include <cmath>

class MeiaVoltaVolverState : public fsm::State {
private:
    std::shared_ptr<Drone> drone;
    
    float max_yaw_rate;
    float initial_yaw;
    float target_yaw;
    float yaw_tolerance;
    float volver_timeout;

    bool rotation_completed;

    std::chrono::steady_clock::time_point start_time;
    std::unique_ptr<PidController> yaw_pid;
    
    static constexpr float PI = 3.14159f;

public:
    MeiaVoltaVolverState() : fsm::State() {}

    void on_enter(fsm::Blackboard& bb) override {
        auto drone_ptr = bb.get<std::shared_ptr<Drone>>("drone");
        
        if (drone_ptr == nullptr || *drone_ptr == nullptr) {
            this->drone->log("ERROR: Drone pointer is null in MeiaVoltaVolverState");
            return;
        }
        
        this->drone = *drone_ptr;

        this->drone->log("STATE: MEIA VOLTA VOLVER STATE");

        this->max_yaw_rate = *bb.get<float>("max_yaw_rate");
        this->yaw_tolerance = *bb.get<float>("yaw_tolerance");
        this->volver_timeout = *bb.get<float>("volver_timeout");
        
        this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
        
        this->initial_yaw = this->drone->getOrientation()[2];
        this->target_yaw = this->initial_yaw + PI;
        
        // Normalizar target_yaw para [-pi, pi]
        while (this->target_yaw > PI) {
            this->target_yaw -= 2 * PI;
        }
        while (this->target_yaw < -PI) {
            this->target_yaw += 2 * PI;
        }
        
        float kp_yaw = *bb.get<float>("pid_angular_kp");
        float ki_yaw = *bb.get<float>("pid_angular_ki");
        float kd_yaw = *bb.get<float>("pid_angular_kd");
        
        this->yaw_pid = std::make_unique<PidController>(
            kp_yaw, ki_yaw, kd_yaw, this->target_yaw
        );
        
        this->rotation_completed = false;
        this->start_time = std::chrono::steady_clock::now();
    }

    std::string act(fsm::Blackboard& bb) override {
        (void) bb; // Evitar warning de parâmetro não usado

        auto current_time = std::chrono::steady_clock::now();
        auto elapsed_time = std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count();

        if (elapsed_time > this->volver_timeout) {
            this->drone->log("TIMEOUT: Rotation exceeded maximum time");
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "TIMEOUT";
        }

        float current_yaw = this->drone->getOrientation()[2];
        
        float yaw_error = this->target_yaw - current_yaw;
        
        // Normalizar erro para o caminho mais curto (entre -pi e pi)
        while (yaw_error > PI) {
            yaw_error -= 2 * PI;
        }
        while (yaw_error < -PI) {
            yaw_error += 2 * PI;
        }

        if (std::abs(yaw_error) < this->yaw_tolerance) {
            if (!this->rotation_completed) {
                this->rotation_completed = true;
            }
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            return "VOLVER COMPLETED";
        }
        
        float yaw_rate = this->yaw_pid->compute(current_yaw);
        yaw_rate = std::clamp(yaw_rate, -this->max_yaw_rate, this->max_yaw_rate);
        
        this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, yaw_rate);
        
        return "";
    }

    void on_exit(fsm::Blackboard& bb) override {
        (void) bb;
        // Parar o drone quando sair do estado
        if (this->drone != nullptr) {
            this->drone->setLocalVelocity(0.0f, 0.0f, 0.0f, 0.0f);
            this->drone->log("Exiting MeiaVoltaVolverState - stopping drone");
        }
    }
};