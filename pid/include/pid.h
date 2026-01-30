#pragma once

struct PIDParams {
    double kp = 0.0;
    double ki = 0.0;
    double kd = 0.0;
    double output_min = -1.0;
    double output_max = 1.0;
    double integral_max = 100.0;
};

class PID {
    public:
        PID(const PIDParams& params);
        
        double update(double error, double dt);
    
        void reset();
        
        void setGains(double kp, double ki, double kd);
    
    private:
        PIDParams params;
        double integral_sum = 0.0;
        double prev_error = 0.0;
};
