#include "../include/PID.hpp"


PID::PID() {}

PID::PID(int direction, float sample_time_ms, float kp, float ki, float kd){

    setControllerDirection(direction);
    setSampleTime(sample_time_ms);
    setTuningParameters(kp, ki, kd);
    setPIDMode(AUTO);
    Integrator = 0;
    Differentiator = 0;
    prev_time =  std::chrono::duration<float, std::milli>(steady_clock::now().time_since_epoch()).count();
    output_data = 0;
    setOutputLimits(std::numeric_limits<float>::min(), std::numeric_limits<float>::max());
}


void PID::setControllerDirection(int direction) {
	
	controller_direction = direction;
}

void PID::setSampleTime(float new_dt_ms) {
	
	if (new_dt_ms > 0) {
		
		float ratio = new_dt_ms / dt_ms;
		
		integral_gain *= ratio; // (Ki * dt) * integral(e(t) <- IS EQUIVALENT TO -> Ki integral(e(t) * dt)
		derivative_gain /= ratio; // (Kd / dt) * de <- IS EQUIVALENT TO -> Kd * (de/dt)
		dt_ms = new_dt_ms;
	}
}

void PID::setTuningParameters(float kp, float ki, float kd) {
	
	if (proportional_gain < 0.0 || integral_gain < 0.0 || derivative_gain < 0.0) return;
	
	float dt_secs = dt_ms / 1000;
	proportional_gain = kp;
	integral_gain = ki * dt_secs;
	derivative_gain = kd / dt_secs;
	
	if (controller_direction == REVERSE) {				
		
		proportional_gain = 0 - proportional_gain;
		integral_gain = 0 - integral_gain;
		derivative_gain = 0 - derivative_gain;
	}
}


void PID::setPIDMode(int mode) { 
	
	bool newAutoMode = (mode == AUTO);
	
    if (mode == AUTO && pid_mode == MANUAL) 
        previous_measurement = current_measurement;	// Backup. Keep derivative from spiking

    pid_mode = newAutoMode;
}




void PID::setOutputLimits(float min, float max) {
	
	if (min > max) return;
	min_output = min;		
	max_output = max;			
	
	if (output_data > max_output) 
        output_data = max_output;			
	else if (output_data < min_output) 
        output_data = min_output;	
	
	if (Integrator > max_output) 
        Integrator = max_output;	
	else if (Integrator < min_output) 
        Integrator = min_output;
}



float PID::pidUpdate(float set_point, float measurement) {
	
	if (pid_mode == MANUAL) {
        std::cerr << "ERROR: Leave Manual Mode to start PID" << std::endl;
        return -1.f;
    }
	
	float current_time = std::chrono::duration<float, std::milli>(steady_clock::now().time_since_epoch()).count();
	float dt = (current_time - prev_time);
	// std::cout << "Current Time: [" << current_time << "] Prev Time: [" << prev_time << "] DT: " << dt << " DT(ms): " << dt_ms << std::endl;
	// std::cout << "Output Data @ Start: " << output_data << std::endl;
	
	if (dt >= dt_ms) {
		
		current_measurement = measurement;
		
        // Proportional Term
		float current_error = abs(set_point - current_measurement);
		float proportional_term = proportional_gain * current_error;
		
        // Integral Term
		Integrator += integral_gain * current_error;
		if (Integrator > max_output) 
            Integrator = max_output;
		else if (Integrator < min_output) 
            Integrator = min_output;
		
        // Derivative Term
		Differentiator = current_measurement - previous_measurement;
		
        // PID formula
		output_data = proportional_term + Integrator + (derivative_gain * Differentiator);
		// std::cout << "Current PID Stats++++++++++++++++++++++++++++++++++++++++++++++++++\n";
		// std::cout << "Error: " << current_error << "\n";
		// std::cout << "Proportional Term: " << proportional_term << "\n";
		// std::cout << "Integral Term: " << Integrator << "\n";
		// std::cout << "Derrivative Term: " << (derivative_gain * Differentiator) << "\n";
		// std::cout << "Output Before Clamp: " << output_data << "\n";
		
		if (output_data > max_output) 
            output_data = max_output;
		else if (output_data < min_output) 
            output_data = min_output;
		
		// Update Measurement & Time
		previous_measurement = current_measurement;
		prev_time = current_time;
	}
	return output_data;
}


