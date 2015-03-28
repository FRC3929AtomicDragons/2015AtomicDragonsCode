package org.usfirst.frc.team3929.robot;

public class PIDTool {

    private double kP, kI, kD;
    private double integral,lastX;
    private double setpoint;
    private double min_control, max_control;
    
    public PIDTool(double kP, double kI, double kD, double setpoint, double min_control, double max_control){
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.setpoint = setpoint;
        this.min_control = min_control;
        this.max_control = max_control;
        
        integral = 0.0;
        lastX = 0;
        
    }
    
    public double computeControl (double x) {
        double e = x - setpoint;
        double deriv = x - lastX;
        this.integral += e;
        lastX = x;
        double control = ((kP * e) + (kI * integral) + (kD * deriv));
        
        if (control < min_control) control = min_control;
        if (control > max_control) control = max_control;
        
        return control;
    }
    
    public void reset() {
        lastX = 0.0;
        integral = 0.0;
        setpoint = 0.0;
    }
    
    public void setSetpoint (double setpoint) {
        this.setpoint = setpoint;
    }
    
    public void setControlLimits (double min_control, double max_control) {
    	this.min_control = min_control;
    	this.max_control = max_control;
    }
}

