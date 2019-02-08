package org.usfirst.frc.team303.robot;

import com.kauailabs.navx.frc.*;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class NavX { //this class controls the PID for the navX as well as the AHRS class itself
	AHRS navX;
	PIDController turnController;
	double rate; //this is the output
	double setPoint = 0;
	double last_world_linear_accel_x;
	double last_world_linear_accel_y;
    static double kCollisionThreshold_DeltaG = 0.8f; 
	//static double kDefaultCollisionThreshold_DeltaG = 0.73f;  

	public NavX() {
		navX = new AHRS(SPI.Port.kMXP);
	//	navX.setPIDSourceType(PIDSourceType.kDisplacement);
		
		
	}
		
	public void setSetpoint(double setpoint) {
		setPoint = setpoint;
		turnController.setSetpoint(setpoint);
	}
	
	public double getYaw() {
		return navX.getYaw();
	}

	public void zeroYaw(){
		navX.zeroYaw();
	}

	
	public boolean collisionDetected() {
        boolean collisionDetected = false;
        
        double curr_world_linear_accel_x = navX.getWorldLinearAccelX();
        double currentJerkX = curr_world_linear_accel_x - last_world_linear_accel_x;
        last_world_linear_accel_x = curr_world_linear_accel_x;
        double curr_world_linear_accel_y = navX.getWorldLinearAccelY();
        double currentJerkY = curr_world_linear_accel_y - last_world_linear_accel_y;
        last_world_linear_accel_y = curr_world_linear_accel_y;
        
        if ( ( Math.abs(currentJerkX) > kCollisionThreshold_DeltaG ) ||
             ( Math.abs(currentJerkY) > kCollisionThreshold_DeltaG) ) {
            collisionDetected = true;
        }
        
        SmartDashboard.putBoolean("Collision Detected", collisionDetected);
        return collisionDetected;
	}
	
}
