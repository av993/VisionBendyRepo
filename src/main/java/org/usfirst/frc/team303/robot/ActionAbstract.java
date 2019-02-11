package org.usfirst.frc.team303.robot;
import org.usfirst.frc.team303.robot.Robot;

public abstract class ActionAbstract {
	
	static final double pixelPerDegreeConstant = 0.146875;
	static final double offsetConstant = 30;
	
	public static double getCameraDegreeOffset() {
		return Robot.camera.getCameraDegreeOffset();
	}

	

	public static double[] driveStraightAngle(double powSetpoint, double angleDifference, double tuningConstant) {                                                                                                                      //memes
		return new double[] {(powSetpoint + (angleDifference*tuningConstant)), (powSetpoint - (angleDifference*tuningConstant))};
	}
}
