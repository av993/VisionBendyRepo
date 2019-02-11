package org.usfirst.frc.team303.robot;

import edu.wpi.first.wpilibj.Timer;

public class ActionDriveToGoalByArea extends ActionAbstract implements Action {

	boolean goalFinished;
	boolean firstRun;
	int stopWidth;
	Timer timer;
	
	public ActionDriveToGoalByArea(int stopWidth) {
		firstRun = true;
		this.stopWidth = stopWidth;
		timer = new Timer();
	}
	
	@Override
	public void run() {
		if(firstRun) {
			timer.reset();
			timer.start();
			Robot.drivebase.zeroEncoder();
			firstRun = false;
		}
		double minPower = 0.53;
		//double scaledCameraArea = (((-(Robot.getArea()-700))+14000)/20000); //scale the power //23000
		
		double scaledPower = 0.6;

		double offset = 0.0;
		double tuningConstant = 0.01;
		System.out.println(getCameraDegreeOffset());

		if (timer.get() <= 0.75) {
			offset = 7; //8
			scaledPower = 0.6;
			//System.out.println(timer.get());
		}
		double degreeOffset = getCameraDegreeOffset();
		degreeOffset += offset;
		double[] pow = driveStraightAngle(scaledPower, (degreeOffset + offset), tuningConstant); //power was 0.55
		Robot.drivebase.drive(pow[0], pow[1]);

		/*double turn = 0;
		double error = Math.toRadians(degreeOffset);
		if (error /90 <= 0.4) {
			turn = 0.4 * Math.signum(error);
		} else {
			turn = (error / 90) * Math.signum(error);
		}*/




		//Robot.drivebase.drive.arcadeDrive(scaledPower, -turn);
		
	}

	@Override
	public boolean isFinished() {
		return (Robot.camera.getWidth() >= stopWidth);	
	}	
	
}
