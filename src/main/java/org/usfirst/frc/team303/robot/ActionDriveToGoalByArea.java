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

		 
		
		
		double scaledCameraArea = 0.6;

		double offset = 0.0;
		double tuningConstant = 0.01;
		System.out.println(getCameraDegreeOffset());

		if (timer.get() <= 1) {
			offset = 0;
			scaledCameraArea = 0.65;
			//System.out.println(timer.get());
		}


		//scaledCameraArea = scaledCameraArea > minPower ? scaledCameraArea : minPower;

		double degreeOffset = getCameraDegreeOffset();
		
		double[] pow = driveStraightAngle(scaledCameraArea, (degreeOffset + offset), tuningConstant); //power was 0.55
		Robot.drivebase.drive(pow[0], pow[1]);
		//Robot.drivebase.drive(0.5, 0.7);
		
	}

	@Override
	public boolean isFinished() {
		if(firstRun) {
			Robot.drivebase.zeroEncoder();
			firstRun = false;
			return false;
		} else {
			return (Robot.navX.collisionDetected() || Robot.getWidth() > stopWidth);
		}	
	}	
	


	public void beRowdy(boolean isRowdy){
		if (isRowdy == true){
			//stopBeingRowdy();
		}
	}
}
