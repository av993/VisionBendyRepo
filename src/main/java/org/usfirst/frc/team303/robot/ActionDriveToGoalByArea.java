package org.usfirst.frc.team303.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ActionDriveToGoalByArea extends ActionAbstract implements Action {

	boolean goalFinished;
	boolean firstRun;
	int stopWidth;
	Timer timer;
	double desiredHeading;

	static final double FOCAL_LENGTH_PIXELS = (640 / 2) / Math.tan(75 / 2.0);


	public static final double A = -1.029258435778039;
	public static final double B = 0.12131357724670491;
	public static final double C = -0.0004412063928563716;
	public static final double D = 7.653889638753478e-7;
	public static boolean turnDone = false;
	public static final double WIDTH_OF_TARGET_INCHES = 8;

	
	public ActionDriveToGoalByArea(int stopWidth, double desiredHeading) {
		firstRun = true;
		this.stopWidth = stopWidth;
		this.desiredHeading = desiredHeading;
		timer = new Timer();
	}
	
	@Override
	public void run() {
		
		double overturn = 0.0;
		double originalPower = 0.6;
		double distanceToCenterX = Robot.camera.getCentDist();
		double targetWidthPixels = Robot.camera.getWidth();
		double distanceFromTargetInches = 6040 / targetWidthPixels;
		
		double offsetFromTargetInches = (distanceToCenterX * WIDTH_OF_TARGET_INCHES) / targetWidthPixels;
		double originalHeading = Robot.navX.getOriginalHeading();
		
		//double cameraOffset = Math.toDegrees(Math.atan(offsetFromTargetInches / distanceFromTargetInches));
		
		double cameraOffset = Robot.camera.getCameraDegreeOffset();		

		
		//Gives you the pixel offset if you are at an angle
		double differenceInHeading = desiredHeading - originalHeading;
		double inchesOffset = distanceFromTargetInches * Math.tan(Math.toRadians(differenceInHeading));
		double pixelOffset = inchesOffset * (targetWidthPixels / WIDTH_OF_TARGET_INCHES);
		distanceToCenterX += pixelOffset;

		//scaledPower = originalPower;

		if(firstRun) {
			timer.reset();
			timer.start();
			Robot.drivebase.zeroEncoder();

			if (Math.abs(distanceToCenterX + pixelOffset) <=25) {
				overturn = 0;
			} else {
				System.out.println("RUN DUMB");
				double X = Math.abs(distanceToCenterX + pixelOffset);
				X = 100;
				overturn = A + (B * X) + (C * Math.pow(X, 2)) + (D * Math.pow(X,3));
				Math.copySign(overturn, -(distanceToCenterX + pixelOffset));
				System.out.println("OVERTURN: " + overturn);
				overturn += 5;
			}

			firstRun = false;

		}
	
		System.out.println("OVERTURN: " + overturn);

		double scaledPower = 0.6; 

		if (timer.get() <= 0.5) {
			scaledPower = 0.6;
			overturn = 12;
			turnDone = true;
		} else {
			overturn = 0;
		}
		



		double[] pow = driveStraightAngle(scaledPower, (cameraOffset + overturn), 0.01); //power was 0.55
		Robot.drivebase.drive(pow[0], pow[1]);	
	}

	@Override
	public boolean isFinished() {
		return (Robot.camera.getWidth() >= stopWidth);	
	}	
	
}
