package org.usfirst.frc.team303.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;

public class ActionTurnToGoal extends ActionAbstract implements Action{

	static final double offsetConstant = 50; 
	static final double pixelPerDegreeConstant = 0.1468; //0.084
	double degreeSetpoint = 0;
	ActionTurnToAngle angleTurn;
	boolean firstRun;
	Timer timer;
	
	/**
	 * Turns to the goal based on offset from center X pixel.
	 * @deprecated Does not reliably work. Use ActionDriveToGoal instead.
	 */
	@Deprecated
	public ActionTurnToGoal() {
		firstRun = true;
	}

	@Override
	public void run() {
		
		if(firstRun) {
			
			double degRelSetpoint = getCameraDegreeOffset();
			SmartDashboard.putNumber("Degree Offset", degRelSetpoint);
			angleTurn = new ActionTurnToAngle(degRelSetpoint, true, 1, false, 1, false);
			firstRun = false;
		} else {
			angleTurn.run();
		}

	}

	@Override
	public boolean isFinished() {
		boolean end = false;
		
		if(!firstRun) {
			end = angleTurn.isFinished();
		} 
		
		if(end) {
			firstRun = true;
		}
		
		return end;
	}
		
}
