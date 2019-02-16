/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team303.robot;

import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.modifiers.TankModifier;
import jaci.pathfinder.followers.EncoderFollower;
import edu.wpi.first.wpilibj.Timer;


/**
 * Add your docs here.
 */
public class ActionDriveToGoalByTrajectory implements Action {

    public static final double timeStep = 0.02;
    public static final double maxVel = 7.0;
    public static final double maxAccel = 6.0;
    public static final double maxJerk = 50.0;
    public static final double wheelBaseWidth = 50.0;

    private Trajectory leftTrajectory;
    private Trajectory rightTrajectory;
  
    private EncoderFollower leftFollower;
    private EncoderFollower rightFollower;

    public boolean isFirst = false;

    public static double desiredHeading = 0;
    public static final double WIDTH_OF_TARGET_INCHES = 8;
    private static final int k_ticks_per_rev = 2230;
    private static final double k_wheel_diameter = 0.333333;
    private static final double k_max_velocity = 8.0;  
    public double turningConstant = 0.01;

    public Timer timer;

    public int leftEncoder = 0;
    public int rightEncoder = 0;

    public double stopWidth = 270;;


    public ActionDriveToGoalByTrajectory(double desiredHeading, double turningConstant, double stopWidth) {
        this.desiredHeading = desiredHeading;
        this.turningConstant = turningConstant;
        this.stopWidth = stopWidth;
    }

    public void generateTrajectory() {

        //Get Difference In Heading
        double differenceInHeading = Robot.navX.getOriginalHeading() - desiredHeading;
        
        //Need to call this so that the values get constantly updated 
        Robot.camera.getCameraDegreeOffset();
        
        //Get Distance To Center
        double distanceToCenterX = Robot.camera.getCentDist();
		double targetWidthPixels = Robot.camera.getWidth();
        double distanceFromTargetInches = 6040 / targetWidthPixels;
        
        //Recalculated distanceToCenterX
		double inchesOffset = distanceFromTargetInches * Math.tan(Math.toRadians(differenceInHeading));
		double pixelOffset = inchesOffset * (targetWidthPixels / WIDTH_OF_TARGET_INCHES);
        distanceToCenterX += pixelOffset;
		
		double offsetFromTargetInches = (distanceToCenterX * WIDTH_OF_TARGET_INCHES) / targetWidthPixels;
		double originalHeading = Robot.navX.getOriginalHeading();
        
        //Convert From Inches To Feet
        offsetFromTargetInches /= 12;
        distanceFromTargetInches /= 12;

        //Stop One Foot Away and Then Continue Vision
        distanceFromTargetInches -= 1;

        Waypoint[] visionWaypoint = new Waypoint[] {
            new Waypoint(0, 0, Pathfinder.d2r(differenceInHeading)),
            new Waypoint(offsetFromTargetInches, distanceFromTargetInches, Pathfinder.d2r(0)) 
        };

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_HIGH,
        timeStep, maxVel, maxAccel, maxJerk);

        Trajectory visionTrajectory = Pathfinder.generate(visionWaypoint, config);
        TankModifier testModifier = new TankModifier(visionTrajectory).modify(wheelBaseWidth);

        leftTrajectory = testModifier.getLeftTrajectory();
        rightTrajectory = testModifier.getLeftTrajectory();

        leftFollower = new EncoderFollower(leftTrajectory);
        rightFollower = new EncoderFollower(rightTrajectory);
    
        leftFollower.configureEncoder(Robot.drivebase.getLeftEncoder(), k_ticks_per_rev, k_wheel_diameter);
        rightFollower.configureEncoder(Robot.drivebase.getRightEncoder(), k_ticks_per_rev, k_wheel_diameter);

        leftFollower.configurePIDVA(0.15, 0, 0, 1 / k_max_velocity, 0);
        rightFollower.configurePIDVA(0.15, 0, 0, 1 / k_max_velocity, 0);

    }

    public void run() {
        if (isFirst) {
            timer.reset();
            timer.start();
            generateTrajectory();
            Robot.navX.zeroYaw();
            Robot.drivebase.zeroEncoder();
            System.out.println("TIME TO GENERATE: " +  timer.get());
            timer.stop();

            isFirst = false;
        }

        Robot.camera.getCameraDegreeOffset();

        double differenceInHeading = Robot.navX.getOriginalHeading() - desiredHeading;

        double distanceToCenterX = Robot.camera.getCentDist();
		double targetWidthPixels = Robot.camera.getWidth();
        double distanceFromTargetInches = 6040 / targetWidthPixels;
        
        //Recalculated distanceToCenterX
		double inchesOffset = distanceFromTargetInches * Math.tan(Math.toRadians(differenceInHeading));
		double pixelOffset = inchesOffset * (targetWidthPixels / WIDTH_OF_TARGET_INCHES);
        distanceToCenterX += pixelOffset;

        if (Math.abs(distanceToCenterX) <= 30) {
            System.out.println("STOP");
        } else {
            System.out.println("GO");
        }


        leftEncoder = Robot.drivebase.getLeftEncoder();
        rightEncoder = Robot.drivebase.getRightEncoder();

        double leftSpeed = leftFollower.calculate(Robot.drivebase.getLeftEncoder());
        double rightSpeed = rightFollower.calculate(Robot.drivebase.getRightEncoder());

        double heading = Robot.navX.getYaw();
        double desired_heading = Pathfinder.r2d(leftFollower.getHeading());
      
        double heading_difference = Pathfinder.boundHalfDegrees(desired_heading - heading);
        double turn =  turningConstant * heading_difference;
        
        Robot.drivebase.drive(leftSpeed + turn, rightSpeed - turn);
    
    }

    public boolean isFinished() {
        return (leftFollower.isFinished() || rightFollower.isFinished() || (Robot.camera.getWidth() >= stopWidth));
    }

}
