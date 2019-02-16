package org.usfirst.frc.team303.robot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Camera {

	static final double pixelPerDegreeConstant = 0.146875;
	static final double offsetConstant = 30;
	static final double FIELD_OF_VIEW_RAD = 70.42 * Math.PI /180.0;
	static final double FOCAL_LENGTH_PIXELS = (640 / 2) / Math.tan(FIELD_OF_VIEW_RAD / 2.0);
	public int width = 0;
	public double distance = 0;
	public double distToCenter = 0;

	public static final double A = -1.029258435778039;
	public static final double B = 0.12131357724670491;
	public static final double C = -0.0004412063928563716;
	public static final double D = 7.653889638753478e-7;
	
	public double getCameraDegreeOffset() {

		int[][] visionArr = getVisionContours(0, Robot.getStringArr());

		if (visionArr.length == 2) {

			double centerX = (visionArr[0][1] + visionArr[1][0]) /2;
			centerX = centerX + offsetConstant;
			width = visionArr[1][0] - visionArr[0][1];
			distToCenter = centerX - 320;

			double angle = Math.toDegrees(Math.atan(distToCenter / FOCAL_LENGTH_PIXELS));
			
			return angle;

		}

		return 0;
	}

	public void test() {

		getCameraDegreeOffset();

		double distanceToCenter = getCentDist();
		double targetWidth = getWidth();

		double distanceFromTargetInches = 4400 / targetWidth;
		double differenceInHeading = Robot.navX.getYaw();
		
		SmartDashboard.putNumber("Distance To Center: ", distanceToCenter);

		double inchesOffset = distanceFromTargetInches * Math.tan(Math.toRadians(differenceInHeading));
		double pixelOffset = inchesOffset * (targetWidth / 8);
		distanceToCenter += pixelOffset;

		double offsetFromTargetInches = (distanceToCenter * 8) / width;

		SmartDashboard.putNumber("Y Distance: ", distanceFromTargetInches);
		SmartDashboard.putNumber("X Distance: ", offsetFromTargetInches);

		System.out.println("Y Distance: " + distanceFromTargetInches);
		System.out.println("X Distance: " + offsetFromTargetInches);
		
		//System.out.println("Width: " + targetWidth);
		SmartDashboard.putNumber("Pixel Offset: ", pixelOffset);



	}


	public double getCentDist() {
		return distToCenter;
	}

	public double getWidth() {
		return width;
	}

	public double getDistance() {

		return distance;
	}

	public int[][] getVisionContours(int position, String[] inputArr) {	
		
		ArrayList<String> nextList = new ArrayList<>();

		for (String cont : inputArr) {
			if (!cont.isBlank()) {
				nextList.add(cont);
			}
		}

		String[] nextArr = new String[nextList.size()];
		for (int i = 0; i < nextList.size(); i++) {
			//System.out.println(nextList.get(i));
			nextArr[i] = nextList.get(i);
		}

		int[][] visionArr = convtVisionArr(nextArr);
		visionArr = sortArr(visionArr);
		
		return calculate(visionArr, position);
	}
	
	public int[][] convtVisionArr(String[] arr){
		int[][] finalArr = new int[arr.length][6];
	
		int outerIndex = 0;

		for (String conts : arr){
			int innerIndex = 0;
			for (String val : conts.split(" ")){
				int tempVal = Integer.parseInt(val);
				finalArr[outerIndex][innerIndex] = tempVal; 
				innerIndex++;
			}
			
			outerIndex++;
		}
		
		return finalArr;
	}

	public int[][] sortArr(int[][] arr){
		
		Arrays.sort(arr, new Comparator<int[]>() {
	        public int compare(int[] entry1, int[] entry2) {
	            final int val1 = entry1[0];
	            final int val2 = entry2[0];
	            
	            if (val1 < val2) {
	            	return -1;
	            }
	            else {
	            	return 1;
	            }
	        }
	    });

		return arr; // quiets the compiler
	}

	public int[][] calculate(int[][] arr, int position) {

		for (int i = 0; i < arr.length - 1; i++) {
			System.out.println(isValidCont(arr[i], arr[i+1]));

		 if (isValidCont(arr[i], arr[i+1])){
			if (position == 0 || position == 1) {
				return new int[][] {arr[i], arr[i + 1]};										
			} 
			else if (position == 2) {
				if (i >= 2) {
					return new int[][] {arr[i], arr[i + 1]};
				}	
			} else if (position == 3) {
				if (i >= 3) {
					return new int[][] {arr[i], arr[i + 1]};
				}	
			}
		 }
		}
		 return new int[][] {};
	}

	//FOR TESTING
	public void printOverturn() {
		double overturn = 0.0;
		double originalPower = 0.6;
		double distanceToCenterX = Robot.camera.getCentDist();
		double targetWidthPixels = Robot.camera.getWidth();
		double distanceFromTargetInches = 6040 / targetWidthPixels;
		double scaledPower = 0.6;
		double offsetFromTargetInches = (distanceToCenterX * 8) / targetWidthPixels;
		
		double cameraOffset = Math.toDegrees(Math.atan(offsetFromTargetInches / distanceFromTargetInches));
		//double cameraOffset = Math.toDegrees(Math.atan(distanceCent / FOCAL_LENGTH_PIXELS));
		double originalHeading = Robot.navX.getOriginalHeading();

		SmartDashboard.putNumber("Initial Degree:", cameraOffset);
		System.out.println("Initial Degree: " + cameraOffset);

		//Gives you the pixel offset if you are at an angle
		double differenceInHeading = 0 - originalHeading;
		double inchesOffset = distanceFromTargetInches * Math.tan(Math.toRadians(differenceInHeading));
		double pixelOffset = inchesOffset * (targetWidthPixels / 8);
		distanceToCenterX += pixelOffset;
		scaledPower = originalPower;

		double X = Math.abs(distToCenter + pixelOffset);
		overturn = A + (B * X) + (C * Math.pow(X, 2)) + (D * Math.pow(X,3));
		Math.copySign(overturn, -(distanceToCenterX + pixelOffset));

		cameraOffset = cameraOffset + overturn;
		SmartDashboard.putNumber("Overturn:", overturn);
		SmartDashboard.putNumber("Pixel Offset:", pixelOffset);
		SmartDashboard.putNumber("Final Degree:", cameraOffset);
		SmartDashboard.putNumber("Heading Difference:", differenceInHeading);
	} 

	// needs to test deciding the distance between ul and ur is less than ll,lr --> get a set of contours that are valid
	public boolean isValidCont(int[] cont1, int[] cont2){
		int ul1 = cont1[0], ur1 = cont1[1], ll1 = cont1[2], lr1 = cont1[3];
		int ul2 = cont2[0], ur2 = cont2[1], ll2 = cont2[2], lr2 = cont2[3];  

		int topDiff = ul2 - ur1;
		int botDiff = ll2 - lr1;
		//System.out.println("TOP DIFF: " + topDiff);
		//System.out.println("BOTTOM DIFF: " + botDiff);


		return topDiff < botDiff; // quiets the compiler
	}
}