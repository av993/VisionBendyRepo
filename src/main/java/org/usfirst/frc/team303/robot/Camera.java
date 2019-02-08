package org.usfirst.frc.team303.robot;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Camera {

	static final double pixelPerDegreeConstant = 0.146875;
	static final double offsetConstant = 30;
	
	public double getCameraDegreeOffset() {
		int[][] visionArr = getVisionContours(0, Robot.getStringArr());

		double centerX = (visionArr[0][1] + visionArr[1][0]) /2;
		double width = visionArr[1][0] - visionArr[0][1];
		double height1 = visionArr[0][4] - visionArr[0][5];
		double height2 = visionArr[1][4] - visionArr[1][5];

		System.out.println("Center X" + centerX);
		System.out.println("Width" + width);
		SmartDashboard.putNumber("Width", width);
		SmartDashboard.putNumber("Center X", centerX);

		double centerXIdeal = 320;
		double centerXCurrent = centerX+offsetConstant;
		double centerXOffset = centerXIdeal-centerXCurrent;
	
		double distanceY = 6040 / width;
		double distanceX = centerXOffset * (8 / width);
		//System.out.println(distanceX + "   &&   " + distanceY + "   &&   " + ( Math.atan(distanceX / distanceY)) );

		return -Math.toDegrees(Math.atan(distanceX / distanceY));
	}



	public int[][] getVisionContours(int position, String[] inputArr) {	
		int[][] visionArr = convtVisionArr(inputArr);
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
		 return new int[][] {{-1,-1,-1,-1},{-1,-1,-1,-1}};
	}
		
	// needs to test deciding the distance between ul and ur is less than ll,lr --> get a set of contours that are valid
	public boolean isValidCont(int[] cont1, int[] cont2){
		int ul1 = cont1[0], ur1 = cont1[1], ll1 = cont1[2], lr1 = cont1[3];
		int ul2 = cont2[0], ur2 = cont2[1], ll2 = cont2[2], lr2 = cont2[3];  

		int topDiff = ur1 - ul2;
		int botDiff = lr1 - ll2;

		return topDiff < botDiff; // quiets the compiler
	}
}


/*abstract
import java.util.ArrayList;
import java.util.Collections;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Camera {

	public Object imgLock = new Object();
	private Thread visionThread;
	private HatchPipeline pipeline;
	private boolean runProcessing = false;
	private double centerXOne = 0.0;
	private double centerYOne = 0.0;
	private double centerXTwo = 0.0;
	private double centerYTwo = 0.0;
	private double centerXAvg = 0.0;
	private double centerYAvg = 0.0;
	private double rectangleArea=0.0;
	public static final int cameraResX = 320;
	public static final int cameraResY = 240;
	
	public Camera() {
		enableVisionThread(); //outputs a processed feed to the dashboard (overlays the found boiler tape)
	}

	public void enableVisionThread() {
		pipeline = new HatchPipeline();
		AxisCamera camera = CameraServer.getInstance().addAxisCamera("10.3.3.8");
		camera.setResolution(cameraResX, cameraResY);

		CvSink cvSink = CameraServer.getInstance().getVideo(); //capture mats from camera
		CvSource outputStream = CameraServer.getInstance().putVideo("Stream", cameraResX, cameraResY); //send steam to CameraServer
		Mat mat = new Mat(); //define mat in order to reuse it

		runProcessing = true;

		visionThread = new Thread(() -> {

			while(!Thread.interrupted()) { //this should only be false when thread is disabled

				if(cvSink.grabFrame(mat)==0) { //fill mat with image from camera)
					outputStream.notifyError(cvSink.getError()); //send an error instead of the mat
					SmartDashboard.putString("Vision State", "Acquisition Error");
					continue; //skip to the next iteration of the thread
				}

				if(runProcessing) {		

					pipeline.process(mat); //process the mat (this does not change the mat, and has an internal output to pipeline)
					int contoursFound = pipeline.filterContoursOutput().size();
					SmartDashboard.putString("More Vision State","Saw "+contoursFound+" Contours");

					if(contoursFound>=2) {
						//test comment

						//get the contours from the vision algorithm
						Rect rectOne = Imgproc.boundingRect(pipeline.filterContoursOutput().get(0));
						Rect rectTwo = Imgproc.boundingRect(pipeline.filterContoursOutput().get(1));
					
						
						//sort the rectangles horizontally
						Rect rectLeft = (rectOne.x<rectTwo.x) ? rectOne : rectTwo;
						Rect rectRight = (rectOne.x>rectTwo.x) ? rectOne : rectTwo;		
						rectOne = rectRight;
						rectTwo = rectLeft;
						
						//calculate center X and center Y pixels
						centerXOne = rectOne.x + (rectOne.width/2); //returns the center of the bounding rectangle
						centerYOne = rectOne.y + (rectOne.height/2); //returns the center of the bounding rectangle
						centerXTwo = rectTwo.x + (rectTwo.width/2);
						centerYTwo = rectTwo.y + (rectTwo.height/2);
						
						double width=rectTwo.x-(rectOne.x+rectOne.width);
						double height=rectOne.y-(rectTwo.y+rectTwo.height);

						rectangleArea=width*height;
						centerYAvg = (centerYOne + centerYTwo)/2;
						centerXAvg = (centerXOne + centerXTwo)/2;
		
						//draws the rectangles onto the camera image sent to the dashboard
						Imgproc.rectangle(mat, new Point(rectOne.x, rectOne.y), new Point(rectTwo.x + rectTwo.width, rectTwo.y + rectTwo.height), new Scalar(0, 0, 255), 2); 
						Imgproc.rectangle(mat, new Point(centerXAvg-3,centerYAvg-3), new Point(centerXAvg+3,centerYAvg+3), new Scalar(255, 0, 0), 3);

						SmartDashboard.putString("Vision State", "Executed overlay!");
					}

					SmartDashboard.putNumber("Center X", centerXAvg);
					outputStream.putFrame(mat); //give stream (and CameraServer) a new frame
				} else {
					outputStream.putFrame(mat); //give stream (and CameraServer) a new frame
				}

				//Timer.delay(0.09);
			}

		});	
		visionThread.setDaemon(true);
		visionThread.start();
	}
	
	public double getArea(){
		return rectangleArea;
	}

	public double getCenterY() {
		return centerYAvg;
	}

	public double getCenterX() {
		return centerXAvg;
	}

	public void disableProcessing() {
		runProcessing = false;
	}

	public void enableProcessing() {
		runProcessing = true;
	}

}*/