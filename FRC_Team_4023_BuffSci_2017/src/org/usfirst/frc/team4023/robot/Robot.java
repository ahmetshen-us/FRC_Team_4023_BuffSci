package org.usfirst.frc.team4023.robot;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.AxisCamera;
import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot = new RobotDrive(0, 1);
	Joystick stick = new Joystick(0);
	// Joystick secondstick = new Joystick(1);
	SpeedController armmotor = new Talon(5);
	Thread visionThread;

	Timer timer = new Timer();

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		// CameraServer.getInstance().startAutomaticCapture();

		// Advanced camera server program
		// In the following example a thread created in robotInit() gets the
		// Camera Server instance. ¬†Each frame of the video is individually
		// processed, in this case converting a color image (BGR) to gray scale
		// using the OpenCV cvtColor() method. The resultant images are then
		// passed to the output stream and sent to the dashboard. You can
		// replace the cvtColor operation with any image processing code that is
		// necessary for your application. You can even annotate the image using
		// OpenCV methods to write targeting information onto the image being
		// sent to the dashboard.

		// new Thread(() -> {
		// UsbCamera camera = CameraServer.getInstance()
		// .startAutomaticCapture();
		// camera.setResolution(640, 480);
		//
		// CvSink cvSink = CameraServer.getInstance().getVideo();
		// CvSource outputStream = CameraServer.getInstance().putVideo("Blur",
		// 640, 480);
		//
		// Mat source = new Mat();
		// Mat output = new Mat();
		//
		// while (!Thread.interrupted()) {
		// cvSink.grabFrame(source);
		// Imgproc.cvtColor(source, output, Imgproc.COLOR_BGR2GRAY);
		// outputStream.putFrame(output);
		// }
		// }).start();

		// CameraServer server = CameraServer.getInstance();
		// server.startAutomaticCapture("cam0", 0);
		// CameraServer server1 = CameraServer.getInstance();
		// server1.startAutomaticCapture("cam1", 1);

		visionThread = new Thread(() -> {
			// Get the Axis camera from CameraServer
			AxisCamera camera = CameraServer.getInstance()
					.addAxisCamera("axis-camera.local");
			// Set the resolution
			camera.setResolution(640, 480);

			// Get a CvSink. This will capture Mats from the camera
			CvSink cvSink = CameraServer.getInstance().getVideo();
			// Setup a CvSource. This will send images back to the Dashboard
			CvSource outputStream = CameraServer.getInstance()
					.putVideo("Rectangle", 640, 480);

			// Mats are very memory expensive. Lets reuse this Mat.
			Mat mat = new Mat();

			// This cannot be 'true'. The program will never exit if it is. This
			// lets the robot stop this thread when restarting robot code or
			// deploying.
			while (!Thread.interrupted()) {
				// Tell the CvSink to grab a frame from the camera and put it
				// in the source mat. If there is an error notify the output.
				if (cvSink.grabFrame(mat) == 0) {
					// Send the output the error.
					outputStream.notifyError(cvSink.getError());
					// skip the rest of the current iteration
					continue;
				}
				// Put a rectangle on the image
				Imgproc.rectangle(mat, new Point(100, 100), new Point(400, 400),
						new Scalar(255, 255, 255), 5);
				// Give the output stream a new image to display
				outputStream.putFrame(mat);
			}
		});
		visionThread.setDaemon(true);
		visionThread.start();
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	@Override
	public void autonomousInit() {
		timer.reset();
		timer.start();
	}

	/**
	 * This function is called periodically during autonomous
	 */

	@Override
	public void autonomousPeriodic() {
		double driveForXSeconds = 5.0;
		double driveSpeed = -(0.5);
		double timerSecondsPassed = timer.get();
		// set curveAmount to 0.0 to got straight left is > 0 and right is < 0
		double curveAmountFirst = 0.0;
		double curveAmountSecond = 0.0;

		if (timerSecondsPassed < driveForXSeconds) {
			myRobot.drive(driveSpeed, curveAmountFirst);
		}

		else if (timerSecondsPassed < (driveForXSeconds + 2.5)) {
			myRobot.drive(driveSpeed, curveAmountSecond);
		}

		else {
			myRobot.drive(0.0, 0.0); // stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated
	 * mode
	 */
	@Override
	public void teleopInit() {
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		double xaxisToRotate = -stick.getX();// rotateValue
		double yaxisToMove = stick.getY(); // moveValue

		myRobot.arcadeDrive(yaxisToMove, xaxisToRotate);

		// armmotor.set(secondstick.getY());

	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
		LiveWindow.run();
	}
}
