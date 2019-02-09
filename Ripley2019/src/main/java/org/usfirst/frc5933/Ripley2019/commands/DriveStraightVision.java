package org.usfirst.frc5933.Ripley2019.commands;

import org.usfirst.frc5933.Ripley2019.Robot;
//import org.usfirst.frc5933.Ripley2019.SocketVisionSender;
import org.usfirst.frc5933.Ripley2019.subsystems.DriveTrain;
import org.usfirst.frc5933.Ripley2019.subsystems.RoboRio;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 *
 */
public class DriveStraightVision extends Command {

	double initHeading;
	double initDistance;
	double vBus;
	String vision;
	double threshold;
	double worstYAccel;
	double agy;
	final double kP = 1;
	// 	public boolean isActive = false;

	/**
	 * Instantiate a command to drive the robot to a set target
	 * @param vbus
	 * Speed to drive forward
	 * @param visionType
	 * The {@link SocketVisionSender} constant string to use
	 * @param threshold
	 * The allowable acceleration to end the command.
	 */
	public DriveStraightVision(double vbus, String visionType, double threshold) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain); 

		this.vision = visionType;
		this.vBus = vbus;
		this.threshold = threshold;
	}

	// Called just before this Command runs the first time
	protected void initialize() {
		//Robot.sender_.setSendData(vision);

		initHeading = Robot.driveTrain.getGyroHeading();
		agy = RoboRio.accelerometer.getY();

		/** if(vision.equalsIgnoreCase(SocketVisionSender.StartRFT)) {
			initDistance = Robot.rft_.get_distance() * DriveTrain.kEncoderTicksPerInch;
		}	
		if(vision.equalsIgnoreCase(SocketVisionSender.PlatformBlueSearch) || vision.equalsIgnoreCase(SocketVisionSender.PlatformRedSearch)) {
			initDistance = Robot.platform_.get_distance();
		} */
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {
		/* if (!isActive){
			return;
		} */

		double error = -1;
		double proportion = 0;
		double coefficient = 1;

		if(agy <= 0)
		{
			worstYAccel = agy;
		}
		SmartDashboard.putNumber("WorstYAccel", worstYAccel);

		// tracking to the RFT target
		error = Robot.rft_.get_degrees_x();

		/* 
		// 2018 Eve: Tracking RFT or the edge of the platform
		if(vision.equalsIgnoreCase(SocketVisionSender.StartRFT)) { error = Robot.rft_.get_degrees_x(); }
		if(vision.equalsIgnoreCase(SocketVisionSender.PlatformBlueSearch) || 
				vision.equalsIgnoreCase(SocketVisionSender.PlatformRedSearch)) { error = Robot.platform_.get_degrees_x(); }
		*/
		// if the vision isn't finding anything, drive straight based on gyro reading
		if(error == -1) {
			proportion = DriveTrain.kPGyroConstant * (Robot.driveTrain.getGyroHeading() - initHeading);

		} else { 
			proportion = error * kP;
			//drive vision
			if(error == 0) initHeading = Robot.driveTrain.getGyroHeading();
		}

		coefficient = 1;
		/* 
		// amplify the correction with the distance to the target
		coefficient = (initDistance - Robot.driveTrain.getRightEncoderPos(0)) / initDistance;
		coefficient = Robot.driveTrain.thresholdVBus(coefficient);
		*/

		
		//Robot.driveTrain.tankDrive(coefficient * (vBus - proportion), -coefficient * (vBus + proportion));
		Robot.driveTrain.tankDrive(coefficient * (vBus - proportion), -coefficient * (vBus + proportion));

	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		// bail when we run into the target... or anything else
		return Robot.roboRio.getYAccelerationComparedToThreshold(threshold, true); 
		//|| initDistance - Robot.driveTrain.getRightEncoderPos(0) < 18 * DriveTrain.kEncoderTicksPerInch; 
	}

	// Called once after isFinished returns true
	protected void end() {
		Robot.driveTrain.tankDrive(0, 0);
	}

	// Called when another command which requires one or more of the same
	// subsystems is scheduled to run
	protected void interrupted() {
		end();
	}
}
