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
public class DriveArc extends Command {

	// set to true for encoder-based (isFinished() gone far enough) end, set to false for gyro-based end
	final boolean endBasedOnEncoder = false;

	// If encoder is on the left, use +1. Eve's is on the R, so -1.  If Ripley's is on the R, use -1
	int encoderS = -1; 

	final double kP = 175; // 0.00100; // P coefficient in PID. Identify something that works for the robot.
	final double halfWheelWidth = 13.0; // 11 for Eve, 10.5 for Ripley.  empirically chose 13 on 2/24 for Eve
	final double encoderTicksPerInch = 4096 / ( 2 * Math.PI * 2); // 4" dia (2" radius) wheel

	// every encoder click on the encoded wheel should equate to some amount of gyro change, 
	//   based on the radius of the desired arc path.  The encoder tick is 'forced' by the vBus;
	//   the gyro responds better/worse based on outside forces. So measure the gyro change, 
	//   compare it to the desired change, and use that difference to further influence the inner
	//   wheel
	double degPerTick; // degrees per encoder tick the robot should turn along this arc
	
	double initEncoder;
	double lastEnc;
	double heading; // where we're pointed, computed by adding normalized change of each new gyro reading to last heading
	double lastHeading; // keep the gyro reading from one tick (execute()) to the next
	double targetHeading; // where we want to end pointing. Not normalized.
	double distR; // distance the right wheel of the robot will travel to complete the desired arc path
	double distL; // distance the left wheel of the robot will travel to complete the desired arc path
	double vBusR; // baseline voltage for the right wheel
	double vBusL; // baseline voltage for the left wheel
	double threshold; // stop if we run into anything

	
	/**
	 * Instantiate a command to drive the robot along an arc
	 * @param vbus
	 * Speed of the outer wheel. Positive to drive forward, negative to drive backwards.
	 * @param centerRadius
	 * The radius of the arc defined by the center of the robot 
	 * @param degrees
	 * The degrees the robot will turn along the desired arc. Negative to go left, positive to go right.
	 * @param threshold
	 * The allowable acceleration (negative when stopping from a forward drive, positive to stop from a backwards drive) to end the command.
	 */
	public DriveArc(double vbus, double centerRadius, double degrees, double threshold) {
		// Use requires() here to declare subsystem dependencies
		// eg. requires(chassis);
		requires(Robot.driveTrain); 

		this.threshold = threshold;
		this.distL = encoderTicksPerInch * Math.abs( degrees * Math.PI / 180 * (centerRadius + Math.signum( degrees) * halfWheelWidth));
		this.distR = encoderTicksPerInch * Math.abs( degrees * Math.PI / 180 * (centerRadius - Math.signum( degrees) * halfWheelWidth));
		SmartDashboard.putNumber("distR", distR);

		double c = Math.abs( centerRadius);
		if( degrees < 0) {
			this.vBusR = vbus;
			this.vBusL = vbus * (c - halfWheelWidth) / (c + halfWheelWidth); // slower so we turn L
		}
		else {
			this.vBusL = vbus;
			this.vBusR = vbus * (c - halfWheelWidth) / (c + halfWheelWidth); // slower so we turn R
		}
		// robot should turn this much for every encoder tick; magnitude should decrease with larger radius
		//   negative for CCW travel (forward leftward or backwards rightward), positive for CW travel
		if( encoderS < 0) {
			this.degPerTick = degrees / distL; 
		} else {
			this.degPerTick = degrees / distR;
		}
		this.degPerTick *= Math.signum( degrees * vbus);

		this.targetHeading = degrees;
	}

	// Called just before this Command runs the first time
	protected void initialize() {

		lastHeading = Robot.driveTrain.getGyroHeading();
		targetHeading += lastHeading;
		heading = lastHeading;
		

		SmartDashboard.putNumber("initial lastHeading", lastHeading);
		SmartDashboard.putNumber("initial R encoder", Robot.driveTrain.getRightEncoderPos(0));

		initEncoder = getRelevantEncoder();
	}

	// read the encoder actually on the robot, as defined by encoderS
	protected double getRelevantEncoder() {
		if( encoderS > 0) { // Left encoder, Eve
			return Robot.driveTrain.getLeftEncoderPos(0);
		} else {
			return Robot.driveTrain.getRightEncoderPos(0);
		}
	}

	// Called repeatedly when this Command is scheduled to run
	protected void execute() {

		// get encoder value and subtract previous encoder val
		double enc = getRelevantEncoder();
		double denc = enc - lastEnc;

		// get heading and subtract previous heading
		// normalezed to -179 to +180:
		//    if turning CCW and get to 358 then 359 then 0 (delta 1 then -359 which % 360 = 1 is good)
		//    or 1 then 0 then 359 (delta -1 then 359 so if > 180 take out - 360)
		double h = Robot.driveTrain.getGyroHeading();
		double dh = ( h - lastHeading) % 360;
		if( dh > 180) { dh -= 360; }

		heading += dh; // accumulate our heading, so it doesn't get normalized

		// error is defference between desired degrees change per encoder tick 
		//    and measured deg change / encoder change
		double e = degPerTick - dh / denc;
		double proportion = kP * e;

		if( degPerTick < 0) { // turning left, lag the L side
			Robot.driveTrain.tankDrive((vBusL - proportion), -vBusR);
		}
		else { // turning right, lag the R side
			Robot.driveTrain.tankDrive( vBusL, -( vBusR - proportion));
		}
		lastHeading = h;
		lastEnc = enc;
	}

	// Make this return true when this Command no longer needs to run execute()
	protected boolean isFinished() {
		SmartDashboard.putNumber("current R encoder", Robot.driveTrain.getRightEncoderPos(0));
		boolean goneFarEnough;

		if( endBasedOnEncoder) {
			// return true (bail) if we've gone far enough
			if( encoderS > 0) { // left encoder
				goneFarEnough =	Math.signum(vBusL) * (Robot.driveTrain.getLeftEncoderPos(0) - initEncoder)
					> distL;
			} else {
				goneFarEnough = Math.signum(vBusR) * (Robot.driveTrain.getRightEncoderPos(0) - initEncoder) 
				> distR;
			}
		} else {
			// multiply both sides by targetHeading in case targetHeading is negative
			if( heading * targetHeading > targetHeading * targetHeading) {  
				goneFarEnough = true;
			}
		}
		
		// bail if we've gone far enough or if we run into anything 
		return goneFarEnough || Robot.roboRio.getYAccelerationComparedToThreshold(threshold, true); 
		
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