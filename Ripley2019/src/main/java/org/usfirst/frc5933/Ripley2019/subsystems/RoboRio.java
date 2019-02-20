// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5933.Ripley2019.subsystems;


import org.usfirst.frc5933.Ripley2019.commands.*;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS
import edu.wpi.first.wpilibj.AnalogAccelerometer;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jdk.jfr.Threshold;
import edu.wpi.first.wpilibj.DigitalInput;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=IMPORTS


/**
 *
 */
public class RoboRio extends Subsystem {

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTANTS

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    private AnalogGyro gyro;
    public static BuiltInAccelerometer accelerometer;
    public static DigitalInput[] DIPs;
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public RoboRio() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        gyro = new AnalogGyro(0);
        addChild("Gyro",gyro);
        gyro.setSensitivity(0.007);
        
        accelerometer = new BuiltInAccelerometer();
        addChild("Accelerometer",accelerometer);
        

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

         
    DIPs = new DigitalInput[10];
        
    //create each individual dip.
    for(int i = 0; i < DIPs.length; i++) {
        DIPs[i] = new DigitalInput(i);   
    }

    
    }

    @Override
    public void initDefaultCommand() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DEFAULT_COMMAND

        // Set the default command for a subsystem here.
        // setDefaultCommand(new MySpecialCommand());
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CMDPIDGETTERS

    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
	 * Read all digital inputs (0-9) and return an integer representation of their state, summed from their binary inputs.
	 * @return
	 */
	public int readDips() {
		int val = 0;

		for(int i = 0; i < DIPs.length; i++) {
			if(!DIPs[i].get()) { //invert because of pull-up resistors.
				val += (int) Math.pow(2, i);
			}
		}

		return val;
    }
    
    /**
	 * Read the Digital Inputs between ports beginIndex and endIndex (exclusive). The largest port is 25, so endIndex must be less than 26.
	 * @param beginIndex
	 * The start point to read dips from. Can range from 0 to 24, inclusive. Must always be smaller than endIndex.
	 * @param endIndex
	 * The index to stop reading dips from. Can range from 1 to 26, inclusive. Must always be larger than beginIndex.
	 * @return
	 */
	public int readDips(int beginIndex, int endIndex, boolean startPowersAtZero) {
		int val = 0;
		
		for(int i = beginIndex; i < endIndex; i ++) {
			if(!DIPs[i].get()) { //invert because of pull-up resistors
				if(startPowersAtZero) {
					val += Math.pow(2, i - beginIndex);
				} else {
					val += Math.pow(2, i);
				}
			}
		}
		return val;
	}

    public boolean getYAccelerationComparedToThreshold(double threshold, boolean accelerationOver) {
        // return (accelerometer.getY() >= threshold) && accelerationOver;
        if (threshold < 0)
        {
            return threshold > accelerometer.getY() && accelerationOver;
        }else{
            return threshold < accelerometer.getY() && accelerationOver;
        }
	}
	
	public double getYAccel() {
		return accelerometer.getY();
	}
	
	@Override
	public void periodic() {
        SmartDashboard.putNumber("Y accel: ", getYAccel());
        SmartDashboard.putBoolean("DIPs 0: ", DIPs[0].get());
    }

    
}

