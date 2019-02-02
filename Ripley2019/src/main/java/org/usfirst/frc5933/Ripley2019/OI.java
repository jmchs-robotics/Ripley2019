// RobotBuilder Version: 2.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package org.usfirst.frc5933.Ripley2019;

import org.usfirst.frc5933.Ripley2019.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.POVButton;
import edu.wpi.first.wpilibj.buttons.Trigger;

import org.usfirst.frc5933.Ripley2019.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    //// CREATING BUTTONS
    // One type of button is a joystick button which is any button on a joystick.
    // You create one by telling it which joystick it's on and which button
    // number it is.
    // Joystick stick = new Joystick(port);
    // Button button = new JoystickButton(stick, buttonNumber);

    // There are a few additional built in buttons you can use. Additionally,
    // by subclassing Button you can create custom triggers and bind those to
    // commands the same as any other Button.

    //// TRIGGERING COMMANDS WITH BUTTONS
    // Once you have a button, it's trivial to bind it to a button in one of
    // three ways:

    // Start the command when the button is pressed and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenPressed(new ExampleCommand());

    // Run the command while the button is being held down and interrupt it once
    // the button is released.
    // button.whileHeld(new ExampleCommand());

    // Start the command when the button is released  and let it run the command
    // until it is finished as determined by it's isFinished method.
    // button.whenReleased(new ExampleCommand());


    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public Joystick driverJoystick;

    public Joystick subsystemJoystick;

    public int angle;

    public JoystickButton subA;
    public JoystickButton subB;
    public JoystickButton subY;
    public JoystickButton subX;
    public JoystickButton subBumperL;
    public JoystickButton subBumperR;
    public JoystickButton subLStick;
    public JoystickButton subRStick;
    public JoystickButton subStart;
    public JoystickButton subBack;
    public POVButton subDPadUp;
    public POVButton subDPadDown;
    public POVButton subDPadUpLeft;
    public POVButton subDPadUpRight;
    public POVButton subDPadDownLeft;
    public POVButton subDPadDownRight;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        subsystemJoystick = new Joystick(1);
        
        driverJoystick = new Joystick(0);

        //For the CricketLegs
        subA = new JoystickButton(subsystemJoystick, 1);
        subA.whenPressed(new CricketLegsOut());

        subB = new JoystickButton(subsystemJoystick, 2);
        subB.whenPressed(new CricketLegsIn()); 

        //For the Hatch Panel
        subBumperL = new JoystickButton(subsystemJoystick, 5);
        subBumperL.whenPressed(new HatchPickUp());

        subBumperR = new JoystickButton(subsystemJoystick, 6);
        subBumperR.whenPressed(new HatchDropOff());


        //Using the numbers for a and b fot test
        subLStick = new JoystickButton(subsystemJoystick, 9);
        subLStick.whileHeld(new CargoIn());

        subRStick = new JoystickButton(subsystemJoystick, 10);
        subRStick.whileHeld(new CargoOut());

        //For Shoulder
        subY = new JoystickButton(subsystemJoystick, 4);
        subY.whileHeld(new MoveArmUp());

        subX = new JoystickButton(subsystemJoystick, 3);
        subX.whileHeld(new MoveArmDown());


        subDPadUp = new POVButton(subsystemJoystick, 90);
        subDPadUp.whenPressed(new MoveArmUp());

        subDPadUpLeft = new POVButton(subsystemJoystick, 135);
        subDPadUpLeft.whenPressed(new MoveArmUp());

        subDPadUpRight = new POVButton(subsystemJoystick, 45);
        subDPadUpRight.whenPressed(new MoveArmUp());

        subDPadDown = new POVButton(subsystemJoystick, 270);
        subDPadDown.whenPressed(new MoveArmDown());

        subDPadDownLeft = new POVButton(subsystemJoystick, 225);
        subDPadDownLeft.whenPressed(new MoveArmDown());

        subDPadDownRight = new POVButton(subsystemJoystick, 315);
        subDPadDownRight.whenPressed(new MoveArmDown());

        //For Wrist

        subStart = new JoystickButton(subsystemJoystick, 7);
        subStart.whenPressed(new WristIn());

        subBack = new JoystickButton(subsystemJoystick, 8);
        subBack.whenPressed(new WristOut());


        // SmartDashboard Buttons
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("NullCommand", new NullCommand());
        SmartDashboard.putData("DefaultTeleopCommand", new DefaultTeleopCommand());
        SmartDashboard.putNumber("POV angle:", subsystemJoystick.getPOV());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
    }

    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
    public Joystick getDriverJoystick() {
        return driverJoystick;
    }

    public Joystick getSubsystemJoystick() {
        return subsystemJoystick;
    }


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=FUNCTIONS
}

