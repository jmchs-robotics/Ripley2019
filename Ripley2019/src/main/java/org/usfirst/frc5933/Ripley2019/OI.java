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
import edu.wpi.first.wpilibj.JoystickBase;
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

    public JoystickButton driverA;
    public JoystickButton driverBumperL;
    public JoystickButton driverBumperR;
    public JoystickButton driverX;
    public JoystickButton driverY;
    public JoystickButton driverB;
    public JoystickButton driverStart;
    public JoystickButton driverBack;
    public POVButton driveDPadUp;
    public POVButton driveDPadDown;
    public POVButton driveDPadUpLeft;
    public POVButton driveDPadUpRight;
    public POVButton driveDPadDownLeft;
    public POVButton driveDPadDownRight;

    public Joystick subsystemJoystick;

    public int angle;

    public JoystickButton subA;
    public JoystickButton subB;
    public JoystickButton subY;
    public JoystickButton subX;
    public JoystickButton subBumperL;
    public JoystickButton subBumperR;
    public JoystickButton subLTrigger;
    public JoystickButton subRTrigger;
    public JoystickButton subStart;
    public JoystickButton subBack;


    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    public OI() {
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS

        subsystemJoystick = new Joystick(1);
        
        driverJoystick = new Joystick(0);
 
        //Mashing Button enables vision proccessing to automatically drive towards RFT target.
        driverA = new JoystickButton(driverJoystick, 1);
        driverA.whenPressed(new DriveStraightVision(0.4, "Software team is Awesome", -1.16));
        driverA.whenReleased(new DefaultTeleopCommand());

        // Test: mashing Button drives an arc.
        driverX = new JoystickButton(driverJoystick, 3);
        // driverX.whenPressed(new DriveArc(-0.7, 47, 180, 1.1)); // trying 47" radius
        // driverX.whenPressed(new DriveArc(-0.7, 27, 180, 1.1)); // trying 27" radius
        //driverX.whenPressed(new Path2CG());
       driverX.whenPressed( new DriveArc( 0.7, 49, 67, -1.1));


        //driverX.whenReleased(new DefaultTeleopCommand());
        //driverX.whenPressed(new EndGameClimb());

        //for yeet arm
        driverBumperL = new JoystickButton(driverJoystick, 5);
        driverBumperL.whileHeld(new YeetArmBackward());

        driverBumperR = new JoystickButton(driverJoystick, 6);
        driverBumperR.whileHeld(new YeetArmForward());

        //For the CricketLegs
        driverY = new JoystickButton(driverJoystick, 4);
        driverY.whenPressed(new CricketLegsOut());

        driverB = new JoystickButton(driverJoystick, 2);
        driverB.whenPressed(new CricketLegsIn()); 

        //Test on Monday
        // subBumperR = new JoystickButton(subsystemJoystick, 6);
        // subBumperR.whenPressed(new HatchDropOff());
      
         subBumperL = new JoystickButton(subsystemJoystick, 5);
         subBumperL.whenPressed(new HatchPickUp());

        subBumperR = new JoystickButton(subsystemJoystick, 6);
        subBumperR.whenPressed(new HatchDropOff());

        //for arm positioning
        subA = new JoystickButton(subsystemJoystick, 1);
        subA.whenPressed(new MoveArmToRocketOne());
        subA.whenReleased(new EndArmEncoderPositioning());

        subB = new JoystickButton(subsystemJoystick, 2);
        subB.whenPressed(new MoveArmToRocketTwo());
        subB.whenReleased(new EndArmEncoderPositioning());

        subY = new JoystickButton(subsystemJoystick, 4);
        subY.whenPressed(new MoveArmToRocketThree());
        subY.whenReleased(new EndArmEncoderPositioning());
      
        //Cargo
        subLTrigger = new JoystickButton(subsystemJoystick, 9);
        subLTrigger.whileHeld(new CargoIn());

        subRTrigger = new JoystickButton(subsystemJoystick, 10);
        subRTrigger.whileHeld(new CargoOut());

        //Training Wheels
        subStart= new JoystickButton(subsystemJoystick, 11);
        subStart.whileHeld(new TrainingWheels());

      
        // //For Path2CG
         //driveDPadUp = new POVButton(subsystemJoystick, 0);
         //driveDPadUp.whenPressed(new Path2CG());

         //driveDPadUpLeft = new POVButton(subsystemJoystick, 315);
         //driveDPadUpLeft.whenPressed(new Path2CG());

         //driveDPadUpRight = new POVButton(subsystemJoystick, 45);
         //driveDPadUpRight.whenPressed(new Path2CG());

        // drivDPadDown = new POVButton(subsystemJoystick, 180);
        // drivDPadDown.whenPressed(new MoveArmDown());

        // drivDPadDownLeft = new POVButton(subsystemJoystick, 225);
        // drivDPadDownLeft.whenPressed(new MoveArmDown());

        // drivDPadDownRight = new POVButton(subsystemJoystick, 135);
        // drivDPadDownRight.whenPressed(new MoveArmDown());

      
        //For Wrist

        driverStart = new JoystickButton(driverJoystick, 8);
        driverStart.whenPressed(new WristOut());

        driverBack = new JoystickButton(driverJoystick, 7);
        driverBack.whenPressed(new WristIn());
      
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