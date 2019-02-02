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

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc5933.Ripley2019.commands.*;
import org.usfirst.frc5933.Ripley2019.subsystems.*;

import org.usfirst.frc5933.Ripley2019.SocketVision;
import org.usfirst.frc5933.Ripley2019.SocketVisionSender;
import org.usfirst.frc5933.Ripley2019.RobotMap;

import edu.wpi.first.cameraserver.*;
import edu.wpi.cscore.UsbCamera;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in 
 * the project.
 */
public class Robot extends TimedRobot {

    	//Socket-based receivers. One is needed for each port to read from
	public static SocketVision rft_;		//5801
	public static SocketVision platform_;	//5804

	//socket sender. One is needed per IP to send to
	public static SocketVisionSender sender_;

	//Socket constants
	public static final boolean show_debug_vision = false;

    Command autonomousCommand;
    SendableChooser<Command> chooser = new SendableChooser<>();

    public static OI oi;
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS
    public static DriveTrain driveTrain;
    public static CricketLegs cricketLegs;
    public static Cargo cargo;
    public static Hatch hatch;
    public static Wrist wrist;
    public static Arm arm;
    public static RoboRio roboRio;
	public static YeetArm yeetArm;
	
	//the camera server and camera objects
	public static CameraServer server;
	UsbCamera cam0, cam1;

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=DECLARATIONS

    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {

        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        driveTrain = new DriveTrain();
        cricketLegs = new CricketLegs();
        cargo = new Cargo();
        hatch = new Hatch();
        wrist = new Wrist();
        arm = new Arm();
        roboRio = new RoboRio();
        yeetArm = new YeetArm();

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTORS
        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();

        // Add commands to Autonomous Sendable Chooser
        // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS

        chooser.setDefaultOption("Autonomous Command", new AutonomousCommand());

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=AUTONOMOUS
        SmartDashboard.putData("Auto mode", chooser);

        RobotMap.init();
		driveTrain.init();
		
		cam0 = server.startAutomaticCapture("FrontLeft",0);
		cam1 = server.startAutomaticCapture("FrontRight",1);
		cam0.setExposureAuto();
		cam1.setExposureAuto();
		cam0.setBrightness(10);
		cam1.setBrightness(10);
		cam0.setFPS(15);
		cam1.setFPS(15);
		cam1.setResolution(160, 120);
		cam0.setResolution(160, 120);
    }

    	/** 
	 * This method properly shuts down all coprocessor related objects and joins them to the main thread
	 * to comply with FRC guidelines during disabled mode. DONT CHANGE A WORD!
	 */
	private void visionShutDown() {
		if(platform_ != null) {
			try {
				platform_.stoprunning();
				platform_.join();
				platform_ = null;
			} catch(Exception e) {
				e.printStackTrace();
			}
		}

		if (rft_ != null) {
			try {
				rft_.stoprunning();
				rft_.join();
				rft_ = null;
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
/*
		if(sender_ != null) {
			try {
				sender_.stoprunning();
				sender_.join();
				sender_ = null;
			}catch (Exception e) {
				e.printStackTrace();
			}
		}
*/

	}

    	/**
	 * This method properly instantiates and initializes all coprocessor related objects. This needs to be called before
	 * any of these objects could be used -- so don't use them during disabled mode. This should be called during autonomous
	 * and teleop init methods. For ease of access, these objects are global and instantiated through the main class.
	 */
	private void visionInit() {
		if (platform_ == null) {
			platform_ = new SocketVision("10.59.33.255", 5804);
			if (show_debug_vision) {
				System.out.println("Vision to platform started.");
			}
			platform_.start();

			if (!platform_.is_connected()) {
				if (!platform_.connect()) {
					if (show_debug_vision) {
						System.err.println("Failed to connect to the Helmsman and I really need my boiled mayonnaise");
					}
				} else {
					if (show_debug_vision) {
						System.out.println("Connected. Love me some boiled mayo.");
					}
				}
			}
		}

		if (rft_ == null) {
			rft_ = new SocketVision("10.59.33.255", 5801);
			if (show_debug_vision) {
				System.out.println("Vision to RFT started.");
			}
			rft_.start();

			if (!rft_.is_connected()) {
				if (!rft_.connect()) {
					if (show_debug_vision) {
						System.err.println("Failed to connect to the Helmsman and I really need my mayonnaise");
					}
				} else {
					if (show_debug_vision) {
						System.out.println("Connected. Love that mayo.");
					}
				}
			}
		}
/*
		if(sender_ == null) {
			sender_ = new SocketVisionSender("10.59.33.255", 5800);
			if(show_debug_vision) {
				System.out.println("Sender started");
			}

			sender_.start();
			if(!sender_.is_connected()) {
				if(!sender_.connect()) {
					if(show_debug_vision) {
						System.err.println("Failed to instantiate Sender... I really need to tell the helmsman to get me my mayo!");
					}
				}else {
					if(show_debug_vision) {
						System.out.println("Connected! Mayo shipments, incoming!!");
					}
				}
			}
        }
        */
	}


    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    @Override
    public void disabledInit(){
		visionShutDown();
    }

    @Override
    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void autonomousInit() {
        autonomousCommand = chooser.getSelected();
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();

        visionInit();
    }

    /**
     * This function is called periodically during autonomous
     */
    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();

        visionInit();

    }

    /**
     * This function is called periodically during operator control
     */
    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
    }
}
