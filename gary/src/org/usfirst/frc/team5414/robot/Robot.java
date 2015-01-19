package org.usfirst.frc.team5414.robot;




import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	Joystick joystick1;
	Joystick joystick2;
	Trigger trigger1; 

	int autoLoopCounter;
	JoystickButton aBtn;
	JoystickButton bBtn;
	JoystickButton xBtn;
	JoystickButton yBtn;
	JoystickButton LB;
	JoystickButton RB;
	Compressor myCompressor;	
	DoubleSolenoid DoubleSolenoid1;
	SpeedController driveRightFrontOrange;
	SpeedController driveLeftRearWhite;
	SpeedController driveRightRearBlue;
	SpeedController driveLeftFrontBrown;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		driveLeftFrontBrown = new Talon(4);
		//LiveWindow.addActuator("Drive", "LeftFrontBrown", (Talon) driveLeftFrontBrown);
	        
	        driveLeftRearWhite = new Talon(1);
		//LiveWindow.addActuator("Drive", "LeftRearWhite", (Talon) driveLeftRearWhite);
	        
	        driveRightRearBlue = new Talon(3);
	    //LiveWindow.addActuator("Drive", "RightRearBlue", (Talon) driveRightRearBlue);
	        
	        driveRightFrontOrange = new Talon(2);
		//LiveWindow.addActuator("Drive", "RightFrontOrange", (Talon) driveRightFrontOrange);
	        
		myRobot = new RobotDrive(driveLeftFrontBrown,driveLeftRearWhite,driveRightFrontOrange,driveRightRearBlue);
		stick = new Joystick(0);
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);
		
		
		aBtn= new JoystickButton(stick, 1);
		bBtn = new JoystickButton(stick, 2);
		xBtn = new JoystickButton(stick, 3);
		yBtn = new JoystickButton(stick, 4);
		LB = new JoystickButton(stick, 5);
		RB = new JoystickButton(stick, 6);

		myCompressor = new Compressor(0);
		DoubleSolenoid1 = new DoubleSolenoid(0,1);
		/*CameraServer server = CameraServer.getInstance();
		server.setQuality(50);
		server.startAutomaticCapture("cam0");
		CameraServer server2 = CameraServer.getInstance();
		server2.setQuality(50);
		server2.startAutomaticCapture("cam1");
		*/
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-.75, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
		} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
	}

	/**
	 * This function is called once each time the robot enters tele-operated mode
	 */
	public void teleopInit(){
	//	myCompressor.setClosedLoopControl(true);
		DoubleSolenoid1 = new DoubleSolenoid(0,1);
	}

	/**
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
		myRobot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
		//myRobot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
		//c.start();
		//c.setClosedLoopControl(true);

		//    	boolean enabled = c.enabled();
		//    	boolean pressureSwitch = c.getPressureSwitchValue();
		//    	float current = c.getCompressorCurrent();
		//    	DoubleSolenoid1.set(DoubleSolenoid.Value.kOff);
		if(LB.get())
		{
//			System.out.println("Solenoid forward");
			DoubleSolenoid1.set(DoubleSolenoid.Value.kForward);
		}
		else if(RB.get())
		{
			DoubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
//			System.out.println("Solenoid reverse");
		}
		else
		{
			DoubleSolenoid1.set(DoubleSolenoid.Value.kOff);
			// Prints here are not a good idea because they flood the console
		}

	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
		LiveWindow.run();
	}

}
