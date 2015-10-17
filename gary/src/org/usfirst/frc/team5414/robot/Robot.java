package org.usfirst.frc.team5414.robot;




import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.AnalogTrigger;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.livewindow.LiveWindowSendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	Command autonomousCommand;
	Command demoCommand;
	SendableChooser autoChooser;
	SendableChooser demochooser;
	int autoLoopCounter;
	JoystickButton aBtn;
	JoystickButton bBtn;
	JoystickButton xBtn;
	JoystickButton yBtn;
	JoystickButton LB;
	JoystickButton RB;
	JoystickButton select;
	JoystickButton start;
	JoystickButton LS;
	JoystickButton RS; 	
	SpeedController driveLeft;
	SpeedController driveRight;
	Encoder encoderleft;
	Encoder encoderright;
	AutonomousOne auton1; 
	AutonomousTwo auton2;
	TeleopOne standardMode;
	TeleopTwo demoMode;
	double offset;
	int autocounter;
	double encoderleftinch;
	double encoderrightinch;
	AnalogPotentiometer potentiometer;
	PowerDistributionPanel pdp;
	DigitalInput limitSwitchUpper;
	DigitalInput limitSwitchLower;
	CANTalon manipulator;
	Gyro gyro;
	AnalogInput analogin;
	DigitalInput modeSwitch;
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		
		autoChooser = new SendableChooser();
		demochooser = new SendableChooser();
		driveLeft = new Talon(0);
		driveRight = new Talon(1);
		manipulator = new CANTalon(2);
		myRobot = new RobotDrive(driveLeft, driveRight);
		stick = new Joystick(0);
		pdp = new PowerDistributionPanel();
		potentiometer  = new AnalogPotentiometer(1,2);
		encoderleft = new Encoder(0, 1, false, EncodingType.k4X);
		encoderright = new Encoder(2, 3, true, EncodingType.k4X);
		encoderleft.setDistancePerPulse(1);
		encoderright.setDistancePerPulse(1);
		encoderleftinch= 23.7978;
		encoderrightinch= 34.0308;
		autoChooser = new SendableChooser();
		demochooser = new SendableChooser();
		aBtn= new JoystickButton(stick, 1);
		bBtn = new JoystickButton(stick, 2);
		xBtn = new JoystickButton(stick, 3);
		yBtn = new JoystickButton(stick, 4);
		LB = new JoystickButton(stick, 5);
		RB = new JoystickButton(stick, 6);
		select = new JoystickButton(stick, 7);
		start = new JoystickButton(stick, 8);
		LS = new JoystickButton(stick, 9);
		RS = new JoystickButton(stick, 10);
		limitSwitchUpper = new DigitalInput(8);
		limitSwitchLower = new DigitalInput(9);
		offset = 57*8;
		autocounter = 0;
		gyro = new Gyro(0);
		gyro.setSensitivity(.007);
   	    gyro.initGyro();
   	    gyro.reset();
   	    auton1 = new AutonomousOne(driveLeft, driveRight, myRobot, encoderleft, encoderright);
		auton2 = new AutonomousTwo(driveLeft, driveRight, myRobot, encoderleft, encoderright, manipulator);
		standardMode = new TeleopOne(driveLeft, driveRight, myRobot, encoderleft, encoderright, stick, LB, RB, aBtn, bBtn, xBtn, yBtn, pdp, gyro, manipulator);
		demoMode = new TeleopTwo(driveLeft, driveRight, myRobot, encoderleft, encoderright, stick, LB, RB, aBtn, bBtn, xBtn, yBtn, pdp, gyro, manipulator);
		modeSwitch = new DigitalInput(6);
	}
	
	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
	
		gyro.reset();
		autoChooser.addDefault("Default program", auton1);
		autoChooser.addObject("Experimental auto", auton2);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
		autonomousCommand = (Command) autoChooser.getSelected();
		autonomousCommand.start();
	
	}
	
	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
	
		Scheduler.getInstance().run();
	
	}
	
	/**
	 * This function is called once each time the robot enters tele-operated mode
	 */
	public void teleopInit(){
		
		Scheduler.getInstance().removeAll();
		if (modeSwitch.get())
		{
			demochooser.addDefault("boring drive", demoMode);
			demochooser.addObject("OVERHYPER DRIVE BETA", standardMode);
		}
		else
		{
			demochooser.addDefault("OVERHYPER DRIVE BETA", standardMode);
			demochooser.addObject("boring drive", demoMode);
		}
		//demochooser.addDefault("boring drive", demoMode);
		//demochooser.addObject("OVERHYPER DRIVE BETA", standardMode);
		SmartDashboard.putData("Drive Mode Chooser", demochooser);
		demoCommand =  (Command)demochooser.getSelected();
		demoCommand.start();
		gyro.reset();
	
	}
	
	/**
	 *
	 * This function is called periodically during operator control
	 */
	public void teleopPeriodic() {
	
		Scheduler.getInstance().run();
	
	}
	
	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
	
		LiveWindow.run();
	
	}
}