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
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.PIDSubsystem;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
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
	Joystick joystick1;
	Joystick joystick2;
	Trigger trigger1; 
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
	Compressor myCompressor;	
	DoubleSolenoid DoubleSolenoid1;
	DoubleSolenoid DoubleSolenoid2;
	SpeedController driveRightFrontOrange;
	SpeedController driveLeftRearWhite;
	SpeedController driveRightRearBlue;
	SpeedController driveLeftFrontBrown;
	//SpeedController driveLeft;
	//SpeedController driveRight;
	Encoder encoderleft;
	Encoder encoderright;
	Command auton1;
	Command auton2;
	Command standardMode;
	Command demoMode;
	CameraServer server;
	int oldrawcount;
	double offset;
	AnalogPotentiometer potentiometer;
	PowerDistributionPanel pdp;
	DigitalInput limitSwitchUpper;
	DigitalInput limitSwitchLower;
	CANTalon manipulator;
	Gyro gyro;
	AnalogInput analogin;
//	PIDController pidRightfront;
//	PIDController pidLeftfront;
//	PIDController pidLeftrear;
//	PIDController pidRightrear;
	MotorPIDSubsystem leftSub;
	MotorPIDSubsystem rightSub;

	
	
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	public void robotInit() {
		
		driveLeftFrontBrown = new Talon(4);

		driveLeftRearWhite = new Talon(1);

		driveRightRearBlue = new Talon(3);

		driveRightFrontOrange = new Talon(2);

		//driveLeft = new Talon(0);
		
		//driveRight = new Talon(1);
		
		manipulator = new CANTalon(2);
		//myRobot = new RobotDrive(driveLeft, driveRight);
		myRobot = new RobotDrive(driveLeftFrontBrown,driveLeftRearWhite,driveRightFrontOrange,driveRightRearBlue);
		stick = new Joystick(0);
		oldrawcount = 0;
		pdp = new PowerDistributionPanel();
		potentiometer  = new AnalogPotentiometer(1,2);
		joystick1 = new Joystick(1);
		joystick2 = new Joystick(2);
		encoderleft = new Encoder(0, 1, true, EncodingType.k4X);
		encoderright = new Encoder(2, 3, true, EncodingType.k4X);
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
		myCompressor = new Compressor(0);
		limitSwitchUpper = new DigitalInput(8);
		limitSwitchLower = new DigitalInput(9);
		DoubleSolenoid1 = new DoubleSolenoid(0,1);
		DoubleSolenoid2 = new DoubleSolenoid(2,3);
		offset = 57*8;
		gyro = new Gyro(0);
		gyro.setSensitivity(.007);
   	    gyro.initGyro();
   	    gyro.reset();
   	    double velocityleft = 0.0;
   	    double velocityright = 0.0;
   	    final double pterm = 1;
   	    final double iterm = 0;
   	    final double dterm = 0;
   	    final double ffterm = 0;
//   	    pidRightfront = new PIDController(pterm, iterm, dterm, ffterm, encoderright, driveRightFrontOrange);
//   	    pidRightrear = new PIDController(pterm, iterm, dterm, ffterm, encoderright, driveRightRearBlue);
//   	    pidLeftfront = new PIDController(pterm, iterm, dterm, ffterm, encoderright, driveLeftFrontBrown);
//   	    pidLeftrear = new PIDController(pterm, iterm, dterm, ffterm, encoderright, driveLeftRearWhite);
   	   
   	    leftSub = new MotorPIDSubsystem( driveLeftFrontBrown, driveLeftRearWhite,encoderleft, "left");
   	    leftSub = new MotorPIDSubsystem(driveRightFrontOrange, driveRightRearBlue, encoderright, "right");
   	    //analogin = new AnalogInput(0);
		
		//server = CameraServer.getInstance();
		//server.setSize(1);
		//server.setQuality(50);
		//server.startAutomaticCapture("cam0");
		auton1=new Command()
		{

			@Override
			protected void initialize() {
				// TODO Auto-generated method stub
				
			}

			@Override
			protected void execute() {
				// TODO Auto-generated method stub
//			
				
				if (oldrawcount > 100000)
				{
					myRobot.drive(-.75, 0.0); 	// drive forwards half speed
					System.out.println(oldrawcount);
					oldrawcount = encoderleft.getRaw();
				}
			else
				{
					myRobot.drive(0.0, 0.0);
				}
			}

			
			
			@Override
			protected boolean isFinished() {
				// TODO Auto-generated method stub
				return false;
			}

			@Override
			protected void end() {
				// TODO Auto-generated method stub
				
			}

			@Override
			protected void interrupted() {
				// TODO Auto-generated method stub
				
			}
		};
		auton2=new Command()
		{

			@Override
			protected void initialize() {
				// TODO Auto-generated method stub
				
			}

			@Override
			protected void execute() {
				// TODO Auto-generated method stub
				if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
				{
					encoderleft.setDistancePerPulse(.5);
					myRobot.drive(-.5, 0.5); 	// drive forwards half speed
					autoLoopCounter++;
				} else {
					myRobot.drive(0.0, 0.0); 	// stop robot
				}
			}

			@Override
			protected boolean isFinished() {
				// TODO Auto-generated method stub
				return false;
			}

			@Override
			protected void end() {
				// TODO Auto-generated method stub
				
			}

			@Override
			protected void interrupted() {
				// TODO Auto-generated method stub
				
			}
		};
		standardMode = new Command()
				{

					@Override
					protected void initialize() {
						System.out.println("standard");
						// TODO Auto-generated method stub
						
					}

					@Override
					protected void execute() {
						// TODO Auto-generated method stub
					
						
						
							//myRobot.tankDrive(stick.getRawAxis(1), stick.getRawAxis(5));
							/*
							pidLeftfront.setSetpoint(0);
							pidLeftrear.setSetpoint(0);
							pidRightfront.setSetpoint(0);
							pidRightrear.setSetpoint(0);
						*/
//						pterm = pterm;
//							pidLeftfront.setSetpoint(stick.getRawAxis(1));
//							pidLeftrear.setSetpoint(stick.getRawAxis(1));
//							pidRightfront.setSetpoint(stick.getRawAxis(5));
//							pidRightrear.setSetpoint(stick.getRawAxis(5));
//							
							leftSub.setSetpoint(stick.getRawAxis(1));
							rightSub.setSetpoint(stick.getRawAxis(5));
							/*
							if(!manipulator.isFwdLimitSwitchClosed())
							{
								System.out.println("forward switch pushedd");
							}
							if(!manipulator.isRevLimitSwitchClosed())
							{
								System.out.println("back switch pushedd");
							}
							*/
						
						SmartDashboard.putNumber("front left current", pdp.getCurrent(15));
						SmartDashboard.putNumber("front right current", pdp.getCurrent(1));
						SmartDashboard.putNumber("back left current", pdp.getCurrent(14));
						SmartDashboard.putNumber("back right current", pdp.getCurrent(2));
						//SmartDashboard.putNumber("analog volts", analogin.getVoltage());
						SmartDashboard.putNumber("string pot", manipulator.getAnalogInRaw());
					    SmartDashboard.putNumber("gyro rate", gyro.getRate());
					    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
					    SmartDashboard.putNumber("left encoder counts", encoderleft.getRate());
					    SmartDashboard.putNumber("right encoder counts", encoderright.getRate());
						int newcount;
						 newcount = encoderleft.getRaw();
						 if (newcount != oldrawcount)
						 {
							 int diff = newcount - oldrawcount;
							 System.out.println(diff);
							 oldrawcount = newcount;
						 }
						if(LB.get())
						{
						 manipulator.set(1);
						 System.out.println("a buttton got");
						}
						else if(RB.get())
						{
							manipulator.set(-1);
							System.out.println("b buttttton got");
						}
						else
						{
							manipulator.set(0);
						}
					    //manipulator.enableLimitSwitch(true , true);
						//manipulator.ConfigFwdLimitSwitchNormallyOpen(false);
						//manipulator.ConfigRevLimitSwitchNormallyOpen(false);
						//manipulator.enableForwardSoftLimit(true);
						//manipulator.enableReverseSoftLimit(true);
						/*
						 if(bBtn.get())
						{
						 manipulator.set(798-offset);
						}
						
						if(xBtn.get())
						{
						 manipulator.set(627-offset);
						}
						
						if(aBtn.get())
						{
							manipulator.set(1282.5-offset);
						}
						
						if(yBtn.get())
						{
							manipulator.set(1510.5-offset);
						}
						*/
						
						
						if(aBtn.get())
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kForward);
						}
						else if(bBtn.get())
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
						}
						if(xBtn.get())
						{
							DoubleSolenoid2.set(DoubleSolenoid.Value.kForward);
						}
						else if(yBtn.get())
						{
							DoubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
						}
						else
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kOff);
							DoubleSolenoid2.set(DoubleSolenoid.Value.kOff);
							// Prints here are not a good idea because they flood the console
						}
				
						
					}

					@Override
					protected boolean isFinished() {
						// TODO Auto-generated method stub
						return false;
					}

					@Override
					protected void end() {
						// TODO Auto-generated method stub
						
					}

					@Override
					protected void interrupted() {
						// TODO Auto-generated method stub
						
					}
			
				};
				demoMode = new Command()
				{

					@Override
					protected void initialize() {
						System.out.println("demo");
						// TODO Auto-generated method stub
						
					}

					@Override
					protected void execute() {
						// TODO Auto-generated method stub
						myRobot.tankDrive((stick.getRawAxis(1)*.75), (stick.getRawAxis(5)*.75));
						SmartDashboard.putNumber("front left current", pdp.getCurrent(15));
						SmartDashboard.putNumber("front right current", pdp.getCurrent(1));
						SmartDashboard.putNumber("back left current", pdp.getCurrent(14));
						SmartDashboard.putNumber("back right current", pdp.getCurrent(2));
						//SmartDashboard.putNumber("analog volts", analogin.getVoltage());
						SmartDashboard.putNumber("string pot", manipulator.getAnalogInRaw());
					    SmartDashboard.putNumber("gyro rate", gyro.getRate());
					    SmartDashboard.putNumber("gyro angle", gyro.getAngle());
					    SmartDashboard.putNumber("left encoder counts", encoderleft.getRate());
					    SmartDashboard.putNumber("right encoder counts", encoderright.getRate());
						int newcount;
						 newcount = encoderleft.getRaw();
						 if (newcount != oldrawcount)
						 {
							 int diff = newcount - oldrawcount;
							 System.out.println(diff);
							 oldrawcount = newcount;
						 }
						if(LB.get())
						{
						 manipulator.set(1);
						 System.out.println("a buttton got");
						}
						else if(RB.get())
						{
							manipulator.set(-1);
							System.out.println("b buttttton got");
						}
						else
						{
							manipulator.set(0);
						}
						
						/*
						 if(bBtn.get())
						{
						 manipulator.set(798-offset);
						}
						
						if(xBtn.get())
						{
						 manipulator.set(627-offset);
						}
						
						if(aBtn.get())
						{
							manipulator.set(1282.5-offset);
						}
						
						if(yBtn.get())
						{
							manipulator.set(1510.5-offset);
						}
						*/
						
						
						if(aBtn.get())
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kForward);
						}
						else if(bBtn.get())
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kReverse);
						}
						if(xBtn.get())
						{
							DoubleSolenoid2.set(DoubleSolenoid.Value.kForward);
						}
						else if(yBtn.get())
						{
							DoubleSolenoid2.set(DoubleSolenoid.Value.kReverse);
						}
						else
						{
							DoubleSolenoid1.set(DoubleSolenoid.Value.kOff);
							DoubleSolenoid2.set(DoubleSolenoid.Value.kOff);
							// Prints here are not a good idea because they flood the console
						}
				
					}

					@Override
					protected boolean isFinished() {
						// TODO Auto-generated method stub
						return false;
					}

					@Override
					protected void end() {
						// TODO Auto-generated method stub
						
					}

					@Override
					protected void interrupted() {
						// TODO Auto-generated method stub
						
					}
					
				};
		
		demochooser.addDefault("boring drive", demoMode);
		demochooser.addObject("OVERHYPER DRIVE BETA", standardMode);
		autoChooser.addDefault("Default program", auton1);
		autoChooser.addObject("Experimental auto", auton2);
		SmartDashboard.putData("Autonomous Mode Chooser", autoChooser);
		SmartDashboard.putData("Drive Mode Chooser", demochooser);
		
	}

	/**
	 * This function is run once each time the robot enters autonomous mode
	 */
	public void autonomousInit() {
		autoLoopCounter = 0;
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
		System.out.println("teleinit");
		demoCommand = (Command) demochooser.getSelected();
		demoCommand.start();
//		pidLeftfront.enable();
//		pidLeftrear.enable();
//		pidRightfront.enable();
//		pidRightrear.enable();
		
//		manipulator.changeControlMode(CANTalon.ControlMode.PercentVbus);
//		manipulator.setFeedbackDevice(CANTalon.FeedbackDevice.AnalogPot);
//		manipulator.setPID(1, 0, 0);
//		manipulator.enableControl();
		manipulator.reverseOutput(true);
		manipulator.reverseSensor(true);
		manipulator.enableLimitSwitch(true , true);
		manipulator.ConfigFwdLimitSwitchNormallyOpen(false);
		manipulator.ConfigRevLimitSwitchNormallyOpen(false);
		
//		manipulator.enableForwardSoftLimit(true);
		//manipulator.enableReverseSoftLimit(true);

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
