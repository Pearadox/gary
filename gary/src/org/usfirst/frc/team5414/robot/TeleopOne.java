package org.usfirst.frc.team5414.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TeleopOne extends Command {
	
	SpeedController leftSide;
	SpeedController rightSide;
	RobotDrive Brodin;
	Encoder leftEncoder;
	Encoder rightEncoder;
	Joystick cont;
	JoystickButton LBump;
	JoystickButton RBump;
	JoystickButton aBtn;
	JoystickButton bBtn;
	JoystickButton xBtn;
	JoystickButton yBtn;
	PowerDistributionPanel powerDist;
	Gyro gyrate;
	CANTalon manipulate;


	public TeleopOne(SpeedController leftCon, SpeedController rightCon, RobotDrive drive, Encoder leftEnc,
			Encoder rightEnc, Joystick controller, JoystickButton LBumper, JoystickButton RBumper, JoystickButton a, JoystickButton b, JoystickButton x, JoystickButton y, 
			PowerDistributionPanel power, Gyro gyrator, CANTalon manipulating)
	{
		leftSide = leftCon;
		rightSide = rightCon;
		Brodin = drive;
		leftEncoder = leftEnc;
		rightEncoder = rightEnc;
		cont = controller;
		LBump = LBumper;
		RBump = RBumper;
		powerDist = power;
		gyrate = gyrator;
		manipulate = manipulating;
		aBtn = a; 
		bBtn = b;
		xBtn = x;
		yBtn = y;
		
		manipulate.reverseOutput(true);
		manipulate.reverseSensor(true);
		manipulate.enableLimitSwitch(true , true);
		manipulate.ConfigFwdLimitSwitchNormallyOpen(true);
		manipulate.ConfigRevLimitSwitchNormallyOpen(true);
		manipulate.enableControl();
		
	}
	@Override
	protected void initialize() {

		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	protected void execute() {
	    //drives the robot	
		Brodin.tankDrive(cont.getRawAxis(1), cont.getRawAxis(5));
		//puts #'s on the smart dashboard
		SmartDashboard.putNumber("front left current", powerDist.getCurrent(15));
		SmartDashboard.putNumber("front right current", powerDist.getCurrent(1));
		SmartDashboard.putNumber("back left current", powerDist.getCurrent(14));
		SmartDashboard.putNumber("back right current", powerDist.getCurrent(2));
	    SmartDashboard.putNumber("gyro rate", gyrate.getRate());
	    SmartDashboard.putNumber("gyro angle", gyrate.getAngle());
	    SmartDashboard.putNumber("left encoder counts", leftEncoder.get());
	    SmartDashboard.putNumber("right encoder counts", rightEncoder.get());
	    SmartDashboard.putNumber("left encoder distance", leftEncoder.getDistance());
	    SmartDashboard.putNumber("right encoder distance", rightEncoder.getDistance());
		//sets the manipulator to go down when the left bumper is pushed
	    if(LBump.get())
		{
			manipulate.set(-1);
		}
	    //sets the manipulator to go up when the right bumper is pushed
		else if(RBump.get())
		{
			manipulate.set(1);
		}
		//sets the manipulator to stop when nothing is pushed
		else
		{
			manipulate.set(0);
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

}
