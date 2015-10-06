package org.usfirst.frc.team5414.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
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
	DoubleSolenoid DoubleSolenoid1;
	DoubleSolenoid DoubleSolenoid2;

	public TeleopOne(SpeedController leftCon, SpeedController rightCon, RobotDrive drive, Encoder leftEnc,
			Encoder rightEnc, Joystick controller, JoystickButton LBumper, JoystickButton RBumper, JoystickButton a, JoystickButton b, JoystickButton x, JoystickButton y, 
			PowerDistributionPanel power, Gyro gyrator, CANTalon manipulating, DoubleSolenoid solenoid1, DoubleSolenoid solenoid2)
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
		DoubleSolenoid1 = solenoid1;
		DoubleSolenoid2 = solenoid2;
		aBtn = a; 
		bBtn = b;
		xBtn = x;
		yBtn = y;
	}
	@Override
	protected void initialize() {
		// TODO Auto-generated method stub

		leftEncoder.reset();
		rightEncoder.reset();
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
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

}
