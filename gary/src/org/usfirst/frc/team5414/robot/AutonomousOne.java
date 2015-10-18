package org.usfirst.frc.team5414.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousOne extends Command {
	
	SpeedController leftSide;
	SpeedController rightSide;
	RobotDrive Brodin;
	Encoder leftEncoder;
	Encoder rightEncoder;
	double encleftinch = 23.7978;
	double encrightinch = 34.0308;
	
	
	public AutonomousOne(SpeedController leftCon, SpeedController rightCon, RobotDrive drive, Encoder leftEnc,
			Encoder rightEnc)
	{
		leftSide = leftCon;
		rightSide = rightCon;
		Brodin = drive;
		leftEncoder = leftEnc;
		rightEncoder = rightEnc;
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
		SmartDashboard.putNumber("left encoder counts", leftEncoder.get());
	    SmartDashboard.putNumber("right encoder counts", rightEncoder.get());
	  //moves the robot forward at 80% speed for 145 inches  
		if(leftEncoder.get()< ((encleftinch*225)) && rightEncoder.get()< ((encrightinch*225)))
		{
			Brodin.tankDrive(-.6, -.6);
		}
	    //stops the robot
		else
		{
			Brodin.tankDrive(0, 0);
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
