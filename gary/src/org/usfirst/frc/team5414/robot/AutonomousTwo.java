package org.usfirst.frc.team5414.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousTwo extends Command{
	
	
	SpeedController leftSide;
	SpeedController rightSide;
	RobotDrive Brodin;
	Encoder leftEncoder;
	Encoder rightEncoder;
	CANTalon manipulate;
	double encleftinch = 23.7978;
	double encrightinch = 34.0308;
	int armMove;
	
	public AutonomousTwo(SpeedController leftCon, SpeedController rightCon, RobotDrive drive, Encoder leftEnc,
			Encoder rightEnc, CANTalon manipulating)
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
		
	}

	@Override
	protected void execute() {
		// TODO Auto-generated method stub
		SmartDashboard.putNumber("left encoder counts", leftEncoder.get());
	    SmartDashboard.putNumber("right encoder counts", rightEncoder.get());
	  //moves arm up to grab tote
	    if(armMove <=15)
	    {
	    	manipulate.set(1);
	    	armMove++;
	    }
	  //moves the robot backwards00 at 80% speed for 97 inches  
		if(leftEncoder.get()< (encleftinch*97) && rightEncoder.get()< (encrightinch*97) && armMove>=15)
		{
			Brodin.tankDrive(.8, .8);
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
