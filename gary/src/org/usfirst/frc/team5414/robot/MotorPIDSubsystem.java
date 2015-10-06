package org.usfirst.frc.team5414.robot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.command.PIDSubsystem;

public class MotorPIDSubsystem extends PIDSubsystem {
	
	SpeedController Motor;
	Encoder encoder;
	
	
	public MotorPIDSubsystem(String name, SpeedController motor,
			Encoder enc) 
	{
		super(name, .5, 0.0, 0.0);
		
		Motor = motor;
		encoder = enc;
		
	}
	
	public void initDefaultCommand()
	{
		
	}
	
	protected double returnPIDInput()
	{
		return encoder.getRate();
	}
	
	protected void usePIDOutput(double output) 
	{
		
		Motor.pidWrite(output);
	}

	
	
		
	

}
