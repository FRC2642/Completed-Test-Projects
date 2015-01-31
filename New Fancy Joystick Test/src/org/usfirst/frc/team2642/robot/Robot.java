package org.usfirst.frc.team2642.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.Gyro;

public class Robot extends IterativeRobot {
	RobotDrive myRobot;
	Joystick stick;
	int autoLoopCounter;
	final int frontLeftChannel	= 2;
    final int rearLeftChannel	= 3;
    final int frontRightChannel	= 1;
    final int rearRightChannel	= 0;
    Gyro gyro;
    double Kp = .03;
    int crabcount = 0;
    double gyroset;
	
    
    public void robotInit() {
        myRobot = new RobotDrive(frontLeftChannel, rearLeftChannel, frontRightChannel, rearRightChannel);
    	stick = new Joystick(0);
    	myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
    	myRobot.setInvertedMotor(MotorType.kFrontRight, true);
    	gyro = new Gyro(0);
    }
    
    public void autonomousInit() {
    	autoLoopCounter = 0;
    }

    public void autonomousPeriodic() {
    	if(autoLoopCounter < 100) //Check if we've completed 100 loops (approximately 2 seconds)
		{
			myRobot.drive(-0.5, 0.0); 	// drive forwards half speed
			autoLoopCounter++;
			} else {
			myRobot.drive(0.0, 0.0); 	// stop robot
		}
    }
    
    public void teleopInit(){
    	
    }

    public void teleopPeriodic(){
    	if(!stick.getRawButton(11)){
    		if (stick.getRawButton(1)){
    		myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), gyro.getAngle());
    		}else if(stick.getRawButton(2) && crabcount <=1){
    		gyroset = gyro.getAngle();
    		crabcount++;
    		}else if(stick.getRawButton(2) && (crabcount >= 2)){
    		myRobot.mecanumDrive_Cartesian(stick.getX()/2, stick.getY()/2, (gyro.getAngle() - gyroset) * Kp, gyro.getAngle()/2);
    		}else if(crabcount >= 1 && !stick.getRawButton(2)){
    		crabcount = 0;
    		}else {
    		myRobot.mecanumDrive_Cartesian(stick.getX() /2, stick.getY() /2, stick.getTwist() /2, gyro.getAngle());}
    	}else{
    		myRobot.mecanumDrive_Cartesian(stick.getX(), stick.getY(), stick.getTwist(), 0.0);

    	}
    }
    
    
    public void testPeriodic() {
    	LiveWindow.run();
    }
    
}
