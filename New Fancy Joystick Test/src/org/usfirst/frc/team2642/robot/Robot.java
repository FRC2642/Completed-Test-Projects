package org.usfirst.frc.team2642.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.interfaces.Potentiometer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.SolenoidBase;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.RobotDrive;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick stick;
	int autoLoopCounter;
	Potentiometer pot;
	Talon belt1;
	Talon belt2;
	Talon lift;
	DigitalInput limitswitch;
	Solenoid pushout1;
	Solenoid pushout2;
	Compressor compressor;
	Solenoid release;
	int dropcounter;
	RobotDrive myRobot;
	Gyro gyro;
	double Kp = .03;
    int crabcount = 0;
    double gyroset;
    Relay wrist;
	DoubleSolenoid arm;
	Solenoid hand;
	
	
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	myRobot = new RobotDrive(2, 3, 1, 0);
    	stick = new Joystick(0);
    	pot = new AnalogPotentiometer(0, 180, 0);
    	belt1 = new Talon(0);
    	belt2 = new Talon(1);
    	lift = new Talon(2);
    	lift = new Talon(3);
    	limitswitch = new DigitalInput(4);
    	pushout1 = new Solenoid(0);
    	pushout2 = new Solenoid(1);
    	release = new Solenoid(2);
    	compressor = new Compressor();
    	dropcounter = 0;
    	myRobot.setInvertedMotor(MotorType.kFrontLeft, true);
    	myRobot.setInvertedMotor(MotorType.kRearLeft, true);
    	gyro = new Gyro(0);

    	

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
		{
			
		}
    }
    
    /**
     * This function is called once each time the robot enters tele-operated mode
     */
    public void teleopInit(){
    	compressor.start();
    }

    /**
     * This function is called periodically during operator control
     */
    public void teleopPeriodic() {
    	if(stick.getRawButton(9)){	//Controls belts based on buttons 1 and 2
    		belt1.set(1);
    		belt2.set(-1);
    	}else if(stick.getRawButton(10)){
    		belt1.set(-1);
    		belt2.set(1);
    	}else{
    		belt1.set(0);
    		belt2.set(0);
    	}
    	boolean toggle = false;
    	
    	if(limitswitch.get() == true){
    		toggle = !toggle;
    	}else if(pot.get() >= 180){
    		toggle = !toggle;
    	}
    	if(stick.getRawButton(11)){
    		if(limitswitch.get() == true || toggle == true){	//Automatically controls lift based off of the limit switch and the potentiometer.
    			lift.set(1);
    		}else if(limitswitch.get() == false && toggle == false){
    			lift.set(-1);
    		}else{
    			lift.set(1);
    		}	
    	}else if(stick.getRawButton(11)){
    		if(stick.getRawButton(7)){	//Override
        		lift.set(1);
        	}else if(stick.getRawButton(8)){
        		lift.set(-1);
        	}else{
        		lift.set(0);
        	}
    	}
    	if(stick.getRawButton(10) && dropcounter <= 30){
    		release.set(false);
    		pushout1.set(false);
    		pushout2.set(false);
    		dropcounter ++;
    	}else if(dropcounter >= 31 && dropcounter <= 70){
    		release.set(false);
    		pushout1.set(true);
    		pushout2.set(true);
    		dropcounter ++;
    	}else if(!stick.getRawButton(10) && dropcounter >= 0){
    		release.set(false);
    		pushout1.set(true);
    		pushout2.set(true);
    		dropcounter = 0;
    	}else{	
    		release.set(true);
    		pushout1.set(false);
    		pushout2.set(false);
    	}
    	if(!stick.getRawButton(6)){
    		if (stick.getRawButton(1)){
    			myRobot.mecanumDrive_Cartesian(-stick.getX(), stick.getY(), -stick.getTwist(), gyro.getAngle());
    		}else if(stick.getRawButton(2) && crabcount <=1){
    			gyroset = gyro.getAngle();
    			crabcount++;
    		}else if(stick.getRawButton(2) && (crabcount >= 2)){
    			myRobot.mecanumDrive_Cartesian(-stick.getX()/2, stick.getY()/2, (gyro.getAngle() - gyroset) * Kp, 0);
    		}else if(crabcount >= 1 && !stick.getRawButton(2)){
    			crabcount = 0;
    		}else{
    			myRobot.mecanumDrive_Cartesian(-stick.getX()/2, stick.getY()/2, -stick.getTwist()/2, gyro.getAngle());}
    	}else{
    		myRobot.mecanumDrive_Cartesian(-stick.getX(), stick.getY(), -stick.getTwist(), 0.0);
    		}
    	if(stick.getY() > .25){							//Arm Control
    		arm.set(DoubleSolenoid.Value.kForward);
    	}else if(stick.getY() < .25){
    		arm.set(DoubleSolenoid.Value.kReverse);
    	}else{
    		arm.set(DoubleSolenoid.Value.kOff);
    	}
    	if(stick.getX() > .25){							//Wrist Control
    		wrist.set(Relay.Value.kForward);
    	}else if(stick.getX() < .25){
    		wrist.set(Relay.Value.kReverse);
    	}else{
    		wrist.set(Relay.Value.kOff);
    	}
    	if(stick.getRawButton(4)){						//Hand Control
    		hand.set(true);
    	}else{
    		hand.set(false);
    	}
    }
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    	LiveWindow.run();
    }
}
