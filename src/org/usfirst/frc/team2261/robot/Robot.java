/*----------------------------------------------------------------------------*/
/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team2261.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Jaguar;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Ultrasonic.Unit;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.vision.AxisCamera;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Spark;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SampleRobot {

    static double UPDATE_DELAY = 0.0075; // How often to refresh
    static double MOTOR_EXPIRATION = UPDATE_DELAY * 4;
    
    // IMPORTANT!!!!!!
    // Set to true for driving forklift from accesory y-axis (when not at limit or being driven by accesory buttons)
    static boolean MAP_YAXIS_TO_FORKLIFT = true;
    
    // Values for forklift motor (determined by testing)
    static double FORKLIFT_NOMINAL_UP = 0.2;
    static double FORKLIFT_ALOT_UP = 0.2;
    static double FORKLIFT_FULL_LIFT = 0.75;
    static double FORKLIFT_NOMINAL_DOWN = -0.5;
    
    // Create a new RobotDrive, composed of the following Jaguar motor controllers
	Talon rightController = new Talon(2);
	Talon leftController = new Talon(3);
	Talon rightController2= new Talon(1);
	Talon leftController2= new Talon(4);
	Spark clawController= new Spark (5);
	Spark upanddownController= new Spark (6);
	Spark kickerController= new Spark (7);
	RobotDrive robotDrive = new RobotDrive(leftController, leftController2, rightController, rightController2);
	/*Talon frontRightController = new Talon(2);
	Talon rearRightController = new Talon(1);
	Talon frontLeftController = new Talon(4);
	Talon rearLeftController = new Talon(3);
    RobotDrive robotDrive = new RobotDrive(frontLeftController, rearLeftController,
                                            frontRightController, rearRightController);
    */
    // Create new Joystick on ports 1 and 2
    Joystick driverJoystick = new Joystick(0);
    Joystick accessoryJoystick = new Joystick(1);
    
    // Forklift related
   /* Jaguar forkliftMotorController = new Jaguar(5); 
    Encoder forkliftEncoder = new Encoder(0, 1);
    DigitalInput forkliftLowerSwitch = new DigitalInput(2);
    DigitalInput forkliftUpperSwitch = new DigitalInput(3);
    DigitalInput pickupItemContactSwitch = new DigitalInput(4);*/
    
    boolean robotInitted = false;
    boolean useForklift = false;
    
    protected void robotInit() {
        // (Re-)Enable motor safety with .25s expiration
        // "Motor safety is a mechanism built into the RobotDrive object that
        // will turn off the motors if the program doesn't continuously update
        // the motor speed." (default "expiration" is 100ms)
        robotDrive.setExpiration(Robot.MOTOR_EXPIRATION);  
        robotDrive.setSafetyEnabled(true);
        
       // forkliftMotorController.setExpiration(Robot.MOTOR_EXPIRATION);
        //forkliftMotorController.setSafetyEnabled(true);
        
        // Reverse left side drive
        //robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);
        
        // Ensure all motors are stopped (I don't believe we should have to do this)
        robotDrive.drive(0, 0);
        
        //forkliftEncoder.setDistancePerPulse(distancePerPulse);

        robotInitted = true;
    }

    public void autonomous() {
    	// Driver forward at half speed for 3 seconds
//        robotDrive.drive(0.25, 0);   
//        Timer.delay(3);
//        robotDrive.drive(0, 0);
    }
    
    
    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        robotDrive.setExpiration(MOTOR_EXPIRATION);  
      
        if(!robotInitted) {
          System.out.println("Robot not initted? Initting...");
          robotInit();
        }
        
        
        
        // While still under operator control and enabled, "arcade drive" robot,
        // updating motors every 0.0075 second
        while(isOperatorControl() && isEnabled()) {
//        	if(!sonar.isRangeValid()) {
//        		sonar.ping();
//        	}
          
            // Drive the Robot
            robotDrive.arcadeDrive(scaleJoystickInput(driverJoystick.getX())/2,
                                   scaleJoystickInput(driverJoystick.getY())/2);
            
            // Forklift Operation
          /*  if(isForkliftAtLowerLimit()) { // Trying to go down and not at lower limit
            	forkliftMotorController.set(Robot.FORKLIFT_NOMINAL_UP);
            } else if(isForkliftAtUpperLimit()) { // While trying to go up and not at the upper limit
            	forkliftMotorController.set(Robot.FORKLIFT_NOMINAL_DOWN);
            } else if(accessoryJoystick.getRawButton(4)) { // While trying to go up and not at the upper limit
            	forkliftMotorController.set(Robot.FORKLIFT_FULL_LIFT);
            } else if(accessoryJoystick.getRawButton(2)) { // Trying to go down and not at lower limit
            	forkliftMotorController.set(Robot.FORKLIFT_NOMINAL_DOWN);
            } else if(accessoryJoystick.getRawButton(1)) { // Hold a little
            	forkliftMotorController.set(Robot.FORKLIFT_NOMINAL_UP);
            } else if(accessoryJoystick.getRawButton(3)) { // Hold a lot
            	forkliftMotorController.set(Robot.FORKLIFT_ALOT_UP);
            } else {
            	if(Robot.MAP_YAXIS_TO_FORKLIFT) {
            		forkliftMotorController.set(-accessoryJoystick.getY()); // For testing, map the accessory 
            										// joystick y-axis directly to the forklift motor controller
            	} else {
                	forkliftMotorController.set(0); // When not testing,f no button is pressed, stop the motor
            	}*/

           // }
            
            
            
            // When we hit the bottom forklift limit, reset to zero
            /*if(isForkliftAtLowerLimit()) {
            	forkliftEncoder.reset();
            }
            
            // Output debug information - e.g. - SmartDashboard.putBoolean("isInContactWithItem()", isInContactWithItem());
        	SmartDashboard.putNumber("accessoryJoystick.getY (forklift)", accessoryJoystick.getY());
        	SmartDashboard.putBoolean("isForkliftAtLowerLimit()", isForkliftAtLowerLimit());
        	SmartDashboard.putBoolean("isForkliftAtUpperLimit()", isForkliftAtUpperLimit());
        	SmartDashboard.putBoolean("isInContactWithItem()", isInContactWithItem());
        	SmartDashboard.putNumber("Timer.getMatchTime()", Timer.getMatchTime());
        	SmartDashboard.putNumber("Timer.getFPGATimestamp()", Timer.getFPGATimestamp());
//        	SmartDashboard.putNumber("forkliftMotorController.getRaw()", forkliftMotorController.getRaw());
//        	if(sonar.isRangeValid()) {
//        		SmartDashboard.putNumber("Range (in)", sonar.getRangeMM());*/
//        	} else {
//        		SmartDashboard.putNumber("Range (in)", -1);
//        	}
            
            Timer.delay(Robot.UPDATE_DELAY); // Wait the specified second before updating again
        }
        
        // Stop the robot
        robotDrive.drive(0, 0);
    }

    protected void disabled() {
        robotDrive.drive(0, 0);
        robotInitted = false;
        //forkliftMotorController.set(0);
    }
    
   /* protected boolean isForkliftAtLowerLimit() {
    	return !forkliftLowerSwitch.get();
    }
    
    protected boolean isForkliftAtUpperLimit() {
    	return !forkliftUpperSwitch.get();
    }
    
    protected boolean isInContactWithItem() {
    	return !pickupItemContactSwitch.get();
    }*/
    
    
    
    /**
     * This method leverages a heuristic to "efficiently" add a dead zone
     *  and scale the input to a quadratic curve
     * @param input value to be scaled, assumed to be [-1,1]
     * @return scaled input
     */
    public double scaleJoystickInput(double input) {
        // Scales input from [-1,1] to [-0.03,0.97]
        double adjustedInput = input * input - 0.03;
        if(adjustedInput < 0) { // Interpret < 0 as deadzone
            adjustedInput = 0;
        } else if(adjustedInput > 0.9) { // Interpret anything near max as max
            adjustedInput = 1;
        }
        
        if(input < 0) { // Adjust sign to match original input
            adjustedInput = -adjustedInput;
        }
        
        return adjustedInput;
    }
    
	private void putSmartDashboardValues() {
		SmartDashboard.putNumber("driverJoystick.getX()", driverJoystick.getX());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getX())",
		                            scaleJoystickInput(driverJoystick.getX()));
		SmartDashboard.putNumber("driverJoystick.getY()",driverJoystick.getY());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getY())",
		                            scaleJoystickInput(driverJoystick.getY()));
		SmartDashboard.putNumber("driverJoystick.getZ()",driverJoystick.getZ());
		SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getZ())",
		                            scaleJoystickInput(driverJoystick.getZ()));
		SmartDashboard.putNumber("driverJoystick.getThrottle()", driverJoystick.getThrottle());
		SmartDashboard.putNumber("accessoryJoystick.getX())",accessoryJoystick.getX());
		SmartDashboard.putNumber("accessoryJoystick.getY()",accessoryJoystick.getY());
		SmartDashboard.putNumber("scaleJoystickInput(accessoryJoystick.getY())",
		                            scaleJoystickInput(accessoryJoystick.getX()));
		SmartDashboard.putNumber("accessoryJoystick.getZ()",accessoryJoystick.getZ());
		SmartDashboard.putNumber("accessoryJoystick.getThrottle()", accessoryJoystick.getThrottle());
		SmartDashboard.putNumber("accessoryJoystick.getTwist()",accessoryJoystick.getTwist());
		SmartDashboard.putBoolean("accessoryJoystick.getRawButton(2)",accessoryJoystick.getRawButton(2));
		SmartDashboard.putBoolean("accessoryJoystick.getRawButton(3)",accessoryJoystick.getRawButton(3));
	}
	
}

