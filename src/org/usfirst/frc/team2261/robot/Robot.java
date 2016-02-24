package org.usfirst.frc.team2261.robot;

/*----------------------------------------------------------------------------*/

/* Copyright (c) FIRST 2008. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/* CLEAN IF YOU CANNOT CONNECT TO THE ROBORIO. HOW TO CLEAN: Go to Project > Clean to clean up project; may be able to work fine after.
 Another way to clean the roboRIO/DNS: Press Windows key > Search for Command Prompt > in the Prompt type in "ping" (roboRIO-2261-FRC.local)
or (10.22.61.78 to see if you can get a reply from it; After that use the command "ipconfig /flushdns" to clear out the Cache; problem
should be fixed to build the code. */
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the SimpleRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends SampleRobot {

	// private static final double DEADZONE = -0.3;
	static double UPDATE_DELAY = 0.0075; // How often to refresh
	static double MOTOR_EXPIRATION = UPDATE_DELAY * 4;
	static double CLAW_UP = 3.0;
	static double CLAW_DOWN = 0.5;

	// Dampening constants
	// Not used until we have ability to do so.
	private static final double kPWM = 0.005; // added .005 to 0; hopefully able
												// to have increased driving
												// ability.
	private static final double kPWM_MAX = 0.1; // dropped 1 down to 0.1; made
												// the driving speed have less
												// turning radius.
	private static final double kdPWM_MAX = 0.1;

	// IMPORTANT!!!!!!
	// Set to true for driving fork_lift from accessory y-axis (when not at
	// limit or being driven by accessory buttons)
	static boolean MAP_YAXIS_TO_FORKLIFT = true;
	static boolean MAP_ZAXIS_TO_ClAW = true;

	// Values for fork_lift motor (determined by testing)
	static double FORKLIFT_NOMINAL_UP = 0.2;
	static double FORKLIFT_ALOT_UP = 0.2;
	static double FORKLIFT_FULL_LIFT = 0.75;
	static double FORKLIFT_NOMINAL_DOWN = -0.5;
	static double CLAW_OPEN = 0.5;
	static double CLAW_CLOSE = -0.5;
	static double KICK_DOWN = -0.9;// ARM Added much more added effect to
									// Kicker; originally -.01 and added to
									// -.09.
	static double KICK_UP = 1.0;// DISARM Added much more added effect to
								// Kicker; originally .01 and added to .09.

	// Create a new RobotDrive, composed of the following motor controllers
	Talon rightController = new Talon(2); // Undo'd the Talon.
	Talon leftController = new Talon(3); // Undo'd the Talon.
	RobotDrive robotDrive = new RobotDrive(leftController, rightController);

	Spark clawController = new Spark(5);

	Spark upAndDownController = new Spark(6);

	Spark kickerController = new Spark(7); // Sparks number 7 and number 8 are
											// shared.

	// Create new Joy_stick on ports 1 and 2
	Joystick driverJoystick = new Joystick(0);
	Joystick accessoryJoystick = new Joystick(1);
	static double prevPWM, dPWM = 0;

	// Kicker_Claw Sensor
	// DigitalInput forkliftLowerSwitch = new DigitalInput(2);
	// DigitalInput forkliftUpperSwitch = new DigitalInput(3);
	DigitalInput kickerSwitch = new DigitalInput(0);
	DigitalInput forkliftSwitch = new DigitalInput(3);

	boolean robotInitted = false;
	boolean useForklift = false;

	protected void robotInit() {
		// (Re-)Enable motor safety with .25s expiration
		// "Motor safety is a mechanism built into the RobotDrive object that
		// will turn off the motors if the program doesn't continuously update
		// the motor speed." (default "expiration" is 100ms)
		robotDrive.setExpiration(Robot.MOTOR_EXPIRATION);
		robotDrive.setSafetyEnabled(true);

		upAndDownController.setExpiration(Robot.MOTOR_EXPIRATION);
		upAndDownController.setSafetyEnabled(true);

		// Reverse left side drive
		// robotDrive.setInvertedMotor(RobotDrive.MotorType.kFrontLeft, true);

		// Ensure all motors are stopped (I don't believe we should have to do
		// this)
		robotDrive.drive(0, 0);

		robotInitted = true;
	}

	public void autonomous() {
		// Driver forward at half speed for 3 seconds
		// robotDrive.drive(0.25, 0);
		// Timer.delay(3);
		// robotDrive.drive(0, 0);
	}

	/**
	 * This function is called once each time the robot enters operator control.
	 */
	public void operatorControl() {
		robotDrive.setExpiration(MOTOR_EXPIRATION);

		if (!robotInitted) {
			System.out.println("Robot not initted? Initting...");
			robotInit();
		}

		// While still under operator control and enabled, "arcade drive" robot,
		// updating motors every 0.0075 second
		while (isOperatorControl() && isEnabled()) {

			// Drive the Robot
			// Changed 2 to -2 on division for both drives
			robotDrive.arcadeDrive(scaleJoystickInput(driverJoystick.getY()) / -1.4123, // Y=Forward
																						// and
																						// Backward
																						// Drive
																						// (Divided
																						// by
																						// -4
																						// instead
																						// of
																						// -2
																						// to
																						// ensure
																						// speed
																						// WAS
																						// lost
																						// during
																						// process.)
					scaleJoystickInput(driverJoystick.getZ()) / -1.4123); // Z=
																			// Rotate
																			// Clockwise
																			// and
																			// Counter-Clockwise
																			// (Divided
																			// by
																			// 4
																			// instead
																			// of
																			// 1
																			// to
																			// ensure
																			// speed
																			// WAS
																			// lost
																			// during
																			// process.)

			// Fork_lift Operation
			/*
			 * if(isForkliftAtLowerLimit()) { // Trying to go down and not at
			 * lower limit upAndDownController.set(Robot.FORKLIFT_NOMINAL_UP); }
			 * else if(isForkliftAtUpperLimit()) { // While trying to go up and
			 * not at the upper limit
			 * upAndDownController.set(Robot.FORKLIFT_NOMINAL_DOWN); } else
			 * if(accessoryJoystick.getRawButton(4)) { // While trying to go up
			 * and not at the upper limit
			 * upAndDownController.set(Robot.FORKLIFT_FULL_LIFT); } else
			 * if(accessoryJoystick.getRawButton(2)) { // Trying to go down and
			 * not at lower limit
			 * upAndDownController.set(Robot.FORKLIFT_NOMINAL_DOWN); } else
			 * if(accessoryJoystick.getRawButton(1)) { // Hold a little
			 * upAndDownController.set(Robot.FORKLIFT_NOMINAL_UP); } else
			 * if(accessoryJoystick.getRawButton(3)) { // Hold a lot
			 * upAndDownController.set(Robot.FORKLIFT_ALOT_UP); } else {
			 * if(Robot.MAP_YAXIS_TO_FORKLIFT) {
			 * upAndDownController.set(-accessoryJoystick.getY()); // For
			 * testing, map the accessory // joystick y-axis directly to the
			 * forklift motor controller } else { upAndDownController.set(0); //
			 * When not testing,f no button is pressed, stop the motor } }
			 */

			if (accessoryJoystick.getRawButton(4)) { // Y button
				upAndDownController.set(Robot.FORKLIFT_FULL_LIFT);

			} else if (accessoryJoystick.getRawButton(2)) { // A button
				upAndDownController.set(Robot.FORKLIFT_NOMINAL_DOWN);

			} else {
				if (Robot.MAP_YAXIS_TO_FORKLIFT) {
					upAndDownController.set(-accessoryJoystick.getY()); // For
																		// testing,
																		// map
																		// the
																		// accessory
					// joystick y-axis directly to the forklift motor controller
				} else {
					upAndDownController.set(0); // When not testing,f no button
												// is pressed, stop the motor
				}
			}

			// Known that claw works; reversing coding and also added Z axis for
			// claw as well.
			if (accessoryJoystick.getRawButton(1)) { // X Button
				clawController.set(Robot.CLAW_OPEN);
			} else if (accessoryJoystick.getRawButton(3)) { // B button
				clawController.set(Robot.CLAW_CLOSE);
			} else {
				if (Robot.MAP_ZAXIS_TO_ClAW) {
					clawController.set(-accessoryJoystick.getZ()); // For
																	// testing,
																	// map the
																	// accessory
					// joystick z-axis directly to the forklift motor controller

				} else {
					clawController.set(0); // When not testing,f no button is
											// pressed, stop the motor
				}
			}

			if ((accessoryJoystick.getRawButton(7) || isForkLiftMoving()) && !isKickerLoaded()) { // Left
																			// Trigger
																			// button
				kickerController.set(Robot.KICK_UP);// Removed original kick up
													// and replaced kick down
													// back to kick up.

			} else if (accessoryJoystick.getRawButton(8)) { // Right Trigger
															
				kickerController.set(Robot.KICK_UP);// Removed original kick up and replaced kick down back to kick up.
													 
			} else {
				kickerController.set(0); // When not testing,f no button is
											// pressed, stop the motor
			}

			// puts claw into shooting position. delay seconds must be tested
			/*
			 * if(accessoryJoystick.getRawButton(6)) { // RT button
			 * upAndDownController.set(Robot.FORKLIFT_FULL_LIFT);
			 * Timer.delay(Robot.CLAW_UP);
			 * upAndDownController.set(Robot.FORKLIFT_NOMINAL_DOWN);
			 * Timer.delay(Robot.CLAW_DOWN); upAndDownController.set(0);
			 * 
			 * }
			 */

			// Print debug info
			putSmartDashboardValues();

			Timer.delay(Robot.UPDATE_DELAY); // Wait the specified second before
												// updating again
		}

		// Stop the robot
		robotDrive.drive(0, 0);
	}

	private boolean isForkLiftMoving() {
		return (accessoryJoystick.getRawButton(4) || accessoryJoystick.getRawButton(2) || accessoryJoystick.getY() > 0.1 || accessoryJoystick.getY() < -0.1 );
	}


	private double getAdjustedJoystickValue() {
		double scaledJoystick = scaleJoystickInput(driverJoystick.getY());

		scaledJoystick = getDampenedJoystickInput(scaledJoystick);

		return scaledJoystick;
	}

	protected static double getDampenedJoystickInput(double scaledJoystick) {
		double dPWN_new = scaledJoystick;

		dPWN_new = kPWM * dPWN_new;
		dPWN_new = sign(dPWN_new) * Math.min(kPWM_MAX, dPWN_new);

		double ddPWM_req = dPWN_new - dPWM;
		double ddPWM_new = sign(ddPWM_req) * Math.min(kdPWM_MAX, ddPWM_req);
		dPWN_new = dPWN_new + ddPWM_new;

		dPWM = dPWN_new;
		prevPWM = dPWN_new + prevPWM;

		return prevPWM;
	}

	private static double sign(double value) {
		return value < 0 ? -1 : 1;
	}

	protected void disabled() {
		robotDrive.drive(0, 0);
		robotInitted = false;
		upAndDownController.set(0);
	}

	protected boolean isKickerLoaded() {
		return !kickerSwitch.get();
	}

	protected boolean isForkliftAtUpperLimit() {
		return !forkliftSwitch.get();
	}

	/**
	 * This method leverages a heuristic to "efficiently" add a dead zone and
	 * scale the input to a quadratic curve
	 * 
	 * @param input
	 *            value to be scaled, assumed to be [-1,1]
	 * @return scaled input
	 */
	public double scaleJoystickInput(double input) {
		// Scales input from [-1,1] to [-0.03,0.97]
		double adjustedInput = input * input - 0.03;
		if (adjustedInput < 0) { // Interpret < 0 as deadzone
			adjustedInput = 0;
		} else if (adjustedInput > 0.9) { // Interpret anything near max as max
			adjustedInput = 1;
		}

		if (input < 0) { // Adjust sign to match original input
			adjustedInput = -adjustedInput;
		}

		return adjustedInput;
	}

	private void putSmartDashboardValues() {
		/*
		 * SmartDashboard.putNumber("accessoryJoystick.getY (forklift)",
		 * accessoryJoystick.getY());
		 * SmartDashboard.putBoolean("isForkliftAtUpperLimit()",
		 * isForkliftAtLowerLimit());
		 */
		/*
		 * SmartDashboard.putBoolean("isKickerLoaded()", isKickerLoaded());
		 * SmartDashboard.putBoolean("isForkliftAtUpperLimit()",
		 * isForkliftAtUpperLimit());
		 * SmartDashboard.putNumber("upAndDownController.getRaw()",
		 * upAndDownController.getRaw());
		 * 
		 * SmartDashboard.putNumber("driverJoystick.getX()",
		 * driverJoystick.getX());
		 * SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getX())",
		 * scaleJoystickInput(driverJoystick.getX()));
		 * SmartDashboard.putNumber("driverJoystick.getY()",driverJoystick.getY(
		 * ));
		 * SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getY())",
		 * scaleJoystickInput(driverJoystick.getY()));
		 * SmartDashboard.putNumber("driverJoystick.getZ()",driverJoystick.getZ(
		 * ));
		 * SmartDashboard.putNumber("scaleJoystickInput(driverJoystick.getZ())",
		 * scaleJoystickInput(driverJoystick.getZ()));
		 * SmartDashboard.putNumber("driverJoystick.getThrottle()",
		 * driverJoystick.getThrottle());
		 * SmartDashboard.putNumber("accessoryJoystick.getX())",
		 * accessoryJoystick.getX());
		 * SmartDashboard.putNumber("accessoryJoystick.getY()",accessoryJoystick
		 * .getY()); SmartDashboard.putNumber(
		 * "scaleJoystickInput(accessoryJoystick.getY())",
		 * scaleJoystickInput(accessoryJoystick.getX()));
		 * SmartDashboard.putNumber("accessoryJoystick.getZ()",accessoryJoystick
		 * .getZ()); SmartDashboard.putNumber("accessoryJoystick.getThrottle()",
		 * accessoryJoystick.getThrottle());
		 * SmartDashboard.putNumber("accessoryJoystick.getTwist()",
		 * accessoryJoystick.getTwist());
		 * SmartDashboard.putBoolean("accessoryJoystick.getRawButton(2)",
		 * accessoryJoystick.getRawButton(2));
		 * SmartDashboard.putBoolean("accessoryJoystick.getRawButton(3)",
		 * accessoryJoystick.getRawButton(3));
		 */
		SmartDashboard.putBoolean("isKickerLoaded()", isKickerLoaded());
		
	}

}
