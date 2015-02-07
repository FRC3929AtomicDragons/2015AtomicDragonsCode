package org.usfirst.frc.team3929.robot;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	public enum AutonState {
		START, FORWARD, LEFT, BACK, RIGHT, STOP
	}

	AutonState currentState = AutonState.START;
	
	public enum DriveMode {
		TELEOP_DRIVE, TELEOP_GYRO_DRIVE
	}
	
	DriveMode currentDriveMode = DriveMode.TELEOP_DRIVE;

	final double kPgyro = 0.02;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = 0.3;

	PIDTool pidGyro;

	Joystick driverStick;
	Joystick operatorStick;
	
	Talon fRight;
	Talon fLeft;
	Talon bLeft;
	Talon bRight;
	Servo vert;
	Servo horiz;

	DigitalInput topLimit;
	DigitalInput oneToteLimit;
	DigitalInput bottomLimit;
	Victor elevator;
	Timer timer;

	Victor rightIntake;
	Victor leftIntake;
	Gyro gyro;

	RobotDrive meci;

	Encoder rightEncoder;
	Encoder leftEncoder;
	
	// Distances in centimeters. forwardDistance increases as the bot travels forward
	// strafeDistance increases as the bot moves to the right.
	double forwardDistance, strafeDistance;

	/**
	 * Initializing all the objects of the robot. This function is run when the
	 * robot is first started up and should be used for any initialization code.
	 */
	public void robotInit() {
		// sets the channel for different components of the robot

		driverStick = new Joystick(0);
		operatorStick = new Joystick(1);

		fRight = new Talon(0);
		bLeft = new Talon(2);
		fLeft = new Talon(3);
		bRight = new Talon(1);
		meci = new RobotDrive(fRight, bRight, bLeft, fLeft);

		elevator = new Victor(4);
		rightIntake = new Victor(5);
		leftIntake = new Victor(6);

		meci = new RobotDrive(fRight, bRight, bLeft, fLeft);

		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		
		topLimit = new DigitalInput(6);
		oneToteLimit = new DigitalInput(4);
		bottomLimit = new DigitalInput(5);

		gyro = new Gyro(0);
		gyro.setSensitivity(0.007);

		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
	}

	public void autonomousInit() {
		currentState = AutonState.START;
		gyro.reset();
		resetEncoders();
		pidGyro.setSetpoint(0.0);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		double angle = gyro.getAngle();
		double xInput, yInput, zInput;
		
		readEncoders();

		SmartDashboard.putString("DB/String 5", Double.toString(forwardDistance));
		SmartDashboard.putString("DB/String 6", Double.toString(strafeDistance));
		SmartDashboard.putString("DB/String 0", currentState.name());

		xInput = 0.0;
		yInput = 0.0;
		zInput = pidGyro.computeControl(angle);

		switch (currentState) {

		case START:
			currentState = AutonState.FORWARD;
			break;

		case FORWARD:
			if (forwardDistance <= 300) {
				yInput = -1.0;
			} else {
				currentState = AutonState.RIGHT;
				resetEncoders();
			}
			break;

		case RIGHT:
			if (strafeDistance <= 100) {
				xInput = 1.0;
			} else {
				currentState = AutonState.BACK;
				resetEncoders();
			}
			break;

		case BACK:
			if (forwardDistance >= -300) {
				yInput = 1.0;
			} else {
				currentState = AutonState.LEFT;
				resetEncoders();
			}
			break;

		case LEFT:
			if (strafeDistance >= -100) {
				xInput = -1.0;
			} else {
				currentState = AutonState.STOP;
				resetEncoders();
			}
			break;

		case STOP:
			break;

		}

		meci.mecanumDrive_Cartesian(xInput * 0.2, zInput, yInput * 0.2, 0);
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopInit() {
		gyro.reset();
		resetEncoders();
		pidGyro.setSetpoint(0.0);
		currentDriveMode = DriveMode.TELEOP_DRIVE;
	}

	public void teleopPeriodic() {
		
		double xInput = 0.0, yInput = 0.0, zInput = 0.0;
		
		// Y input when pushing joystick forward is negative
		
		// Handle base control from driverStick
		switch (currentDriveMode) {
		case TELEOP_DRIVE:
			
			xInput = driverStick.getX() * 0.4;
			yInput = driverStick.getY() * 0.4;
			zInput = -driverStick.getZ() * 0.4;
			
			if (driverStick.getRawButton(1)) {
				currentDriveMode = DriveMode.TELEOP_GYRO_DRIVE;
				gyro.reset();
			}
			break;
		case TELEOP_GYRO_DRIVE:
			xInput = 0.0;
			yInput = driverStick.getY() * 0.4;
			zInput = pidGyro.computeControl(gyro.getAngle());
			
			if (!driverStick.getRawButton(1))
				currentDriveMode = DriveMode.TELEOP_DRIVE;
			break;
		}
		
		meci.mecanumDrive_Cartesian(xInput, zInput, yInput, 0);

		
		readEncoders();


		SmartDashboard.putString("DB/String 0", currentState.name());
		
		SmartDashboard.putString("DB/String 2", Double.toString(forwardDistance));
		SmartDashboard.putString("DB/String 3", Double.toString(strafeDistance));

		SmartDashboard.putString("DB/String 4", Double.toString(gyro.getAngle()));

		SmartDashboard.putString("DB/String 9", Boolean.toString(oneToteLimit.get()));


		if (driverStick.getRawButton(12)) {
			resetEncoders();
		}
		
		// Handle operator controls
		elevatorControl (operatorStick.getY());
	}
	
	public void elevatorControl (double input) {
		// Check limit switches
		if ((input < 0) && (!topLimit.get())) input = 0.0;
		if ((input > 0) && (!bottomLimit.get())) input = 0.0;
		
		elevator.set(input);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
	}

	/**
	public double forwardDistance(int countLeft, int countRight) {
		final double FORWARD_TO_CM = 0.1;
		return FORWARD_TO_CM * (countLeft + countRight);
	}

	public double strafeDistance(int countLeft, int countRight) {
		final double STRAFE_TO_CM = 0.085;
		return STRAFE_TO_CM * (countLeft - countRight);
	}
	**/
	
	public void readEncoders () {
		int countLeft   = leftEncoder.get();
		int countRight  = rightEncoder.get();
		
		forwardDistance = 0.1 * (countLeft + countRight);
		strafeDistance  = 0.085 * (countLeft - countRight);
	}

	public void resetEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
		forwardDistance = 0.0;
		strafeDistance = 0.0;
	}

	public void pickUp() {
		if (bottomLimit.get() == true) {
			elevator.set(1.0);

			if (bottomLimit.get() == false) {
				elevator.set(-1.0);

				if (oneToteLimit.get() == false) {
					elevator.set(0.0);
				}
			}
		}
	}

	public void intake() {
		if (operatorStick.getRawButton(4)) {
			rightIntake.set(1.0);
			leftIntake.set(1.0);
		} else if (operatorStick.getRawButton(5)) {
			rightIntake.set(-1.0);
			leftIntake.set(-1.0);
		} else {
			rightIntake.set(0.0);
			leftIntake.set(0.0);
		}
	}
	

}