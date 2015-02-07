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
	
	public enum State {
		INACTIVE, START, FORWARD, LEFT, BACK, RIGHT,
		TELEOP_DRIVE, TELEOP_GYRO_DRIVE
	}

	State currentState = State.START;

	final double kPgyro = 0.02;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = 0.3;

	PIDTool pidGyro;

	Joystick driverStick;
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

		fRight = new Talon(0);
		bLeft = new Talon(2);
		fLeft = new Talon(3);
		bRight = new Talon(1);
		meci = new RobotDrive(fRight, bRight, bLeft, fLeft);

		elevator = new Victor(4);
		rightIntake = new Victor(5);
		leftIntake = new Victor(6);

		meci = new RobotDrive(fRight, bRight, bLeft, fLeft);

		gyro = new Gyro(0);
		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);
		topLimit = new DigitalInput(6);
		oneToteLimit = new DigitalInput(4);
		bottomLimit = new DigitalInput(5);
		gyro.setSensitivity(0.007);

		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
	}

	public void autonomousInit() {
		currentState = State.START;
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
			currentState = State.FORWARD;
			break;

		case FORWARD:
			if (forwardDistance <= 300) {
				yInput = -1.0;
			} else {
				currentState = State.RIGHT;
				resetEncoders();
			}
			break;

		case RIGHT:
			if (strafeDistance <= 100) {
				xInput = 1.0;
			} else {
				currentState = State.BACK;
				resetEncoders();
			}
			break;

		case BACK:
			if (forwardDistance >= -300) {
				yInput = 1.0;
			} else {
				currentState = State.LEFT;
				resetEncoders();
			}
			break;

		case LEFT:
			if (strafeDistance >= -100) {
				xInput = -1.0;
			} else {
				currentState = State.INACTIVE;
				resetEncoders();
			}
			break;

		case INACTIVE:
			break;

		}

		meci.mecanumDrive_Cartesian(xInput * 0.2, -zInput, yInput * 0.2, 0);
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopInit() {
		gyro.reset();
		resetEncoders();
		pidGyro.setSetpoint(0.0);
		currentState = State.TELEOP_DRIVE;
	}

	public void teleopPeriodic() {
		
		double xInput, yInput, zInput;
		double angle = gyro.getAngle();
		
		
		if(driverStick.getRawButton(9)){
			gyro.reset();
			
			xInput = 0.0;
			yInput = 0.4;
			zInput = pidGyro.computeControl(angle);
		} else {
		// Input (z) is - clockwise, + counterclockwise,

		// Y input when pushing joystick forward is negative
		xInput = driverStick.getX() * 0.4;
		yInput = driverStick.getY() * 0.4;
		zInput = driverStick.getZ() * 0.4;
		}
		// Read the gyro angle in degrees - angle increases clockwise

		readEncoders();

		// Press 1 to enable rotational control
		if (driverStick.getRawButton(1)) {
			if (currentState != State.TELEOP_GYRO_DRIVE)
				gyro.reset();
			
			currentState = State.TELEOP_GYRO_DRIVE;
			zInput = pidGyro.computeControl(gyro.getAngle());
		} else {
			currentState = State.TELEOP_DRIVE;
		}

		meci.mecanumDrive_Cartesian(xInput, zInput, yInput, 0);

		SmartDashboard.putString("DB/String 0", currentState.name());
		
		SmartDashboard.putString("DB/String 2", Double.toString(forwardDistance));
		SmartDashboard.putString("DB/String 3", Double.toString(strafeDistance));

		SmartDashboard.putString("DB/String 4", Double.toString(gyro.getAngle()));

		SmartDashboard.putString("DB/String 9", Boolean.toString(oneToteLimit.get()));

		if (driverStick.getRawButton(12)) {
			resetEncoders();
		}

		intake();

		// Elevator control
		if (driverStick.getRawButton(3)) {
			if (bottomLimit.get() == true) {
				elevator.set(1.0);
			} else {
				elevator.set(0.0);
			}
		} else if (driverStick.getRawButton(4)) {
			if (topLimit.get() == true) {
				elevator.set(-1.0);
			} else {
				elevator.set(0.0);
			}
		} else if (driverStick.getRawButton(6)) {
			pickUp();
		} else {
			elevator.set(0.0);
		}

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
		if (driverStick.getRawButton(7)) {
			rightIntake.set(1.0);
			leftIntake.set(1.0);
		} else if (driverStick.getRawButton(8)) {
			rightIntake.set(-1.0);
			leftIntake.set(-1.0);
		} else {
			rightIntake.set(0.0);
			leftIntake.set(0.0);
		}
	}
	

}