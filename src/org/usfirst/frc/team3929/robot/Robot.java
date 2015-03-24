package org.usfirst.frc.team3929.robot;

import com.kauailabs.navx_mxp.AHRS;
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
		START, INTAKE, LOWER, RAISE, ROTATE, FORWARD, PLACE, STOP
	}

	AutonState currentState = AutonState.START;

	public enum DriveMode {
		TELEOP_DRIVE, TELEOP_GYRO_DRIVE, TELEOP_GYRO_STRAFE
	}

	DriveMode currentDriveMode = DriveMode.TELEOP_DRIVE;

	public enum PickUpState {
		OPEN, DOWN, PICKUP, CLOSE
	}

	public PickUpState stateofAction = PickUpState.OPEN;
	final double kPgyro = 0.04;
	final double kIgyro = 0.0;
	final double kDgyro = 0.0;
	final double MAX_ROTATION_INPUT = 0.3;

	double DRIVE_POWER_LEVEL = 0.6;

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

	Talon rightIntake;
	Talon leftIntake;

	Servo cameraY;
	Servo cameraX;

	double cameraXIn = 0.5;
	double cameraYIn = 0.5;

	RobotDrive meci;

	Encoder rightEncoder;
	Encoder leftEncoder;

	DoubleSolenoid sol;

	SerialPort serial_port;
	// IMU imu; // This class can be used w/nav6 and navX MXP.
	// IMUAdvanced imu; // This class can be used w/nav6 and navX MXP.
	AHRS imu; // This class can only be used w/the navX MXP.

	// Distances in centimeters. forwardDistance increases as the bot travels
	// forward
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

		// elevator originally at port 4
		elevator = new Victor(4);
		rightIntake = new Talon(5);
		leftIntake = new Talon(6);

		meci = new RobotDrive(fRight, bRight, bLeft, fLeft);

		rightEncoder = new Encoder(0, 1, false, Encoder.EncodingType.k4X);
		leftEncoder = new Encoder(2, 3, true, Encoder.EncodingType.k4X);

		topLimit = new DigitalInput(6);
		oneToteLimit = new DigitalInput(4);
		bottomLimit = new DigitalInput(5);

		cameraX = new Servo(7);
		cameraY = new Servo(8);

		sol = new DoubleSolenoid(2, 3);

		// Old analog gyro initialization
		// gyro count increases going counter-clockwise
		// gyro = new Gyro(0);
		// gyro.setSensitivity(0.007);

		try {

			// Use SerialPort.Port.kOnboard if connecting nav6 to Roborio Rs-232
			// port
			// Use SerialPort.Port.kMXP if connecting navX MXP to the RoboRio
			// MXP port
			// Use SerialPort.Port.kUSB if connecting nav6 or navX MXP to the
			// RoboRio USB port

			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);

			// You can add a second parameter to modify the
			// update rate (in hz) from. The minimum is 4.
			// The maximum (and the default) is 100 on a nav6, 60 on a navX MXP.
			// If you need to minimize CPU load, you can set it to a
			// lower value, as shown here, depending upon your needs.
			// The recommended maximum update rate is 50Hz

			// You can also use the IMUAdvanced class for advanced
			// features on a nav6 or a navX MXP.

			// You can also use the AHRS class for advanced features on
			// a navX MXP. This offers superior performance to the
			// IMU Advanced class, and also access to 9-axis headings
			// and magnetic disturbance detection. This class also offers
			// access to altitude/barometric pressure data from a
			// navX MXP Aero.

			byte update_rate_hz = 50;
			// imu = new IMU(serial_port,update_rate_hz);
			// imu = new IMUAdvanced(serial_port,update_rate_hz);
			imu = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {

		}

		timer = new Timer();
		timer.reset();

		pidGyro = new PIDTool(kPgyro, kIgyro, kDgyro, 0, -MAX_ROTATION_INPUT, MAX_ROTATION_INPUT);
	}

	public void autonomousInit() {
		currentState = AutonState.START;
		imu.zeroYaw();
		resetEncoders();
	}

	/**
	 * This function is called periodically during autonomous
	 */
	public void autonomousPeriodic() {
		double angle = imu.getYaw();
		double xInput, yInput, zInput, intakeSpeed, elevatorSpeed;

		readEncoders();

		SmartDashboard.putString("DB/String 5", Double.toString(forwardDistance));
		SmartDashboard.putString("DB/String 6", Double.toString(strafeDistance));
		SmartDashboard.putString("DB/String 8", Double.toString(imu.getYaw()));
		SmartDashboard.putString("DB/String 0", currentState.name());
		SmartDashboard.putString("DB/String 4", Boolean.toString(oneToteLimit.get()));

		xInput = 0.0;
		yInput = 0.0;
		zInput = 0.0;

		intakeSpeed = 0.0;
		elevatorSpeed = 0.0;

		switch (currentState) {

		case START:
			timer.reset();
			timer.start();
			closeArms();
			currentState = AutonState.INTAKE;
			break;

		case INTAKE:
			if (timer.get() <= 2.0) {
				intakeSpeed = 1.0;
			} else {
				currentState = AutonState.LOWER;
				openArms();
				timer.reset();
			}
			break;

		case LOWER:
			if (bottomLimit.get()) {
				elevatorSpeed = 1.0;
			} else {
				currentState = AutonState.RAISE;
			}
			break;

		case RAISE:
			if (!oneToteLimit.get()) {
				elevatorSpeed = -1.0;
			} else {
				resetEncoders();
				imu.zeroYaw();
				pidGyro.setSetpoint(90.0);
				timer.reset();
				currentState = AutonState.ROTATE;
			}
			break;

		case ROTATE:
			
			if ((Math.abs(angle - 90) > 2.0) && (timer.get() < 3.0)) {
				zInput = pidGyro.computeControl(angle);
			} else {
				resetEncoders();
				imu.zeroYaw();
				pidGyro.setSetpoint(0.0);
				currentState = AutonState.FORWARD;
			}
			
			/*if (timer.get() < 2.0) {
				zInput = -0.4;
			} else {
				resetEncoders();
				imu.zeroYaw();
				pidGyro.setSetpoint(0.0);
				currentState = AutonState.FORWARD;
			}*/
			break;

		case FORWARD:
			if (forwardDistance >= -130) {
				zInput = 0.0;
				yInput = -0.5;
			} else {
				currentState = AutonState.STOP;
			}

			break;

		case PLACE:
			if (bottomLimit.get()) {
				elevatorSpeed = 1.0;
			} else {
				currentState = AutonState.STOP;
			}
			break;

		case STOP:
			break;
		}

		intake(0.0, intakeSpeed);
		elevatorControl(elevatorSpeed);
		meci.mecanumDrive_Cartesian(xInput * 0.8, zInput, yInput * 0.8, 0);
	}

	/**
	 * This function is called periodically during operator control
	 */

	public void teleopInit() {
		imu.zeroYaw();
		resetEncoders();
		pidGyro.setSetpoint(0.0);
		currentDriveMode = DriveMode.TELEOP_DRIVE;
	}

	public void teleopPeriodic() {

		double xInput = 0.0, yInput = 0.0, zInput = 0.0;

		// Y input when pushing joystick forward is negative

		// Handle base control from driverStick
		if (driverStick.getRawButton(4)) {
			DRIVE_POWER_LEVEL = 1.0;
		} else {
			DRIVE_POWER_LEVEL = 0.4;
		}

		switch (currentDriveMode) {
		case TELEOP_DRIVE:

			xInput = driverStick.getX() * DRIVE_POWER_LEVEL;
			yInput = driverStick.getY() * DRIVE_POWER_LEVEL;
			zInput = -driverStick.getZ() * DRIVE_POWER_LEVEL;

			if (driverStick.getRawButton(1)) {
				currentDriveMode = DriveMode.TELEOP_GYRO_DRIVE;
				imu.zeroYaw();
			} else if (driverStick.getRawButton(2)) {
				currentDriveMode = DriveMode.TELEOP_GYRO_STRAFE;
				imu.zeroYaw();
			}
			break;

		case TELEOP_GYRO_DRIVE:
			xInput = 0.0;
			yInput = driverStick.getY() * DRIVE_POWER_LEVEL;
			zInput = pidGyro.computeControl(imu.getYaw());
			if (!driverStick.getRawButton(1))
				currentDriveMode = DriveMode.TELEOP_DRIVE;
			break;

		case TELEOP_GYRO_STRAFE:
			xInput = driverStick.getX() * DRIVE_POWER_LEVEL;
			yInput = driverStick.getY() * DRIVE_POWER_LEVEL;
			zInput = pidGyro.computeControl(imu.getYaw());
			if (!driverStick.getRawButton(2)) {
				currentDriveMode = DriveMode.TELEOP_DRIVE;
			}
			break;
		}

		meci.mecanumDrive_Cartesian(xInput, zInput, yInput, 0);

		readEncoders();

		SmartDashboard.putString("DB/String 0", currentDriveMode.name());
		SmartDashboard.putString("DB/String 2", Double.toString(forwardDistance));
		SmartDashboard.putString("DB/String 3", Double.toString(strafeDistance));

		SmartDashboard.putString("DB/String 6", Double.toString(imu.getYaw()));
//		SmartDashboard.putString("DB/String 7", Double.toString(gyro.lastTime));

/*		SmartDashboard.putString("DB/String 5", Double.toString(gyro.gyroBias));
		SmartDashboard.putString("DB/String 4", Short.toString(gyro.getRotationZ()));*/

		if (driverStick.getRawButton(12)) {
			resetEncoders();
		}

		// Handle operator controls
		if (operatorStick.getRawButton(1)) {
			elevatorControl(-operatorStick.getY());
			intake(0.0, 0.0);
		} else if (operatorStick.getRawButton(4)) {
			cameraControl();
			intake(0.0, 0.0);
			elevatorControl(0.0);
		} else {
			intake(operatorStick.getX(), operatorStick.getY());
			elevatorControl(0.0);
		}

		// Setting Servo Position
		cameraX.set(cameraXIn);
		cameraY.set(cameraYIn);

		// Open and close intake
		if (operatorStick.getRawButton(3)) {
			closeArms();
		} else if (operatorStick.getRawButton(2) || operatorStick.getRawButton(1)) {
			openArms();
		} else {
			sol.set(DoubleSolenoid.Value.kOff);
		}
	}

	public void cameraControl() {
		final double PAN_DELTA = 0.01;
		final double TILT_DELTA = 0.01;

		if (operatorStick.getX() > 0.5)
			cameraXIn += PAN_DELTA;
		if (operatorStick.getX() < -0.5)
			cameraXIn -= PAN_DELTA;

		if (operatorStick.getY() > 0.5)
			cameraYIn += TILT_DELTA;
		if (operatorStick.getY() < -0.5)
			cameraYIn -= TILT_DELTA;

		if (cameraXIn > 1.0)
			cameraXIn = 1.0;
		if (cameraXIn < 0.0)
			cameraXIn = 0.0;

		if (cameraYIn > 1)
			cameraYIn = 1.0;
		if (cameraYIn < 0)
			cameraYIn = 0.0;
	}

	public void elevatorControl(double input) {
		// Check limit switches
		if ((input < 0) && (!topLimit.get()))
			input = 0.0;
		if ((input > 0) && (!bottomLimit.get()))
			input = 0.0;

		elevator.set(input);
	}

	/**
	 * This function is called periodically during test mode
	 */
	public void testPeriodic() {
	}

	public void readEncoders() {
		int countLeft = leftEncoder.get();
		int countRight = rightEncoder.get();

		forwardDistance = 0.1 * (countLeft + countRight);
		strafeDistance = 0.085 * (countLeft - countRight);
	}

	public void resetEncoders() {
		rightEncoder.reset();
		leftEncoder.reset();
		forwardDistance = 0.0;
		strafeDistance = 0.0;
	}

	public void intake(double inputX, double inputY) {
		rightIntake.set(inputX + inputY);
		leftIntake.set(inputX - inputY);
	}

	public void openArms() {
		sol.set(DoubleSolenoid.Value.kReverse);
	}

	public void closeArms() {
		sol.set(DoubleSolenoid.Value.kForward);
	}

}
