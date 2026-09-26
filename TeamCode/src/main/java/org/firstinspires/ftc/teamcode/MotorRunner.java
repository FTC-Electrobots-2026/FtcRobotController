package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MotorRunner", group = "TeleOp")
public class MotorRunner extends OpMode {

	private static final double JOYSTICK_DEADBAND = 0.08;

	// PID coefficients for encoder-based control
	private static final double PID_KP = 0.02;
	private static final double PID_KI = 0.001;
	private static final double PID_KD = 0.0;
	private static final double MAX_RPM = 300.0; // Example: adjust to your motor

	private DcMotorEx leftFront;
	private IMU imu;
	private long lastImuLogTimeMs;

	// PID state for left joystick control
	private double pidIntegral = 0.0;
	private double pidPrevError = 0.0;
	private long pidLastTimeMs = 0;

	@Override
	public void init() {
		leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setPower(0.0);

		imu = hardwareMap.get(IMU.class, "imu");

		RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
				RevHubOrientationOnRobot.LogoFacingDirection.UP;
		RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
				RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
		RevHubOrientationOnRobot orientationOnRobot =
				new RevHubOrientationOnRobot(logoDirection, usbDirection);

		imu.initialize(new IMU.Parameters(orientationOnRobot));

		// Reset PID state
		pidLastTimeMs = 0;
		pidIntegral = 0.0;
		pidPrevError = 0.0;

		telemetry.addData("Status", "Initialized");
		telemetry.addData("Motor", "leftFront ready");
		telemetry.addData("Deadband", JOYSTICK_DEADBAND);
		telemetry.addData("IMU Mount", "Logo UP, USB FORWARD");
		telemetry.addData("", "");
		telemetry.addData("RIGHT joystick: Raw power (no feedback)", "");
		telemetry.addData("LEFT joystick: PID + Encoder (feedback)", "");
		telemetry.update();
	}

	@Override
	public void loop() {
		// RIGHT joystick: raw power (no feedback)
		double rightStickY = -gamepad1.right_stick_y * 0.5;
		double rawPower = applyDeadband(rightStickY);
		rawPower = Range.clip(rawPower, -1.0, 1.0);

		// LEFT joystick: PID-controlled speed (with encoder feedback)
		double leftStickY = -gamepad1.left_stick_y * 0.5;
		double targetSpeedFraction = applyDeadband(leftStickY);
		targetSpeedFraction = Range.clip(targetSpeedFraction, -1.0, 1.0);
		double pidPower = calculatePidPower(targetSpeedFraction);

		// Choose control mode: if right stick is used, go raw; otherwise use left stick with PID
		double activePower;
		String controlMode;
		if (Math.abs(rawPower) > JOYSTICK_DEADBAND) {
			activePower = rawPower;
			controlMode = "RAW (Right Stick)";
		} else {
			activePower = pidPower;
			controlMode = "PID (Left Stick)";
		}

		leftFront.setPower(activePower);

		// Calculate RPM from encoder
		double rpm = calculateRpm();

		telemetry.addData("Status", "Running");
		telemetry.addData("Control Mode", controlMode);
		telemetry.addData("Motor Power", "%.3f", activePower);
		telemetry.addData("Motor RPM", "%.1f", rpm);
		telemetry.addData("", "");
		telemetry.addData("Right Stick Y (raw)", "%.3f", rightStickY);
		telemetry.addData("Left Stick Y (PID)", "%.3f", leftStickY);
		telemetry.addData("Deadband", JOYSTICK_DEADBAND);

		showImuInfo();
		telemetry.update();
	}

	private double applyDeadband(double value) {
		if (Math.abs(value) <= JOYSTICK_DEADBAND) {
			return 0.0;
		}

		return value;
	}

	/**
	 * Calculate RPM from motor encoder.
	 * Uses velocity; divide by ticks-per-rev if available.
	 */
	private double calculateRpm() {
		double velocityTicksPerSec = leftFront.getVelocity();
		// REV HD Hex Motor: 8.3 ticks per revolution (at output)
		// Adjust this constant for your specific motor
		double ticksPerRevolution = 537.7;
		double rps = velocityTicksPerSec / ticksPerRevolution;
		double rpm = rps * 60.0;
		return rpm;
	}

	/**
	 * PID controller for target speed control using encoder feedback.
	 * targetSpeedFraction: -1 to 1 representing desired speed
	 */
	private double calculatePidPower(double targetSpeedFraction) {
		long now = System.currentTimeMillis();

		if (pidLastTimeMs == 0) {
			pidLastTimeMs = now;
			pidIntegral = 0.0;
			pidPrevError = 0.0;
			return 0.0;
		}

		double dtSec = (now - pidLastTimeMs) / 1000.0;
		pidLastTimeMs = now;

		// Current RPM
		double currentRpm = calculateRpm();

		// Target RPM based on joystick input
		double targetRpm = targetSpeedFraction * MAX_RPM;

		// Error: difference between target and current
		double error = targetRpm - currentRpm;

		// PID terms
		double pTerm = PID_KP * error;
		pidIntegral += error * dtSec;
		pidIntegral = Range.clip(pidIntegral, -1.0, 1.0); // Anti-windup
		double iTerm = PID_KI * pidIntegral;

		double dTerm = 0.0;
		if (dtSec > 0) {
			double errorRate = (error - pidPrevError) / dtSec;
			dTerm = PID_KD * errorRate;
		}

		pidPrevError = error;

		// Calculate power output
		double power = pTerm + iTerm + dTerm;
		power = Range.clip(power, -1.0, 1.0);

		return power;
	}

	private void showImuInfo() {
		YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
		AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

		telemetry.addData("IMU Yaw (deg)", "%.2f", orientation.getYaw(AngleUnit.DEGREES));
		telemetry.addData("IMU Pitch (deg)", "%.2f", orientation.getPitch(AngleUnit.DEGREES));
		telemetry.addData("IMU Roll (deg)", "%.2f", orientation.getRoll(AngleUnit.DEGREES));
		telemetry.addData("IMU Yaw Rate (deg/s)", "%.2f", angularVelocity.zRotationRate);
		telemetry.addData("IMU Pitch Rate (deg/s)", "%.2f", angularVelocity.xRotationRate);
		telemetry.addData("IMU Roll Rate (deg/s)", "%.2f", angularVelocity.yRotationRate);

		long now = System.currentTimeMillis();
		if (now - lastImuLogTimeMs >= 250) {
			RobotLog.ii(
					"MotorRunner",
					"IMU yaw=%.2f pitch=%.2f roll=%.2f yawRate=%.2f pitchRate=%.2f rollRate=%.2f",
					orientation.getYaw(AngleUnit.DEGREES),
					orientation.getPitch(AngleUnit.DEGREES),
					orientation.getRoll(AngleUnit.DEGREES),
					angularVelocity.zRotationRate,
					angularVelocity.xRotationRate,
					angularVelocity.yRotationRate
			);
			lastImuLogTimeMs = now;
		}
	}

	@Override
	public void stop() {
		if (leftFront != null) {
			leftFront.setPower(0.0);
		}
	}
}
