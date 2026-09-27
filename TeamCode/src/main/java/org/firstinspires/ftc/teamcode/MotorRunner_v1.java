package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "MotorRunner_v1", group = "TeleOp")
public class MotorRunner_v1 extends OpMode {

	private static final double JOYSTICK_DEADBAND = 0.08;

	// PID coefficients for encoder-based control
	// These values are tuned to reduce oscillation and jerking
	private static final double PID_KP = 0.015;    // Proportional gain for responsiveness
	private static final double PID_KD = 0.015;    // Damping to prevent oscillation
	private static final double MAX_RPM = 300.0;   // Max RPM for the motor
	private static final double FEEDFORWARD_FF = 1.0; // Scale factor for feedforward (1.0 = direct stick input)
	private static final double MIN_POWER = 0.03;  // Minimum power to turn motor
	private static final double ERROR_DEADZONE = 0.03; // Tighter deadzone for better tracking

	private DcMotorEx leftFront;
	private IMU imu;
	private long lastImuLogTimeMs;

	// PID state for left joystick control
	private double pidPrevError = 0.0;
	private long pidLastTimeMs = 0;

	@Override
	public void init() {
		leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
		leftFront.setDirection(DcMotor.Direction.REVERSE);
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
		boolean rightStickActive = Math.abs(rawPower) > 0.0;
		boolean leftStickActive = Math.abs(targetSpeedFraction) > 0.0;

		double activePower;
		String controlMode;
		if (rightStickActive) {
			activePower = rawPower;
			controlMode = "RAW (Right Stick)";
			resetPidState();
		} else if (leftStickActive) {
			activePower = calculatePidPower(targetSpeedFraction);

			controlMode = "PID (Left Stick)";
		} else {
			activePower = 0.0;
			controlMode = "STOP";
			resetPidState();
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

		//showImuInfo();
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
		// GoBILDA 5203 19:1: 537.7 ticks per revolution (at output shaft)
		double ticksPerRevolution = 537.7;
		double rps = velocityTicksPerSec / ticksPerRevolution;
		return rps * 60.0;
	}

	/**
	 * PID controller for target speed control using encoder feedback.
	 * targetSpeedFraction: -1 to 1 representing desired speed
	 * Heavily damped to prevent oscillation lock.
	 */
	private double calculatePidPower(double targetSpeedFraction) {
		if (Math.abs(targetSpeedFraction) <= JOYSTICK_DEADBAND) {
			resetPidState();
			return 0.0;
		}

		long now = System.currentTimeMillis();

		if (pidLastTimeMs == 0) {
			pidLastTimeMs = now;
			pidPrevError = 0.0;
			// Start with feedforward power scaled to target speed
			return targetSpeedFraction * FEEDFORWARD_FF;
		}

		double dtSec = (now - pidLastTimeMs) / 1000.0;
		pidLastTimeMs = now;

		// Prevent dt from being too large or too small
		dtSec = Math.max(0.001, Math.min(0.1, dtSec));

		// Current RPM
		double currentRpm = calculateRpm();

		// Target RPM based on joystick input
		double targetRpm = targetSpeedFraction * MAX_RPM;

		// Normalize error to 0-1 range
		double normalizedError = (targetRpm - currentRpm) / MAX_RPM;

		// Error deadzone: if error is very small, don't correct (prevents oscillation)
		if (Math.abs(normalizedError) < ERROR_DEADZONE) {
			// Just use feedforward to maintain speed
			double power = targetSpeedFraction * FEEDFORWARD_FF;
			pidPrevError = normalizedError;
			return power;
		}

		// Proportional term (very small)
		double pTerm = PID_KP * normalizedError;

		// Derivative term (strong damping) - the key to stopping oscillation
		double dTerm = 0.0;
		if (dtSec > 0) {
			double errorRate = (normalizedError - pidPrevError) / dtSec;
			dTerm = PID_KD * errorRate;
		}

		pidPrevError = normalizedError;

		// Feedforward term: apply power proportional to stick input
		double feedForward = targetSpeedFraction * FEEDFORWARD_FF;

		// Calculate power output (P + D only, no I term)
		double power = feedForward + pTerm + dTerm;

		// Apply minimum power threshold to overcome static friction
		if (Math.abs(power) < MIN_POWER && Math.abs(power) > 0) {
			power = Math.copySign(MIN_POWER, power);
		}

		power = Range.clip(power, -1.0, 1.0);

		return power;
	}

	private void resetPidState() {
		pidLastTimeMs = 0;
		pidPrevError = 0.0;
	}


	@Override
	public void stop() {
		if (leftFront != null) {
			leftFront.setPower(0.0);
		}
	}
}
