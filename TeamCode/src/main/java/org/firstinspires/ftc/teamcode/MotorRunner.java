package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MotorRunner", group = "TeleOp")
public class MotorRunner extends OpMode {

	private static final double JOYSTICK_DEADBAND = 0.08;

	private DcMotor leftFront;
	private IMU imu;
	private long lastImuLogTimeMs;

	@Override
	public void init() {
		leftFront = hardwareMap.get(DcMotor.class, "leftFront");
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

		telemetry.addData("Status", "Initialized");
		telemetry.addData("Motor", "leftFront ready");
		telemetry.addData("Deadband", JOYSTICK_DEADBAND);
		telemetry.addData("IMU Mount", "Logo UP, USB FORWARD");
		telemetry.update();
	}

	@Override
	public void loop() {
		double leftStickY = -gamepad1.left_stick_y*0.5;
		double motorPower = applyDeadband(leftStickY);
		motorPower = Range.clip(motorPower, -1.0, 1.0);

		leftFront.setPower(motorPower);

		telemetry.addData("Status", "Running");
		telemetry.addData("Deadband", JOYSTICK_DEADBAND);
		telemetry.addData("Left Stick Y (inverted)", "%.3f", leftStickY);
		telemetry.addData("Motor Power", "%.3f", motorPower);

		showImuInfo();
		telemetry.update();
	}

	private double applyDeadband(double value) {
		if (Math.abs(value) <= JOYSTICK_DEADBAND) {
			return 0.0;
		}

		return value;
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
