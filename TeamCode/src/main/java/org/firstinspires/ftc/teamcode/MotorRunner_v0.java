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

@TeleOp(name = "MotorRunner_v0", group = "TeleOp")
public class MotorRunner_v0 extends OpMode {

	private static final double JOYSTICK_DEADBAND = 0.08;

	private DcMotor leftFront;
	private IMU imu;
	private long lastImuLogTimeMs;

	@Override
	public void init() {
		leftFront = hardwareMap.get(DcMotor.class, "zz");
		leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		leftFront.setPower(0.0);


	}

	@Override
	public void loop() {
		leftFront.setPower(1);
	}
}
