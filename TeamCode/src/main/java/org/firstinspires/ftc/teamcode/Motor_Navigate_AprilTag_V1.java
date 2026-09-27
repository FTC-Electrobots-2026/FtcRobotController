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
import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
@TeleOp(name = "Motor_Navigate_AprilTag_V1", group = "TeleOp")
public class Motor_Navigate_AprilTag_V1  extends OpMode{
        private static final double JOYSTICK_DEADBAND = 0.08;
        private AprilTagWebcam RedTagReader;


        private DcMotor leftFront;

        @Override
        public void init() {
            leftFront = hardwareMap.get(DcMotor.class, "leftFront");
            leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            leftFront.setPower(0.0);

            RedTagReader = new AprilTagWebcam();
            RedTagReader.init(hardwareMap, telemetry);
            telemetry.addData("Status", "Initialized");
            telemetry.addData("Motor", "leftFront ready");
            telemetry.addData("Deadband", JOYSTICK_DEADBAND);
            telemetry.addData("IMU Mount", "Logo UP, USB FORWARD");
            telemetry.update();
        }

        @Override
        public void loop() {
            double leftStickY = -gamepad1.left_stick_y*0.5;
            double x_value;
            double motorPower = 0;
            double DEADBAND_THRESHOLD_APRILTAG = 10;
            double DEADBAND_THRESHOLD_JOYSTICK = 0.08;
            RedTagReader.updateDetections();
            if(RedTagReader.tagDetected)
            {
                x_value = RedTagReader.x_value;
                motorPower = applyDeadband(x_value, DEADBAND_THRESHOLD_APRILTAG);
                motorPower = Range.clip(motorPower, -1.0, 1.0);
                leftFront.setPower(motorPower);

            }
            else {
                motorPower = applyDeadband(leftStickY, DEADBAND_THRESHOLD_JOYSTICK);
                motorPower = Range.clip(motorPower, -1.0, 1.0);
                leftFront.setPower(motorPower);
            }
            telemetry.addData("Status", "Running");
            telemetry.addData("Deadband", DEADBAND_THRESHOLD_JOYSTICK);
            telemetry.addData("Left Stick Y (inverted)", "%.3f", leftStickY);
            telemetry.addData("Motor Power", "%.3f", motorPower);

            telemetry.update();
        }

        private double applyDeadband(double value, double deadbandThreshold) {
            if (Math.abs(value) <= deadbandThreshold) {
                return 0.0;
            }

            return value;
        }

}
/*
        @Override
        public void stop() {
            if (leftFront != null) {
                leftFront.setPower(0.0);
            }
        }
  */


