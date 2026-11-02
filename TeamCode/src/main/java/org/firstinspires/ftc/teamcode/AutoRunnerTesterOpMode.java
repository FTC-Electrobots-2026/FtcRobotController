package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;

/**
 * Simple autonomous tester for AutoRunner command dispatch.
 */
@Autonomous(name = "AutoRunner Tester", group = "Robot")
public class AutoRunnerTesterOpMode extends LinearOpMode {

    private static final double TEST_TIMEOUT_SECONDS = 30.0;

    private DcMotor frontLeftDrive;
    private DcMotor frontRightDrive;
    private DcMotor backLeftDrive;
    private DcMotor backRightDrive;
    private IMU imu;

    private SnapMecanumMover snapMecanumMover;
    private AutoRunner autoRunner;

    @Override
    public void runOpMode() {
        initDriveHardware();

        snapMecanumMover = new SnapMecanumMover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );

        autoRunner = new AutoRunner(snapMecanumMover)
                .MoveForward(2.0)
                .MoveBackward(1)
                .MoveLeft(0.5)
                .MoveRight(0.5)
                .Turn(-90.0)
                .Turn(90.0);

        telemetry.addLine("AutoRunner tester ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobotMotion();
            return;
        }

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && !isStopRequested() && timer.seconds() <= TEST_TIMEOUT_SECONDS) {
            boolean done = autoRunner.update();

            MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();
            telemetry.addData("Done", done ? "YES" : "NO");
            telemetry.addData("Pending", autoRunner.getPendingCommandCount());
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f", pose.xInches, pose.yInches, Math.toDegrees(pose.headingRadians));
            telemetry.addData("Time", "%.1f / %.1f", timer.seconds(), TEST_TIMEOUT_SECONDS);
            telemetry.update();

            if (done) {
                break;
            }

            idle();
        }

        autoRunner.clear();
        stopRobotMotion();
    }

    private void initDriveHardware() {
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    private void stopRobotMotion() {
        if (snapMecanumMover != null) {
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            snapMecanumMover.cancelTarget();
        }
    }
}

