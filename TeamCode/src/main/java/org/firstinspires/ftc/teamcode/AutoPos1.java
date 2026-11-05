package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;

/**
 * AutoPos1:
 * 1) Move forward by 0.5 wheel rotations.
 * 2) Strafe field-east by 3.0 wheel rotations.
 *
 * Uses existing AutoRunner + SnapMecanumMover libraries without rewriting drive logic.
 */
@Autonomous(name = "AutoPos1", group = "Robot", preselectTeleOp = "Teleop-Scrimmage_v1")
public class AutoPos1 extends LinearOpMode {

    private static final double AUTO_TIMEOUT_SECONDS = 20.0;

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
                .MoveForward(0.25)
                .MoveRight(1.5);

        telemetry.addLine("AutoPos1 ready");
        telemetry.addData("Plan", "Forward 0.25 rot, East 1.0 rot");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            stopRobotMotion();
            return;
        }

        // Define field north at autonomous start and preserve it for auto->teleop flow.
        imu.resetYaw();
        Electrobots_biobuzz_scrimmage_master.markFieldNorthFromAuto();

        long startNanos = System.nanoTime();

        while (opModeIsActive() && !isStopRequested()) {
            boolean done = autoRunner.update();
            double elapsedSeconds = (System.nanoTime() - startNanos) / 1_000_000_000.0;

            MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();
            telemetry.addData("Done", done ? "YES" : "NO");
            telemetry.addData("Pending", autoRunner.getPendingCommandCount());
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f", pose.xInches, pose.yInches, Math.toDegrees(pose.headingRadians));
            telemetry.addData("Time", "%.1f / %.1f", elapsedSeconds, AUTO_TIMEOUT_SECONDS);
            telemetry.update();

            if (done || elapsedSeconds >= AUTO_TIMEOUT_SECONDS) {
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

