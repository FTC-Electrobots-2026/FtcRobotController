package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;

/**
 * AutoPos2 sequence:
 * 1) Execute right-trigger snap turn and wait for completion.
 * 2) Spin flywheel to target speed and wait until ready.
 * 3) Run windmill feed routine 8 times.
 *
 * Uses existing routines/controllers only.
 */
@Autonomous(name = "AutoPos2", group = "Robot")
public class AutoPos2 extends LinearOpMode {

    private static final double SNAP_TURN_TIMEOUT_SECONDS = 5.0;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 4.0;
    private static final double WINDMILL_ON_SECONDS = 0.35;
    private static final double WINDMILL_OFF_SECONDS = 0.15;
    private static final int WINDMILL_CYCLES = 8;

    @Override
    public void runOpMode() {
        DcMotor frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        IMU imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        SnapMecanumMover snapMecanumMover = new SnapMecanumMover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );

        ShooterController shooterController = new ShooterController();
        shooterController.init(hardwareMap);

        IntakeController intakeController = new IntakeController();
        intakeController.init(hardwareMap);

        telemetry.addLine("AutoPos2 ready");
        telemetry.addLine("1) Snap turn right-trigger target");
        telemetry.addLine("2) Spin flywheel to target");
        telemetry.addData("3) Windmill cycles", WINDMILL_CYCLES);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            shooterController.stop();
            intakeController.stop();
            snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
            snapMecanumMover.cancelTarget();
            return;
        }

        // 1) Same behavior as driver right-trigger snap turn, then wait until it settles.
        snapMecanumMover.snapTurnRightTriggerShortcut();
        ElapsedTime snapTimer = new ElapsedTime();
        snapTimer.reset();

        while (opModeIsActive() && !isStopRequested() && snapTimer.seconds() <= SNAP_TURN_TIMEOUT_SECONDS) {
            boolean turnDone = snapMecanumMover.stepSnapTurn(telemetry);
            telemetry.addData("Step", "Snap turn");
            telemetry.addData("Turn done", turnDone ? "YES" : "NO");
            telemetry.addData("Time", "%.1f / %.1f", snapTimer.seconds(), SNAP_TURN_TIMEOUT_SECONDS);
            telemetry.update();

            if (turnDone) {
                break;
            }
            idle();
        }

        // 2) Spin flywheel with existing closed-loop controller until ready (or timeout).
        ElapsedTime spinupTimer = new ElapsedTime();
        spinupTimer.reset();
        while (opModeIsActive() && !isStopRequested() && spinupTimer.seconds() <= SHOOTER_SPINUP_TIMEOUT_SECONDS) {
            shooterController.applyShooterSpeedControl(true, 0.0);
            boolean ready = shooterController.isReady();

            telemetry.addData("Step", "Flywheel spinup");
            telemetry.addData("RPM", "%.1f", shooterController.getLastRpm());
            telemetry.addData("Target RPM", "%.1f", shooterController.getTargetRpm());
            telemetry.addData("Ready", ready ? "YES" : "NO");
            telemetry.addData("Time", "%.1f / %.1f", spinupTimer.seconds(), SHOOTER_SPINUP_TIMEOUT_SECONDS);
            telemetry.update();

            if (ready) {
                break;
            }
            idle();
        }

        // 3) Run the windmill routine 8 times using existing intake/shooter control methods.
        for (int cycle = 1; cycle <= WINDMILL_CYCLES && opModeIsActive() && !isStopRequested(); cycle++) {
            ElapsedTime onTimer = new ElapsedTime();
            onTimer.reset();
            while (opModeIsActive() && !isStopRequested() && onTimer.seconds() <= WINDMILL_ON_SECONDS) {
                shooterController.applyShooterSpeedControl(true, 0.0);
                intakeController.applyIntakeAndWindmillControl(false, false, true);

                telemetry.addData("Step", "Windmill ON");
                telemetry.addData("Cycle", "%d / %d", cycle, WINDMILL_CYCLES);
                telemetry.addData("RPM", "%.1f", shooterController.getLastRpm());
                telemetry.update();
                idle();
            }

            ElapsedTime offTimer = new ElapsedTime();
            offTimer.reset();
            while (opModeIsActive() && !isStopRequested() && offTimer.seconds() <= WINDMILL_OFF_SECONDS) {
                shooterController.applyShooterSpeedControl(true, 0.0);
                intakeController.applyIntakeAndWindmillControl(false, false, false);

                telemetry.addData("Step", "Windmill OFF");
                telemetry.addData("Cycle", "%d / %d", cycle, WINDMILL_CYCLES);
                telemetry.addData("RPM", "%.1f", shooterController.getLastRpm());
                telemetry.update();
                idle();
            }
        }

        intakeController.stop();
        shooterController.stop();
        snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
        snapMecanumMover.cancelTarget();
    }
}

