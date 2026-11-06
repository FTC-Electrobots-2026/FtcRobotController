package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.auto.AutoRunner;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;

/**
 * AutoPos2 sequence:
 * 1) Move left 0.25 rotations.
 * 2) Execute right-trigger snap turn and wait for completion.
 * 3) Move right 0.25 rotations.
 * 4) Spin flywheel to target speed and wait until ready.
 * 5) Run windmill feed routine 8 times.
 *
 * Uses existing routines/controllers only.
 */
@Autonomous(name = "AutoPos2", group = "Robot", preselectTeleOp = "Teleop-Scrimmage_v1")
public class AutoPos2 extends LinearOpMode {

    private static final double SNAP_TURN_TIMEOUT_SECONDS = 5.0;
    private static final double PRE_TURN_MOVE_TIMEOUT_SECONDS = 3.0;
    private static final double SHOOTER_SPINUP_TIMEOUT_SECONDS = 4.0;
    private static final double SHOOTER_SETTLE_BEFORE_WINDMILL_SECONDS = 1.0;
    private static final double WINDMILL_ON_SECONDS = 0.35;
    private static final double WINDMILL_OFF_SECONDS = 0.15;
    private static final double AUTO_WINDMILL_POWER = 0.6;
    private static final int WINDMILL_CYCLES = 10;
    private static final double POST_WINDMILL_MOVE_TIMEOUT_SECONDS = 15.0;

    // Tile convention for AutoPos2 only:
    // - Southwest (bottom/left) tile is 1A.
    // - Row increases south->north: 1..6.
    // - Column increases west->east: A..F.
    private static final String START_TILE = "1A";
    private static final double START_HEADING_DEGREES = 0.0;

    private static final int FIELD_TILES_PER_SIDE = 6;
    private static final double TILE_SIZE_INCHES = MecanumWheelMotorOdometry.FIELD_SIZE_INCHES / FIELD_TILES_PER_SIDE;

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
        CRServo windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");

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

        // Define field north at autonomous start and preserve it for auto->teleop flow.
        imu.resetYaw();
        Electrobots_biobuzz_scrimmage_master.markFieldNorthFromAuto();
        applyStartTilePose(snapMecanumMover, START_TILE, START_HEADING_DEGREES);

        // 1) Move left 0.25 rotations before starting snap turn.
        AutoRunner preTurnMove = new AutoRunner(snapMecanumMover)
                .MoveLeft(0.5);

        ElapsedTime preTurnTimer = new ElapsedTime();
        preTurnTimer.reset();
        while (opModeIsActive() && !isStopRequested() && preTurnTimer.seconds() <= PRE_TURN_MOVE_TIMEOUT_SECONDS) {
            boolean done = preTurnMove.update();

            telemetry.addData("Step", "Pre-turn move left 0.25");
            telemetry.addData("Done", done ? "YES" : "NO");
            telemetry.addData("Pending", preTurnMove.getPendingCommandCount());
            telemetry.addData("Time", "%.1f / %.1f", preTurnTimer.seconds(), PRE_TURN_MOVE_TIMEOUT_SECONDS);
            telemetry.update();

            if (done) {
                break;
            }
            idle();
        }
        preTurnMove.clear();

        // 2) Same behavior as driver right-trigger snap turn, then wait until it settles.
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

        // 3) After turn, move right 0.25 rotations before flywheel spin-up.
        AutoRunner postTurnMove = new AutoRunner(snapMecanumMover)
                .MoveRight(0.5);

        ElapsedTime postTurnTimer = new ElapsedTime();
        postTurnTimer.reset();
        while (opModeIsActive() && !isStopRequested() && postTurnTimer.seconds() <= PRE_TURN_MOVE_TIMEOUT_SECONDS) {
            boolean done = postTurnMove.update();

            telemetry.addData("Step", "Post-turn move right 0.25");
            telemetry.addData("Done", done ? "YES" : "NO");
            telemetry.addData("Pending", postTurnMove.getPendingCommandCount());
            telemetry.addData("Time", "%.1f / %.1f", postTurnTimer.seconds(), PRE_TURN_MOVE_TIMEOUT_SECONDS);
            telemetry.update();

            if (done) {
                break;
            }
            idle();
        }
        postTurnMove.clear();

        // 4) Spin flywheel with existing closed-loop controller until ready (or timeout).
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

        // Keep flywheel under closed-loop control for 1 second before feeding.
        ElapsedTime settleTimer = new ElapsedTime();
        settleTimer.reset();
        while (opModeIsActive() && !isStopRequested() && settleTimer.seconds() <= SHOOTER_SETTLE_BEFORE_WINDMILL_SECONDS) {
            shooterController.applyShooterSpeedControl(true, 0.0);
            intakeController.applyIntakeAndWindmillControl(false, false, false);

            telemetry.addData("Step", "Flywheel settle");
            telemetry.addData("RPM", "%.1f", shooterController.getLastRpm());
            telemetry.addData("Target RPM", "%.1f", shooterController.getTargetRpm());
            telemetry.addData("Time", "%.1f / %.1f", settleTimer.seconds(), SHOOTER_SETTLE_BEFORE_WINDMILL_SECONDS);
            telemetry.update();
            idle();
        }

        // 5) Run the windmill routine 8 times using existing intake/shooter control methods.
        for (int cycle = 1; cycle <= WINDMILL_CYCLES && opModeIsActive() && !isStopRequested(); cycle++) {
            ElapsedTime onTimer = new ElapsedTime();
            onTimer.reset();
            while (opModeIsActive() && !isStopRequested() && onTimer.seconds() <= WINDMILL_ON_SECONDS) {
                shooterController.applyShooterSpeedControl(true, 0.0);
                intakeController.applyIntakeAndWindmillControl(false, false, false);
                // Auto-only override: run windmill slower than normal TeleOp/scoring config.
                windmillServo.setPower(-AUTO_WINDMILL_POWER);

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
                windmillServo.setPower(0.0);

                telemetry.addData("Step", "Windmill OFF");
                telemetry.addData("Cycle", "%d / %d", cycle, WINDMILL_CYCLES);
                telemetry.addData("RPM", "%.1f", shooterController.getLastRpm());
                telemetry.update();
                idle();
            }
        }

        // 6) After windmill cycles, move backward by 2.5 wheel rotations using existing auto command flow.
        AutoRunner autoRunner = new AutoRunner(snapMecanumMover)
                .MoveBackward(3.4)
                .MoveLeft(8.0);


        ElapsedTime moveTimer = new ElapsedTime();
        moveTimer.reset();
        while (opModeIsActive() && !isStopRequested() && moveTimer.seconds() <= POST_WINDMILL_MOVE_TIMEOUT_SECONDS) {
            //shooterController.applyShooterSpeedControl(true, 0.0);
            //intakeController.applyIntakeAndWindmillControl(false, false, false);

            boolean done = autoRunner.update();
            MecanumWheelMotorOdometry.Pose2d pose = snapMecanumMover.getOdometry().getPose();

            telemetry.addData("Step", "Move back 2.5 rotations");
            telemetry.addData("Done", done ? "YES" : "NO");
            telemetry.addData("Pending", autoRunner.getPendingCommandCount());
            telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f", pose.xInches, pose.yInches, Math.toDegrees(pose.headingRadians));
            telemetry.addData("Time", "%.1f / %.1f", moveTimer.seconds(), POST_WINDMILL_MOVE_TIMEOUT_SECONDS);
            telemetry.update();

            if (done) {
                break;
            }
            idle();
        }

        autoRunner.clear();
        windmillServo.setPower(0.0);

        intakeController.stop();
        shooterController.stop();
        snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0);
        snapMecanumMover.cancelTarget();
    }

    private void applyStartTilePose(SnapMecanumMover snapMecanumMover, String tileLabel, double headingDegrees) {
        snapMecanumMover.initializeOdometry();

        TileCoordinate tile = parseTileLabel(tileLabel);
        double xInches = (tile.columnIndex + 0.5) * TILE_SIZE_INCHES;
        double yInches = (tile.rowIndex + 0.5) * TILE_SIZE_INCHES;

        snapMecanumMover.getOdometry().resetPose(xInches, yInches, Math.toRadians(headingDegrees));
    }

    private TileCoordinate parseTileLabel(String tileLabel) {
        if (tileLabel == null || tileLabel.length() < 2) {
            throw new IllegalArgumentException("Tile label must look like 1A through 6F.");
        }

        char rowChar = tileLabel.charAt(0);
        char columnChar = Character.toUpperCase(tileLabel.charAt(1));

        if (rowChar < '1' || rowChar > '6' || columnChar < 'A' || columnChar > 'F') {
            throw new IllegalArgumentException("Tile label must be in range 1A..6F.");
        }

        int rowIndex = (rowChar - '1');
        int columnIndex = (columnChar - 'A');
        return new TileCoordinate(rowIndex, columnIndex);
    }

    private static class TileCoordinate {
        final int rowIndex;
        final int columnIndex;

        TileCoordinate(int rowIndex, int columnIndex) {
            this.rowIndex = rowIndex;
            this.columnIndex = columnIndex;
        }
    }
}

