package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;

import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.drivesupport.Robot_Placement;
import org.firstinspires.ftc.teamcode.drivesupport.LightIndicator;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;
import org.firstinspires.ftc.teamcode.scoring.ScoringConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class Electrobots_biobuzz_scrimmage_master extends OpMode {
    // Four drive motors for a mecanum drivetrain.
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;

    // IMU gives heading/orientation so field-relative driving can work.
    IMU imu;

    // Helper that converts joystick inputs into motor powers.
    private SnapMecanumMover snapMecanumMover;
    // Helper that detects AprilTags and stores values like x offset and confidence/percent.
    private AprilTagWebcam aprilTagWebcam;

    // Helper that decides which target AprilTag ID we care about.
    private Robot_Placement robotPlacement;

    // Driver/starting-side profiles used by Robot_Placement.
    private static final int RED_1 = 1;
    private static final int RED_2 = 2;
    private static final int BLUE_1 = 3;
    private static final int BLUE_2 = 4;

    private IntakeController intakeController;
    private ShooterController shooterController;
    private boolean intakeRoutineActive;
    private boolean shooterRoutineActive;

    // Light helper for driver-facing state feedback.
    private LightIndicator lightIndicator;

    /*
     * What this method does:
     * - Uses camera numbers to move left/right until the AprilTag is centered.
     *
     * Values it reads:
     * - aprilTagWebcam.percent_value: how "good"/complete the detection is.
     *   Smaller than 100 here means we still need to adjust position.
     * - aprilTagWebcam.x_value: horizontal error from center.
     *   < 0 means tag appears left, > 0 means tag appears right, 0 means centered.
     *
     * Parameters passed to driveFieldRelative(forward, strafe, turn):
     * - forward = 0.0 (do not move forward/back)
     * - strafe  = +/-0.2 (slide left/right slowly)
     * - turn    = 0.0 (do not rotate)
     *
     * Outcome:
     * - Robot nudges sideways to line the tag up in camera view.
     */
    public void snap_to_tag() {
        if (aprilTagWebcam.percent_value < 100) {
            if (aprilTagWebcam.x_value < 0) {
                snapMecanumMover.driveFieldRelative(0.0, -0.2, 0.0); // Tag is left, so strafe left.
            } else if (aprilTagWebcam.x_value > 0) {
                snapMecanumMover.driveFieldRelative(0.0, 0.2, 0.0); // Tag is right, so strafe right.
            } else {
                snapMecanumMover.driveFieldRelative(0.0, 0.0, 0.0); // Centered: stop sideways correction.
            }
            // aprilTagWebcam.displayDetections(telemetry); // Useful for debugging camera values on screen.
        }
    }

    /**
     * Adds dedicated driver telemetry while snap-to-tag is active.
     *
     * Data is fetched from AprilTagWebcam as a ready-to-render snapshot so
     * detection parsing and type decisions stay inside the camera helper.
     */
    private void addSnapToTagTelemetry() {
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        telemetry.addLine("=== SNAP TO TAG ===");
        telemetry.addData("Visible tags", snapTelemetry.visibleTagCount);

        if (!snapTelemetry.hasCluster) {
            telemetry.addData("Selected cluster", "NONE");
            return;
        }

        telemetry.addData("Selected cluster", snapTelemetry.clusterName);
        telemetry.addData("Cluster percent", "%.0f", snapTelemetry.percentClusterFound);
        telemetry.addData("Tag XYZ (in)", "%.1f, %.1f, %.1f",
                snapTelemetry.xInches,
                snapTelemetry.yInches,
                snapTelemetry.zInches);
        telemetry.addData("Tag PRY (deg)", "%.1f, %.1f, %.1f",
                snapTelemetry.pitchDegrees,
                snapTelemetry.rollDegrees,
                snapTelemetry.yawDegrees);
        telemetry.addData("Tag RBE", "%.1f in, %.1f deg, %.1f deg",
                snapTelemetry.rangeInches,
                snapTelemetry.bearingDegrees,
                snapTelemetry.elevationDegrees);
    }

    /*
     * FTC calls init() once when you press INIT on Driver Station.
     *
     * Job of this method:
     * 1) Connect Java variables to real hardware using names from Robot Configuration.
     * 2) Set motor directions/modes.
     * 3) Set IMU orientation so heading math is correct.
     * 4) Build helper classes and tell camera which AprilTag to track.
     */
    @Override
    public void init() {
        // hardwareMap.get(Type, "name") links code to configured hardware devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Left motors are reversed because they are mirrored physically on many drivetrains.
        // Without this, one side can fight the other side.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        // RUN_USING_ENCODER lets the motor controller use encoder feedback for steadier speed control.
        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        imu = hardwareMap.get(IMU.class, "imu");

        // These two lines describe how the Control/Expansion Hub is mounted on the robot.
        // If these are wrong, field-relative driving will feel "rotated" or incorrect.
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new
                RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot)); // Apply mounting info to IMU.

        // Build driving helper with all 4 motors + imu.
        snapMecanumMover = new SnapMecanumMover(
                frontLeftDrive,
                frontRightDrive,
                backLeftDrive,
                backRightDrive,
                imu
        );

        // Choose team/start profile, then it sets the target AprilTag.
        // Keep profile in a variable so we can reuse it for indicator color selection.
        int placementProfile = RED_1; // Change this if your side/starting position changes.

        robotPlacement = new Robot_Placement();
        robotPlacement.init(placementProfile);

        // Start camera/AprilTag system and pass in which tag we want to track.
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry, robotPlacement.target_tag_1, robotPlacement.target_tag_2);

        intakeController = new IntakeController();
        intakeController.init(hardwareMap);

        shooterController = new ShooterController();
        shooterController.init(hardwareMap);

        // Light setup:
        // - hardware name expected in Robot Config: "ftc_light"
        // - placement profile selects default team color
        lightIndicator = new LightIndicator();
        lightIndicator.init(hardwareMap);

        LightIndicator.LightColor placementColor =
                (placementProfile == RED_1 || placementProfile == RED_2)
                        ? LightIndicator.RED
                        : LightIndicator.BLUE;
        lightIndicator.state(true, placementColor); // Turn on using placement profile color.

        intakeRoutineActive = false;
        shooterRoutineActive = false;
    }

    /*
     * FTC calls loop() over and over while the OpMode is running.
     *
     * Flow each cycle:
     * 1) Update camera detections.
     * 2) Decide if driver requested "snap" mode (gamepad2 right bumper) and tag is visible.
     * 3) Run normal drive logic.
     * 4) If snap mode is active, add sideways auto-correction.
     */
    @Override
    public void loop() {
        boolean tagDetected = aprilTagWebcam.updateDetections();
        // Must be true for BOTH conditions:
        // - driver is holding right bumper
        // - camera currently sees the target tag
        boolean driver_snapping = gamepad2.right_bumper && tagDetected;

        // Passes both controllers and telemetry into the drive helper.
        // Last parameter tells helper whether auto-snap mode is active.
        snapMecanumMover.loop(gamepad1, gamepad2, telemetry, driver_snapping);

        if (driver_snapping) {
            snap_to_tag(); // Extra correction only when snap mode is requested and tag is detected.
            addSnapToTagTelemetry();
        }

        boolean intakeRequested = gamepad2.left_trigger > ScoringConfig.TRIGGER_THRESHOLD;
        boolean shooterRequested = gamepad2.right_trigger > ScoringConfig.TRIGGER_THRESHOLD;
        boolean windmillRequested = shooterRequested && gamepad2.a;
        boolean shooterTuningRequested = gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.left_bumper;
        double shooterYawAdjustRpm = 0.0;

        if (windmillRequested) {
            double yawDegrees = Math.abs(imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            if (yawDegrees > ScoringConfig.SHOOTER_YAW_ADJUST_THRESHOLD_DEGREES) {
                shooterYawAdjustRpm = ScoringConfig.SHOOTER_YAW_ADJUSTER;
            }
        }

        if (intakeRequested || windmillRequested) {
            intakeController.applyIntakeAndWindmillControl(intakeRequested, windmillRequested);
            intakeRoutineActive = true;
        } else if (intakeRoutineActive) {
            intakeController.stop();
            intakeRoutineActive = false;
        }

        if (shooterRequested || shooterTuningRequested) {
            shooterController.updateTargetRpmFromDriverButtons(gamepad2);
            shooterController.applyShooterSpeedControl(shooterRequested, shooterYawAdjustRpm);
            shooterRoutineActive = true;

            if (shooterRequested) {
                double shooterRpm = shooterController.getLastRpm();
                double shooterTargetRpm = shooterController.getTargetRpm();
                double shooterPower = shooterController.getLastPower();
                double rpmError = shooterTargetRpm - shooterRpm;
                boolean shooterPowerSaturated = Math.abs(shooterPower) >= 0.999;

                telemetry.addData("Shooter RPM", "%.1f", shooterRpm);
                telemetry.addData("Shooter Target RPM", "%.1f", shooterTargetRpm);
                telemetry.addData("Shooter Baseline RPM", "%.1f", shooterController.getBaselineRpm());
                telemetry.addData("Shooter Power", "%.3f", shooterPower);
                telemetry.addData("Shooter Power Saturated", shooterPowerSaturated ? "YES" : "NO");
                telemetry.addData("Shooter RPM Error", "%.1f", rpmError);
                telemetry.addData("Shooter Estimated Max Seen RPM", "%.1f", shooterController.getMaxSeenRpm());
                telemetry.addData("Shooter Ready", shooterController.isReady() ? "YES" : "NO");
                telemetry.addData("Intake Active", intakeRequested);
                telemetry.addData("Windmill Active", windmillRequested);
            }
        } else if (shooterRoutineActive) {
            shooterController.stop();
            shooterRoutineActive = false;
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        if (intakeController != null) {
            intakeController.stop();
        }

        if (shooterController != null) {
            shooterController.stop();
        }

        if (lightIndicator != null) {
            lightIndicator.state(false);
        }

        intakeRoutineActive = false;
        shooterRoutineActive = false;
    }
}
