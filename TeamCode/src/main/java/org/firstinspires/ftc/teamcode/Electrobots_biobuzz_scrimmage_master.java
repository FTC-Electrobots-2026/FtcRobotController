package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import org.firstinspires.ftc.teamcode.navigate.SnapToAprilTagHelper;

import org.firstinspires.ftc.teamcode.drivesupport.Robot_Placement;
import org.firstinspires.ftc.teamcode.drivesupport.LightIndicator;
import org.firstinspires.ftc.teamcode.scoring.IntakeController;
import org.firstinspires.ftc.teamcode.scoring.ShooterController;
import org.firstinspires.ftc.teamcode.scoring.ScoringConfig;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Teleop-Scrimmage_v1", group = "Scrimmage")

public class Electrobots_biobuzz_scrimmage_master extends OpMode {
    private static final double SCORABLE_ROLL_ABS_MAX_DEG = 90.0;
    private static boolean preserveFieldNorthFromAuto;

    public static void markFieldNorthFromAuto() {
        preserveFieldNorthFromAuto = true;
    }

    private static boolean consumeFieldNorthLatch() {
        boolean preserve = preserveFieldNorthFromAuto;
        preserveFieldNorthFromAuto = false;
        return preserve;
    }

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

    // true: run full scoring-bin recovery flow (turn + lateral move + re-snap).
    // false: legacy mode (no east/west move), with side-based yaw snap assist only.
    private static final boolean SNAP_TO_SCORING_BIN = true;

    private IntakeController intakeController;
    private ShooterController shooterController;
    private boolean intakeRoutineActive;
    private boolean shooterRoutineActive;

    // Light helper for driver-facing state feedback.
    private LightIndicator lightIndicator;
    /////**************change only this*********////
    private int placementProfile = BLUE_1;
    /////**************change only this*********////
    private boolean previousSnapRequest;

    private SnapToAprilTagHelper snapToAprilTagHelper;


    /**
     * Snap light policy:
     * - Target out of frame or wrong color target: light OFF
     * - Intended target visible: BLUE
     * - Intended target visible on scoring side: GREEN
     */
    private void applySnapToTagLight(AprilTagWebcam.SnapTelemetry snapTelemetry) {
        if (snapTelemetry == null
                || !snapTelemetry.hasCluster
                || !snapToAprilTagHelper.isPlacementColorMatch(placementProfile, snapTelemetry.clusterName)) {
            lightIndicator.state(false);
            return;
        }

        boolean scoringBinFound = Math.abs(snapTelemetry.rollDegrees) < SCORABLE_ROLL_ABS_MAX_DEG;

        if (scoringBinFound) {
            lightIndicator.state(true, LightIndicator.GREEN);
        } else {
            lightIndicator.state(true, LightIndicator.BLUE);
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

        // Match base starterbot mecanum direction profile.
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

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
//        placementProfile = RED_1; // Change this if your side/starting position changes.

        robotPlacement = new Robot_Placement();
        robotPlacement.init(placementProfile);

        // Start camera/AprilTag system and pass in which tag we want to track.
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry, robotPlacement.target_tag_1, robotPlacement.target_tag_2);
        snapToAprilTagHelper = new SnapToAprilTagHelper();

        intakeController = new IntakeController();
        intakeController.init(hardwareMap);

        shooterController = new ShooterController();
        shooterController.init(hardwareMap);

        // Needed for short distance shifts during assisted snap recovery.
        snapMecanumMover.initializeOdometry();

        // Light setup:
        // - hardware name expected in Robot Config: "indicator" (legacy "ftc_light" also supported)
        // - placement profile selects default team color
        lightIndicator = new LightIndicator();
        lightIndicator.init(hardwareMap);

        // Keep startup light off; snap/shooter logic will command explicit states.
        lightIndicator.state(false);

        intakeRoutineActive = false;
        shooterRoutineActive = false;
    }

    @Override
    public void start() {
        boolean keepAutoFieldNorth = consumeFieldNorthLatch();
        if (!keepAutoFieldNorth) {
            // Direct teleop entry: define field north from current heading.
            imu.resetYaw();
        }
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
        aprilTagWebcam.updateDetections();
        AprilTagWebcam.SnapTelemetry snapTelemetry = aprilTagWebcam.getSnapTelemetry();
        // Force baseline OFF each cycle; later logic enables only allowed states.
        lightIndicator.state(false);
        boolean keepFlywheelReadyLight = false;
        // Must be true for BOTH conditions:
        // - driver is holding right bumper
        // - camera currently sees any tag
        boolean driverSnappingRequested = gamepad2.right_bumper;
        boolean driver_snapping = driverSnappingRequested && aprilTagWebcam.hasActiveTags();
        boolean snapRequestEdge = driverSnappingRequested && !previousSnapRequest;
        previousSnapRequest = driverSnappingRequested;
        //telemetry.addData("tag detected value:", tagDetected);

        // Passes both controllers and telemetry into the drive helper.
        // Last parameter tells helper whether auto-snap mode is active.
        // Pass A / A.1 refactor note: call contract is intentionally unchanged.
        boolean legacyYawSnapActive = !SNAP_TO_SCORING_BIN && snapMecanumMover.isSnapTurnActive();
        boolean snapRoutineActive = driver_snapping || legacyYawSnapActive;
        snapMecanumMover.loop(gamepad1, gamepad2, telemetry, snapRoutineActive);

        if (driver_snapping) {
            if (SNAP_TO_SCORING_BIN) {
                snapToAprilTagHelper.applyScoringBinSnap(snapMecanumMover, snapTelemetry, placementProfile);
            } else {
                snapToAprilTagHelper.runLegacySnapMode(
                        snapMecanumMover,
                        telemetry,
                        snapTelemetry,
                        snapRequestEdge,
                        placementProfile
                );
            }
            addSnapToTagTelemetry();
        } else if (!SNAP_TO_SCORING_BIN && snapMecanumMover.isSnapTurnActive()) {
            // Finish legacy yaw snap cleanly even if driver released the button mid-turn.
            snapMecanumMover.stepSnapTurn(telemetry);
        }

        boolean intakeRequested = gamepad2.left_trigger > ScoringConfig.TRIGGER_THRESHOLD;
        boolean intakeReverseRequested = intakeRequested && gamepad2.x;
        boolean feederRequested = gamepad2.b
                || (ScoringConfig.intake_side_and_stratight_combined && intakeRequested);
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

        if (intakeRequested || feederRequested || windmillRequested) {
            intakeController.applyIntakeAndWindmillControl(
                    intakeRequested,
                    feederRequested,
                    windmillRequested,
                    intakeReverseRequested
            );
            intakeRoutineActive = true;
        } else if (intakeRoutineActive) {
            intakeController.stop();
            intakeRoutineActive = false;
        }

        if (shooterRequested || shooterTuningRequested) {
            shooterController.updateTargetRpmFromDriverButtons(gamepad2);
            shooterController.applyShooterSpeedControl(shooterRequested, shooterYawAdjustRpm);
            shooterRoutineActive = true;
            keepFlywheelReadyLight = shooterRequested && shooterController.isReady();

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

        if (keepFlywheelReadyLight) {
            // RPM ready indicator is the only snap-light override.
            lightIndicator.state(true, LightIndicator.RED);
        } else {
            // Default light behavior follows alliance AprilTag state continuously.
            applySnapToTagLight(aprilTagWebcam.getSnapTelemetry());
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
