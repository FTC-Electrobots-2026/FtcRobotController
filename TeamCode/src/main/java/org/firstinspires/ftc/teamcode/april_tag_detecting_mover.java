package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.navigate.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.drivesupport.Robot_Placement;
/*
 * Big picture of this OpMode:
 * - It lets a driver control a mecanum robot.
 * - If the second driver holds right bumper, the robot tries to line up ("snap") to an AprilTag.
 * - It uses camera data (AprilTagWebcam) + drive math helper (SnapMecanumMover).
 *
 * Where names come from:
 * - Motor names like "front_left_drive" must match exactly in the Robot Configuration on the phone.
 * - IMU name "imu" must also match hardware config.
 */
@TeleOp(name = "april_tag_detecting_mover", group = "Robot")

public class april_tag_detecting_mover extends OpMode {
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
        robotPlacement = new Robot_Placement();
        robotPlacement.init(RED_1); // Change this if your side/starting position changes.

        // Start camera/AprilTag system and pass in which tag we want to track.
        aprilTagWebcam = new AprilTagWebcam();
        aprilTagWebcam.init(hardwareMap, telemetry, robotPlacement.target_tag_1, robotPlacement.target_tag_2);
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
        }
    }
}
