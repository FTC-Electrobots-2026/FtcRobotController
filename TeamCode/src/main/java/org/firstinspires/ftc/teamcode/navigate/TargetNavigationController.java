package org.firstinspires.ftc.teamcode.navigate;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Holds target-navigation state and math so SnapMecanumMover can stay as an orchestrator.
 *
 * Behavior in this class is intentionally copied from the prior in-class implementation,
 * preserving all constants and equations.
 */
public class TargetNavigationController {

    public interface DriveFieldRelativeCommand {
        void driveFieldRelative(double forward, double right, double rotate);
    }

    private static final double TARGET_POSITION_TOLERANCE_INCHES = DriveConfig.TARGET_POSITION_TOLERANCE_INCHES;
    private static final double TARGET_HEADING_TOLERANCE_RADIANS = DriveConfig.TARGET_HEADING_TOLERANCE_RADIANS;
    private static final double TARGET_TRANSLATION_KP = 0.035;
    private static final double TARGET_HEADING_KP = 0.018;
    private static final double TARGET_HEADING_KD = 0.0015;
    private static final double TARGET_MAX_TRANSLATION_COMMAND = DriveConfig.TARGET_MAX_TRANSLATION_COMMAND;
    private static final double TARGET_MIN_TRANSLATION_COMMAND = DriveConfig.TARGET_MIN_TRANSLATION_COMMAND;
    private static final double TARGET_MAX_ROTATION_COMMAND = DriveConfig.TARGET_MAX_ROTATION_COMMAND;
    private static final double TARGET_MIN_ROTATION_COMMAND = DriveConfig.TARGET_MIN_ROTATION_COMMAND;

    private MecanumWheelMotorOdometry.Pose2d targetPose;
    private boolean targetNavigationActive = false;
    private double targetPositionToleranceInches = TARGET_POSITION_TOLERANCE_INCHES;
    private double targetHeadingToleranceRadians = TARGET_HEADING_TOLERANCE_RADIANS;
    private double targetTranslationKp = TARGET_TRANSLATION_KP;
    private double targetHeadingKp = TARGET_HEADING_KP;
    private double targetHeadingKd = TARGET_HEADING_KD;
    private double targetMaxTranslationCommand = TARGET_MAX_TRANSLATION_COMMAND;
    private double targetMinTranslationCommand = TARGET_MIN_TRANSLATION_COMMAND;
    private double targetMaxRotationCommand = TARGET_MAX_ROTATION_COMMAND;
    private double targetMinRotationCommand = TARGET_MIN_ROTATION_COMMAND;
    private double targetDistanceInches = 0.0;
    private double targetHeadingErrorRadians = 0.0;
    private double previousTargetHeadingErrorRadians = 0.0;
    private long previousTargetNavigationUpdateTimeNanos = 0L;

    private SnapMecanumMover.NavigationCommand lastTargetNavigationCommand =
            new SnapMecanumMover.NavigationCommand(0.0, 0.0, 0.0);

    public void setTargetPose(double xInches, double yInches, double headingRadians, MecanumWheelMotorOdometry activeOdometry) {
        double targetXInches = activeOdometry.isClampPoseToField()
                ? Range.clip(xInches, 0.0, activeOdometry.getFieldSizeInches())
                : xInches;
        double targetYInches = activeOdometry.isClampPoseToField()
                ? Range.clip(yInches, 0.0, activeOdometry.getFieldSizeInches())
                : yInches;

        targetPose = new MecanumWheelMotorOdometry.Pose2d(targetXInches, targetYInches, headingRadians);
        targetNavigationActive = true;

        previousTargetHeadingErrorRadians = 0.0;
        previousTargetNavigationUpdateTimeNanos = System.nanoTime();
    }

    public boolean navigateToTarget(
            MecanumWheelMotorOdometry activeOdometry,
            DriveFieldRelativeCommand driveCommand
    ) {
        activeOdometry.update();

        if (targetPose == null) {
            stopTargetNavigation(driveCommand);
            targetNavigationActive = false;
            return true;
        }

        updateTargetErrors(activeOdometry);

        if (isAtTarget(activeOdometry)) {
            stopTargetNavigation(driveCommand);
            targetNavigationActive = false;
            return true;
        }

        double rightCommand = (targetPose.xInches - activeOdometry.getXInches()) * targetTranslationKp;
        double forwardCommand = (targetPose.yInches - activeOdometry.getYInches()) * targetTranslationKp;
        double translationMagnitude = Math.hypot(forwardCommand, rightCommand);

        if (targetDistanceInches <= targetPositionToleranceInches) {
            forwardCommand = 0.0;
            rightCommand = 0.0;
        } else {
            if (translationMagnitude > targetMaxTranslationCommand && translationMagnitude > 0.0) {
                double scale = targetMaxTranslationCommand / translationMagnitude;
                forwardCommand *= scale;
                rightCommand *= scale;
                translationMagnitude = targetMaxTranslationCommand;
            }

            if (translationMagnitude < targetMinTranslationCommand && translationMagnitude > 0.0) {
                double scale = targetMinTranslationCommand / translationMagnitude;
                forwardCommand *= scale;
                rightCommand *= scale;
            }

            double headingScale = Range.clip(1.0 - (Math.abs(targetHeadingErrorRadians) / Math.PI), 0.25, 1.0);
            forwardCommand *= headingScale;
            rightCommand *= headingScale;
        }

        long nowNanos = System.nanoTime();
        double deltaTimeSeconds = (nowNanos - previousTargetNavigationUpdateTimeNanos) / 1_000_000_000.0;
        if (deltaTimeSeconds <= 0.0) {
            deltaTimeSeconds = 0.001;
        }

        double headingErrorRate =
                AngleUnit.normalizeRadians(targetHeadingErrorRadians - previousTargetHeadingErrorRadians) / deltaTimeSeconds;
        double rotateCommand = (targetHeadingErrorRadians * targetHeadingKp) + (headingErrorRate * targetHeadingKd);
        rotateCommand = Range.clip(rotateCommand, -targetMaxRotationCommand, targetMaxRotationCommand);

        if (Math.abs(targetHeadingErrorRadians) <= targetHeadingToleranceRadians) {
            rotateCommand = 0.0;
        } else if (Math.abs(rotateCommand) < targetMinRotationCommand) {
            rotateCommand = Math.copySign(targetMinRotationCommand, rotateCommand);
        }

        if (targetDistanceInches <= Math.max(targetPositionToleranceInches * 2.0, 3.0)) {
            forwardCommand *= 0.4;
            rightCommand *= 0.4;
        }

        driveCommand.driveFieldRelative(forwardCommand, rightCommand, rotateCommand);
        lastTargetNavigationCommand = new SnapMecanumMover.NavigationCommand(forwardCommand, rightCommand, rotateCommand);
        previousTargetHeadingErrorRadians = targetHeadingErrorRadians;
        previousTargetNavigationUpdateTimeNanos = nowNanos;
        targetNavigationActive = true;

        return false;
    }

    public boolean isAtTarget(MecanumWheelMotorOdometry activeOdometry) {
        if (targetPose == null) {
            return true;
        }

        updateTargetErrors(activeOdometry);
        return targetDistanceInches <= targetPositionToleranceInches
                && Math.abs(targetHeadingErrorRadians) <= targetHeadingToleranceRadians;
    }

    public void cancelTarget(DriveFieldRelativeCommand driveCommand) {
        targetPose = null;
        targetNavigationActive = false;
        targetDistanceInches = 0.0;
        targetHeadingErrorRadians = 0.0;
        previousTargetHeadingErrorRadians = 0.0;
        stopTargetNavigation(driveCommand);
    }

    public void addTargetNavigationTelemetry(Telemetry telemetry, MecanumWheelMotorOdometry activeOdometry) {
        telemetry.addLine("=== SNAP TARGET NAVIGATION ===");
        telemetry.addData("Target navigation active", targetNavigationActive ? "YES" : "NO");
        telemetry.addData("At target", isAtTarget(activeOdometry) ? "YES" : "NO");

        if (targetPose != null) {
            telemetry.addData("Target X / Y (in)", "%.2f / %.2f", targetPose.xInches, targetPose.yInches);
            telemetry.addData("Target X / Y (ft)", "%.2f / %.2f", targetPose.getXFeet(), targetPose.getYFeet());
            telemetry.addData("Target heading (deg)", "%.2f", targetPose.getHeadingDegrees());
            telemetry.addData("Target distance (in)", "%.2f", targetDistanceInches);
            telemetry.addData("Target heading error (deg)", "%.2f", Math.toDegrees(targetHeadingErrorRadians));
        }

        telemetry.addData("Last nav F / R / Turn", "%.3f / %.3f / %.3f",
                lastTargetNavigationCommand.forward,
                lastTargetNavigationCommand.right,
                lastTargetNavigationCommand.rotate);
    }

    public SnapMecanumMover.NavigationCommand getLastTargetNavigationCommand() {
        return lastTargetNavigationCommand;
    }

    public boolean isTargetNavigationActive() {
        return targetNavigationActive;
    }

    private void updateTargetErrors(MecanumWheelMotorOdometry activeOdometry) {
        if (targetPose == null) {
            targetDistanceInches = 0.0;
            targetHeadingErrorRadians = 0.0;
            return;
        }

        double xErrorInches = targetPose.xInches - activeOdometry.getXInches();
        double yErrorInches = targetPose.yInches - activeOdometry.getYInches();
        targetDistanceInches = Math.hypot(xErrorInches, yErrorInches);
        targetHeadingErrorRadians = AngleUnit.normalizeRadians(targetPose.headingRadians - activeOdometry.getHeadingRadians());
    }

    private void stopTargetNavigation(DriveFieldRelativeCommand driveCommand) {
        driveCommand.driveFieldRelative(0.0, 0.0, 0.0);
        lastTargetNavigationCommand = new SnapMecanumMover.NavigationCommand(0.0, 0.0, 0.0);
    }
}

