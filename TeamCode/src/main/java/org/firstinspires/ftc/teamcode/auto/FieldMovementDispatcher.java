package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.navigate.DriveConfig;
import org.firstinspires.ftc.teamcode.navigate.MecanumWheelMotorOdometry;
import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;

/**
 * Converts high-level field commands into  target poses for SnapMecanumMover navigation.
 */
public class FieldMovementDispatcher {
    private final SnapMecanumMover mover;

    public FieldMovementDispatcher(SnapMecanumMover mover) {
        this.mover = mover;
    }

    public void dispatch(AutoCommand command) {
        switch (command.getType()) {
            case MOVE_FORWARD:
                moveForward(command.getValue());
                break;
            case MOVE_BACKWARD:
                moveBackward(command.getValue());
                break;
            case MOVE_LEFT:
                moveLeft(command.getValue());
                break;
            case MOVE_RIGHT:
                moveRight(command.getValue());
                break;
            case TURN:
                turn(command.getValue());
                break;
            default:
                throw new IllegalStateException("Unsupported command: " + command.getType());
        }
    }

    public void moveForward(double wheelRotations) {
        moveFieldDelta(0.0, WheelRotationConverter.wheelRotationsToInches(wheelRotations));
    }

    public void moveBackward(double wheelRotations) {
        moveFieldDelta(0.0, -WheelRotationConverter.wheelRotationsToInches(wheelRotations));
    }

    public void moveLeft(double wheelRotations) {
        moveFieldDelta(-WheelRotationConverter.wheelRotationsToInches(wheelRotations), 0.0);
    }

    public void moveRight(double wheelRotations) {
        moveFieldDelta(WheelRotationConverter.wheelRotationsToInches(wheelRotations), 0.0);
    }

    /**
     * Turn to an absolute field heading where north is 0 deg, right is positive, left is negative.
     */
    public void turn(double headingDegreesFromNorth) {
        MecanumWheelMotorOdometry.Pose2d pose = mover.getOdometry().getPose();
        double headingRadians = AngleUnit.normalizeRadians(Math.toRadians(headingDegreesFromNorth));
        mover.setTargetPose(pose.xInches, pose.yInches, headingRadians);
    }

    private void moveFieldDelta(double deltaXInches, double deltaYInches) {
        MecanumWheelMotorOdometry.Pose2d pose = mover.getOdometry().getPose();
        double targetX = Range.clip(pose.xInches + deltaXInches, 0.0, DriveConfig.FIELD_SIZE_INCHES);
        double targetY = Range.clip(pose.yInches + deltaYInches, 0.0, DriveConfig.FIELD_SIZE_INCHES);
        mover.setTargetPose(targetX, targetY, pose.headingRadians);
    }
}

