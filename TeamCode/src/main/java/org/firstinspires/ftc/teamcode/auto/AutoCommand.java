package org.firstinspires.ftc.teamcode.auto;

/**
 * Immutable command type used by AutoRunner.
 */
public final class AutoCommand {
    public enum Type {
        MOVE_FORWARD,
        MOVE_BACKWARD,
        MOVE_LEFT,
        MOVE_RIGHT,
        TURN
    }

    private final Type type;
    private final double value;

    private AutoCommand(Type type, double value) {
        this.type = type;
        this.value = value;
    }

    public static AutoCommand moveForward(double wheelRotations) {
        return new AutoCommand(Type.MOVE_FORWARD, wheelRotations);
    }

    public static AutoCommand moveBackward(double wheelRotations) {
        return new AutoCommand(Type.MOVE_BACKWARD, wheelRotations);
    }

    public static AutoCommand moveLeft(double wheelRotations) {
        return new AutoCommand(Type.MOVE_LEFT, wheelRotations);
    }

    public static AutoCommand moveRight(double wheelRotations) {
        return new AutoCommand(Type.MOVE_RIGHT, wheelRotations);
    }

    public static AutoCommand turn(double headingDegreesFromNorth) {
        return new AutoCommand(Type.TURN, headingDegreesFromNorth);
    }

    public Type getType() {
        return type;
    }

    public double getValue() {
        return value;
    }
}

