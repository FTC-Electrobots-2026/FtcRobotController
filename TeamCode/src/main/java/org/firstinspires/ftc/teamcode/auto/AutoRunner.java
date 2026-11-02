package org.firstinspires.ftc.teamcode.auto;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import java.util.ArrayDeque;
import java.util.Queue;

import org.firstinspires.ftc.teamcode.navigate.SnapMecanumMover;

/**
 * Queues and executes field-oriented auto nomous commands via SnapMecanumMover navigation.
 */
public class AutoRunner {
    private static final double MOVE_TIMEOUT_SECONDS = 7.0;
    private static final double TURN_TIMEOUT_SECONDS = 4.0;
    private static final double TURN_STOP_TOLERANCE_RADIANS = Math.toRadians(2.0);
    private static final int REQUIRED_SETTLED_LOOPS = 6;
    private static final double TURN_MAX_POWER = 0.55;
    private static final double TURN_MIN_POWER = 0.11;
    private static final double TURN_KP = 1.15;
    private static final double TURN_MAX_SLEW_PER_SECOND = 2.4;

    private final SnapMecanumMover mover;
    private final FieldMovementDispatcher dispatcher;
    private final Queue<AutoCommand> commandQueue = new ArrayDeque<>();

    private AutoCommand activeCommand;
    private long activeCommandStartNanos;
    private double turnTargetHeadingRadians;
    private int settledLoopCount;
    private double previousTurnCommand;
    private long previousTurnUpdateNanos;

    public AutoRunner(SnapMecanumMover mover) {
        this.mover = mover;
        this.dispatcher = new FieldMovementDispatcher(mover);
        this.mover.initializeOdometry();
    }

    public AutoRunner MoveForward(double wheelRotations) {
        commandQueue.add(AutoCommand.moveForward(wheelRotations));
        return this;
    }

    public AutoRunner MoveBackward(double wheelRotations) {
        commandQueue.add(AutoCommand.moveBackward(wheelRotations));
        return this;
    }

    public AutoRunner MoveLeft(double wheelRotations) {
        commandQueue.add(AutoCommand.moveLeft(wheelRotations));
        return this;
    }

    public AutoRunner MoveRight(double wheelRotations) {
        commandQueue.add(AutoCommand.moveRight(wheelRotations));
        return this;
    }

    /**
     * Turn to absolute field heading where north is 0 deg, right is positive, left is negative.
     */
    public AutoRunner Turn(double angleDegreesFromNorth) {
        commandQueue.add(AutoCommand.turn(angleDegreesFromNorth));
        return this;
    }

    /**
     * Runs one autonomous update cycle.
     *
     * @return true when all queued commands have completed.
     */
    public boolean update() {
        if (activeCommand == null) {
            activeCommand = commandQueue.poll();
            if (activeCommand != null) {
                startCommand(activeCommand);
            }
        }

        if (activeCommand == null) {
            return true;
        }

        boolean finished = (activeCommand.getType() == AutoCommand.Type.TURN)
                ? updateTurnCommand()
                : updateMoveCommand();

        if (finished) {
            finishActiveCommand();
        }

        return activeCommand == null && commandQueue.isEmpty();
    }

    private void startCommand(AutoCommand command) {
        activeCommandStartNanos = System.nanoTime();

        if (command.getType() == AutoCommand.Type.TURN) {
            mover.cancelTarget();
            turnTargetHeadingRadians = AngleUnit.normalizeRadians(Math.toRadians(command.getValue()));
            settledLoopCount = 0;
            previousTurnCommand = 0.0;
            previousTurnUpdateNanos = System.nanoTime();
            return;
        }

        dispatcher.dispatch(command);
    }

    private boolean updateMoveCommand() {
        if (mover.navigateToTarget()) {
            return true;
        }

        return elapsedSeconds(activeCommandStartNanos) >= MOVE_TIMEOUT_SECONDS;
    }

    private boolean updateTurnCommand() {
        mover.getOdometry().update();
        double currentHeadingRadians = mover.getOdometry().getHeadingRadians();
        double headingErrorRadians = AngleUnit.normalizeRadians(turnTargetHeadingRadians - currentHeadingRadians);

        if (Math.abs(headingErrorRadians) <= TURN_STOP_TOLERANCE_RADIANS) {
            settledLoopCount++;
        } else {
            settledLoopCount = 0;
        }

        if (settledLoopCount >= REQUIRED_SETTLED_LOOPS) {
            mover.driveFieldRelative(0.0, 0.0, 0.0);
            return true;
        }

        if (elapsedSeconds(activeCommandStartNanos) >= TURN_TIMEOUT_SECONDS) {
            mover.driveFieldRelative(0.0, 0.0, 0.0);
            return true;
        }

        double turnCommand = headingErrorRadians * TURN_KP;
        turnCommand = clip(turnCommand, -TURN_MAX_POWER, TURN_MAX_POWER);
        if (Math.abs(turnCommand) < TURN_MIN_POWER) {
            turnCommand = Math.copySign(TURN_MIN_POWER, turnCommand);
        }

        long nowNanos = System.nanoTime();
        double dtSeconds = Math.max((nowNanos - previousTurnUpdateNanos) / 1_000_000_000.0, 0.001);
        double maxDelta = TURN_MAX_SLEW_PER_SECOND * dtSeconds;
        turnCommand = clip(turnCommand, previousTurnCommand - maxDelta, previousTurnCommand + maxDelta);
        previousTurnCommand = turnCommand;
        previousTurnUpdateNanos = nowNanos;

        mover.driveFieldRelative(0.0, 0.0, turnCommand);
        return false;
    }

    private void finishActiveCommand() {
        mover.cancelTarget();
        activeCommand = null;
    }

    private static double elapsedSeconds(long startNanos) {
        return (System.nanoTime() - startNanos) / 1_000_000_000.0;
    }

    private static double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public boolean isIdle() {
        return activeCommand == null && commandQueue.isEmpty();
    }

    public int getPendingCommandCount() {
        return commandQueue.size() + (activeCommand == null ? 0 : 1);
    }

    public void clear() {
        commandQueue.clear();
        activeCommand = null;
        settledLoopCount = 0;
        previousTurnCommand = 0.0;
        mover.cancelTarget();
        mover.driveFieldRelative(0.0, 0.0, 0.0);
    }
}

