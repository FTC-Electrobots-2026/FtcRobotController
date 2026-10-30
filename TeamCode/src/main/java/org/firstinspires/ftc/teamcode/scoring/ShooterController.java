package org.firstinspires.ftc.teamcode.scoring;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class ShooterController {

    private static final double SHOOTER_KP = 0.00045;
    private static final double SHOOTER_KI = 0.00008;
    private static final double SHOOTER_KD = 0.00004;
    private static final double SHOOTER_KF = 1.0 / ScoringConfig.SHOOTER_MAX_RPM;
    private static final double SHOOTER_INTEGRAL_LIMIT = 2500.0;

    private DcMotorEx shooterMotor;

    private double shooterIntegral;
    private double shooterLastError;
    private long shooterLastTimeMs;
    private double shooterTargetRpm;
    private double shooterMaxSeenRpm;
    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevLeftBumper;

    private double lastRpm;
    private double lastPower;

    /**
     * Initializes shooter hardware and controller state.
     *
     * This should be called once from OpMode init().
     * It sets motor mode, zero-power behavior, and resets PID memory.
     */
    public void init(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0.0);

        shooterTargetRpm = ScoringConfig.SHOOTER_BASELINE_RPM;
        shooterMaxSeenRpm = 0.0;
        resetPid();
        resetButtonEdges();
        lastRpm = 0.0;
        lastPower = 0.0;
    }

    /**
     * Updates the persistent shooter target RPM from tuning buttons.
     *
     * Controls (edge-detected so one change per physical press):
     * - D-pad up: increase target RPM by configured step.
     * - D-pad down: decrease target RPM by configured step.
     * - Left bumper: reset target RPM back to baseline.
     */
    public void updateTargetRpmFromDriverButtons(Gamepad gamepad2) {
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        boolean leftBumper = gamepad2.left_bumper;

        if (dpadUp && !prevDpadUp) {
            shooterTargetRpm = Math.min(ScoringConfig.SHOOTER_MAX_RPM, shooterTargetRpm + ScoringConfig.SHOOTER_STEP_RPM);
            resetPid();
        }

        if (dpadDown && !prevDpadDown) {
            shooterTargetRpm = Math.max(ScoringConfig.SHOOTER_MIN_RPM, shooterTargetRpm - ScoringConfig.SHOOTER_STEP_RPM);
            resetPid();
        }

        if (leftBumper && !prevLeftBumper) {
            shooterTargetRpm = ScoringConfig.SHOOTER_BASELINE_RPM;
            shooterMaxSeenRpm = 0.0;
            resetPid();
        }

        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevLeftBumper = leftBumper;
    }

    /**
     * Applies one closed-loop shooter control step.
     *
     * @param shooterRequested true while driver is actively requesting shooter spin (RT held)
     * @param transientRpmAdjust temporary RPM offset applied only for this cycle
     *                           (example: windmill-time IMU yaw compensation)
     *
     * Control flow:
     * - Read current RPM from encoder velocity.
     * - If shooter requested, run PID+feedforward toward (target + transient adjust).
     * - If shooter not requested, stop motor and clear PID memory.
     */
    public void applyShooterSpeedControl(boolean shooterRequested, double transientRpmAdjust) {
        lastRpm = getShooterRpm();
        shooterMaxSeenRpm = Math.max(shooterMaxSeenRpm, lastRpm);

        if (shooterRequested) {
            double effectiveTargetRpm = Range.clip(
                    shooterTargetRpm + transientRpmAdjust,
                    ScoringConfig.SHOOTER_MIN_RPM,
                    ScoringConfig.SHOOTER_MAX_RPM
            );
            lastPower = computeShooterPidPower(effectiveTargetRpm, lastRpm);
            shooterMotor.setPower(lastPower);
        } else {
            lastPower = 0.0;
            shooterMotor.setPower(0.0);
            resetPid();
        }
    }

    public void stop() {
        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }
        lastPower = 0.0;
        resetPid();
        resetButtonEdges();
    }

    public double getTargetRpm() {
        return shooterTargetRpm;
    }

    public double getBaselineRpm() {
        return ScoringConfig.SHOOTER_BASELINE_RPM;
    }

    public double getLastRpm() {
        return lastRpm;
    }

    public double getLastPower() {
        return lastPower;
    }

    public double getMaxSeenRpm() {
        return shooterMaxSeenRpm;
    }

    public boolean isReady() {
        return Math.abs(shooterTargetRpm - lastRpm) <= ScoringConfig.SHOOTER_RPM_TOLERANCE;
    }

    private double getShooterRpm() {
        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / ScoringConfig.SHOOTER_TICKS_PER_REV) * 60.0;
    }

    private double computeShooterPidPower(double targetRpm, double currentRpm) {
        long now = System.currentTimeMillis();

        if (shooterLastTimeMs == 0) {
            shooterLastTimeMs = now;
            shooterLastError = targetRpm - currentRpm;
            return Range.clip(targetRpm * SHOOTER_KF, -1.0, 1.0);
        }

        double dt = (now - shooterLastTimeMs) / 1000.0;
        shooterLastTimeMs = now;
        dt = Math.max(0.001, Math.min(0.1, dt));

        double error = targetRpm - currentRpm;
        shooterIntegral += error * dt;
        shooterIntegral = Range.clip(shooterIntegral, -SHOOTER_INTEGRAL_LIMIT, SHOOTER_INTEGRAL_LIMIT);

        double derivative = (error - shooterLastError) / dt;
        shooterLastError = error;

        double ff = targetRpm * SHOOTER_KF;
        double pid = (SHOOTER_KP * error) + (SHOOTER_KI * shooterIntegral) + (SHOOTER_KD * derivative);
        return Range.clip(ff + pid, -1.0, 1.0);
    }

    private void resetPid() {
        shooterIntegral = 0.0;
        shooterLastError = 0.0;
        shooterLastTimeMs = 0;
    }

    private void resetButtonEdges() {
        prevDpadUp = false;
        prevDpadDown = false;
        prevLeftBumper = false;
    }
}

