package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "scoring_shoot", group = "TeleOp")
public class scoring_shoot extends OpMode {

    // Configurable constants
    private static final double INTAKE_HALF_POWER = 1;
    private static final double WINDMILL_HALF_POWER = 1;
    private static final double SHOOTER_TARGET_RPM = 5600.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 5203 at 1:1 output ratio
    private static final double SHOOTER_MAX_RPM = 6000.0;
    private static final double SHOOTER_RPM_TOLERANCE = 100.0;
    private static final double TRIGGER_THRESHOLD = 0.1;

    // Simple PID + feedforward values for closed-loop RPM hold
    private static final double SHOOTER_KP = 0.00045;
    private static final double SHOOTER_KI = 0.00008;
    private static final double SHOOTER_KD = 0.00004;
    private static final double SHOOTER_KF = 1.0 / SHOOTER_MAX_RPM;
    private static final double SHOOTER_INTEGRAL_LIMIT = 2500.0;

    private CRServo frontLeftFeeder;
    private CRServo frontRightFeeder;
    private CRServo windmillServo;

    private DcMotor frontIntakeMotor;
    private DcMotorEx shooterMotor;

    private double shooterIntegral;
    private double shooterLastError;
    private long shooterLastTimeMs;

    @Override
    public void init() {
        frontLeftFeeder = hardwareMap.get(CRServo.class, "front_left_servo");
        frontRightFeeder = hardwareMap.get(CRServo.class, "front_right_servo");
        windmillServo = hardwareMap.get(CRServo.class, "windmill_servo");

        frontIntakeMotor = hardwareMap.get(DcMotor.class, "front_intake_motor");
        frontIntakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontIntakeMotor.setPower(0.0);

        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooter_motor");
        shooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooterMotor.setPower(0.0);

        stopFeedAndWindmill();
        resetShooterPid();

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Gamepad2: LT intake, RT shooter, A+RT windmill");
        telemetry.addData("Shooter Target RPM", SHOOTER_TARGET_RPM);
        telemetry.update();
    }

    @Override
    public void loop() {
        boolean intakeRequested = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        boolean shooterRequested = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        boolean windmillRequested = shooterRequested && gamepad2.a;

        if (intakeRequested) {
            // Left feeder CW, right feeder CCW, intake motor CW.
            frontLeftFeeder.setPower(INTAKE_HALF_POWER);
            frontRightFeeder.setPower(-INTAKE_HALF_POWER);
            frontIntakeMotor.setPower(INTAKE_HALF_POWER);
        } else {
            frontLeftFeeder.setPower(0.0);
            frontRightFeeder.setPower(0.0);
            frontIntakeMotor.setPower(0.0);
        }

        double shooterPower = 0.0;
        double shooterRpm = getShooterRpm();
        if (shooterRequested) {
            shooterPower = computeShooterPidPower(SHOOTER_TARGET_RPM, shooterRpm);
            shooterMotor.setPower(shooterPower);
        } else {
            shooterMotor.setPower(0.0);
            resetShooterPid();
        }

        if (windmillRequested) {
            // Counterclockwise windmill at half speed while RT and A are both held.
            windmillServo.setPower(-WINDMILL_HALF_POWER);
        } else {
            windmillServo.setPower(0.0);
        }

        boolean atTargetRpm = Math.abs(SHOOTER_TARGET_RPM - shooterRpm) <= SHOOTER_RPM_TOLERANCE;
        telemetry.addData("Shooter RPM", "%.1f", shooterRpm);
        telemetry.addData("Shooter Target RPM", "%.1f", SHOOTER_TARGET_RPM);
        telemetry.addData("Shooter Power", "%.3f", shooterPower);
        telemetry.addData("Shooter Ready", atTargetRpm ? "YES" : "NO");
        telemetry.addData("Intake Active", intakeRequested);
        telemetry.addData("Windmill Active", windmillRequested);
        telemetry.update();
    }

    @Override
    public void stop() {
        stopFeedAndWindmill();

        if (frontIntakeMotor != null) {
            frontIntakeMotor.setPower(0.0);
        }

        if (shooterMotor != null) {
            shooterMotor.setPower(0.0);
        }

        resetShooterPid();
    }

    private double getShooterRpm() {
        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
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

    private void resetShooterPid() {
        shooterIntegral = 0.0;
        shooterLastError = 0.0;
        shooterLastTimeMs = 0;
    }

    private void stopFeedAndWindmill() {
        if (frontLeftFeeder != null) {
            frontLeftFeeder.setPower(0.0);
        }
        if (frontRightFeeder != null) {
            frontRightFeeder.setPower(0.0);
        }
        if (windmillServo != null) {
            windmillServo.setPower(0.0);
        }
    }
}
