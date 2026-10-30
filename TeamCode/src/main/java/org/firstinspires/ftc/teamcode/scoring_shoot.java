package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "scoring_shoot", group = "TeleOp")
public class scoring_shoot extends OpMode {

    /*
     * =========================
     * STUDENT QUICKSTART
     * =========================
     * What this OpMode controls:
     * - Intake (pull game pieces in)
     * - Shooter wheel (spin to target RPM)
     * - Windmill servo (extra feed action while shooting)
     *
     * Where values come from:
     * - gamepad2 triggers/buttons from Driver Station
     * - shooter motor encoder velocity for RPM feedback
     *
     * What values go out:
     * - servo powers for left feeder, right feeder, windmill
     * - motor powers for intake motor and shooter motor
     *
     * 10-Step Flow (every loop):
     * 1) Read controls: LT, RT, and A.
     * 2) Decide booleans: intakeRequested, shooterRequested, windmillRequested.
     * 3) If intake requested, run feeder + intake motor.
     * 4) Else stop feeder + intake motor.
     * 5) Read shooter RPM from encoder velocity.
     * 6) If shooter requested, compute PID+FF power and apply it.
     * 7) Else stop shooter and reset PID memory.
     * 8) If windmillRequested (RT + A), run windmill servo; else stop it.
     * 9) Check if shooter is within RPM tolerance.
     * 10) Send telemetry so driver sees status.
     */

    // Configurable constants
    private static final double INTAKE_POWER = 1;
    private static final double WINDMILL_POWER = 0.75;
    private static final double SHOOTER_BASELINE_RPM = 5100.0;
    private static final double SHOOTER_STEP_RPM = 100.0;
    private static final double SHOOTER_MIN_RPM = 0.0;
    private static final double SHOOTER_TICKS_PER_REV = 28.0; // 5203 at 1:1 output ratio
    private static final double SHOOTER_MAX_RPM = 5600.0;
    private static final double SHOOTER_RPM_TOLERANCE = 100.0;
    private static final double TRIGGER_THRESHOLD = 0.1;



    /*
     * CONTROL MATH (LAYERED EXPLANATION)
     *
     * Layer 1: Beginner idea
     * - We want the shooter wheel to stay near one speed (target RPM).
     * - The code keeps checking: "Am I too slow or too fast?"
     * - Then it adjusts motor power to correct that error.
     *
     * Layer 2: Slightly deeper idea
     * - Feedforward (FF): a smart starting guess for motor power based on target RPM.
     * - P term: pushes harder when speed error is bigger.
     * - I term: fixes small leftover error that lasts over time.
     * - D term: reacts to how quickly error is changing (trend), helping prevent overshoot.
     *
     * Why feedforward is important:
     * - Without FF, PID must do all the work from zero and can feel slow or wobbly.
     * - With FF, we start close to the correct power, then PID only fine-tunes.
     */
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
    private double shooterTargetRpm;
    private double shooterMaxSeenRpm;
    private boolean prevDpadUp;
    private boolean prevDpadDown;
    private boolean prevLeftBumper;

    /**
     * Called once when you press INIT.
     *
     * What this method does:
     * - Connects code variables to real hardware names from the Robot Configuration.
     * - Sets safe startup states (motors/servos stopped).
     * - Configures shooter motor mode so encoder velocity is available.
     * - Resets PID memory so first control step starts clean.
     *
     * Outcome:
     * - Robot is ready for loop() control without accidental movement.
     */
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
        shooterTargetRpm = SHOOTER_BASELINE_RPM;
        shooterMaxSeenRpm = 0.0;
        prevDpadUp = false;
        prevDpadDown = false;
        prevLeftBumper = false;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "Gamepad2: LT intake, RT shooter, A+RT windmill, Dpad Up/Down target, LB reset");
        telemetry.addData("Shooter Target RPM", shooterTargetRpm);
        telemetry.update();
    }

    /**
     * Called repeatedly while TeleOp is running.
     *
     * Inputs read here:
     * - gamepad2.left_trigger  -> intake request
     * - gamepad2.right_trigger -> shooter request
     * - gamepad2.a + right_trigger -> windmill request
     *
     * Main outcomes:
     * - Starts/stops intake path
     * - Holds shooter near target RPM using PID + feedforward
     * - Starts/stops windmill servo
     * - Publishes telemetry for driver feedback
     */
    @Override
    public void loop() {
        updateShooterTargetRpmFromDpad();

        boolean intakeRequested = gamepad2.left_trigger > TRIGGER_THRESHOLD;
        boolean shooterRequested = gamepad2.right_trigger > TRIGGER_THRESHOLD;
        // Windmill only runs while BOTH shooter trigger and A are held.
        // This avoids accidental windmill movement when shooter is off.
        boolean windmillRequested = shooterRequested && gamepad2.a;
        double intakeMotorPower;

        if (intakeRequested) {
            // Left feeder CW, right feeder CCW, intake motor CW.
            frontLeftFeeder.setPower(INTAKE_POWER);
            frontRightFeeder.setPower(-INTAKE_POWER);
            intakeMotorPower = INTAKE_POWER;
        } else {
            frontLeftFeeder.setPower(0.0);
            frontRightFeeder.setPower(0.0);
            intakeMotorPower = 0.0;
        }

        double shooterPower = 0.0;
        double shooterRpm = getShooterRpm();
        shooterMaxSeenRpm = Math.max(shooterMaxSeenRpm, shooterRpm);
        if (shooterRequested) {
            // Closed-loop control: compute needed power based on current error.
            shooterPower = computeShooterPidPower(shooterTargetRpm, shooterRpm);
            shooterMotor.setPower(shooterPower);
        } else {
            shooterMotor.setPower(0.0);
            // Important: clear PID memory so old integral/derivative does not carry over.
            resetShooterPid();
        }

        if (windmillRequested) {
            // Counterclockwise windmill at half speed while RT and A are both held.
            windmillServo.setPower(-WINDMILL_POWER);
            // Intake assist: keep at least 50% intake power to help clear jams while feeding.
            intakeMotorPower = Math.max(intakeMotorPower, 0.5);
        } else {
            windmillServo.setPower(0.0);
        }

        frontIntakeMotor.setPower(intakeMotorPower);

        boolean atTargetRpmRange = Math.abs(shooterTargetRpm - shooterRpm) <= SHOOTER_RPM_TOLERANCE;
        double rpmError = shooterTargetRpm - shooterRpm;
        boolean shooterPowerSaturated = shooterRequested && Math.abs(shooterPower) >= 0.999;
        telemetry.addData("Shooter RPM", "%.1f", shooterRpm);
        telemetry.addData("Shooter Target RPM", "%.1f", shooterTargetRpm);
        telemetry.addData("Shooter Baseline RPM", "%.1f", SHOOTER_BASELINE_RPM);
        telemetry.addData("Shooter Power", "%.3f", shooterPower);
        telemetry.addData("Shooter Power Saturated", shooterPowerSaturated ? "YES" : "NO");
        telemetry.addData("Shooter RPM Error", "%.1f", rpmError);
        telemetry.addData("Shooter Estimated Max Seen RPM", "%.1f", shooterMaxSeenRpm);
        telemetry.addData("Shooter Ready", atTargetRpmRange ? "YES" : "NO");
        telemetry.addData("Intake Active", intakeRequested);
        telemetry.addData("Windmill Active", windmillRequested);
        telemetry.update();
    }

    /**
     * Called once when OpMode is stopped.
     *
     * Outcome:
     * - All motors/servos are commanded to zero power.
     * - PID memory is reset so next run starts fresh.
     */
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

    /**
     * Converts shooter encoder velocity to RPM.
     *
     * Where numbers come from:
     * - shooterMotor.getVelocity() gives ticks per second.
     * - SHOOTER_TICKS_PER_REV converts ticks -> wheel revolutions.
     * - multiply by 60 to convert per-second to per-minute.
     *
     * Returns:
     * - Shooter speed in RPM.
     */
    private double getShooterRpm() {
        double ticksPerSecond = shooterMotor.getVelocity();
        return (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
    }

    /**
     * Computes shooter motor power using PID + feedforward.
     *
     * Parameters:
     * - targetRpm: desired shooter speed
     * - currentRpm: measured shooter speed right now
     *
     * Returns:
     * - motor power command in range [-1.0, 1.0]
     *
     * Layer 1 (basics):
     * - "Error" means targetRpm - currentRpm.
     * - If error is big, we need more correction.
     * - If error is near zero, we are close to desired speed.
     *
     * Layer 2 (control details):
     * - Feedforward (ff): base power estimate from target RPM only.
     * - Proportional (P): immediate correction from current error.
     * - Integral (I): adds correction if error stays for a long time.
     * - Derivative (D): reacts to error trend (is error shrinking fast or growing fast?).
     *
     * About "derivative spikes":
     * - Derivative uses division by dt (time step).
     * - If dt is tiny because loop timing jittered, derivative can become huge suddenly.
     * - That huge jump can cause power spikes, so we clamp dt to keep math stable.
     */
    private double computeShooterPidPower(double targetRpm, double currentRpm) {
        long now = System.currentTimeMillis();

        if (shooterLastTimeMs == 0) {
            shooterLastTimeMs = now;
            shooterLastError = targetRpm - currentRpm;
            // First loop has no meaningful dt yet, so start with feedforward only.
            return Range.clip(targetRpm * SHOOTER_KF, -1.0, 1.0);
        }

        double dt = (now - shooterLastTimeMs) / 1000.0;
        shooterLastTimeMs = now;
        // Clamp dt to avoid unstable derivative spikes if loop timing jitters.
        // Very small dt would make (error change / dt) unrealistically large.
        dt = Math.max(0.001, Math.min(0.1, dt));

        double error = targetRpm - currentRpm;
        shooterIntegral += error * dt;
        // Anti-windup: keep integral from growing too large.
        shooterIntegral = Range.clip(shooterIntegral, -SHOOTER_INTEGRAL_LIMIT, SHOOTER_INTEGRAL_LIMIT);

        // Derivative = "how fast error is changing per second".
        // Positive large value: error is getting worse quickly.
        // Negative large value: error is being corrected quickly.
        double derivative = (error - shooterLastError) / dt;
        shooterLastError = error;

        // Feedforward: expected baseline power for desired RPM.
        double ff = targetRpm * SHOOTER_KF;
        // PID correction around feedforward baseline.
        // Final command = smart guess + correction.
        double pid = (SHOOTER_KP * error) + (SHOOTER_KI * shooterIntegral) + (SHOOTER_KD * derivative);
        return Range.clip(ff + pid, -1.0, 1.0);
    }

    /**
     * Clears PID state memory.
     *
     * Why needed:
     * - Prevents old integral/derivative history from affecting a later shooter restart.
     */
    private void resetShooterPid() {
        shooterIntegral = 0.0;
        shooterLastError = 0.0;
        shooterLastTimeMs = 0;
    }

    /**
     * Updates the persistent shooter target RPM from gamepad2 button presses.
     *
     * What this control does:
     * - Lets drivers "trim" shooter speed in fixed 100 RPM steps during a match.
     * - Keeps the new target even if shooter trigger is released and pressed again.
     * - Provides a quick reset (left bumper) back to baseline tuning.
     *
     * Button behavior:
     * - D-pad up    -> target += 100 RPM, capped at SHOOTER_MAX_RPM.
     * - D-pad down  -> target -= 100 RPM, floored at SHOOTER_MIN_RPM.
     * - Left bumper -> target = SHOOTER_BASELINE_RPM and clears max-seen RPM telemetry.
     *
     * Why rising-edge detection is used:
     * - FTC loop() runs many times per second.
     * - Without edge detection, holding a D-pad direction would apply many increments per second.
     * - By comparing current button state with previous state, each physical press changes exactly once.
     *
     * Why PID is reset after target changes:
     * - Integral and derivative history are tied to the previous target.
     * - Clearing PID memory avoids a stale correction "kick" when target steps up/down.
     */
    private void updateShooterTargetRpmFromDpad() {
        boolean dpadUp = gamepad2.dpad_up;
        boolean dpadDown = gamepad2.dpad_down;
        boolean leftBumper = gamepad2.left_bumper;

        // Rising edge: false -> true means a new press event.
        if (dpadUp && !prevDpadUp) {
            shooterTargetRpm = Math.min(SHOOTER_MAX_RPM, shooterTargetRpm + SHOOTER_STEP_RPM);
            resetShooterPid();
        }

        // Rising edge: false -> true means a new press event.
        if (dpadDown && !prevDpadDown) {
            shooterTargetRpm = Math.max(SHOOTER_MIN_RPM, shooterTargetRpm - SHOOTER_STEP_RPM);
            resetShooterPid();
        }

        // Rising edge reset back to baseline tune.
        if (leftBumper && !prevLeftBumper) {
            shooterTargetRpm = SHOOTER_BASELINE_RPM;
            shooterMaxSeenRpm = 0.0;
            resetShooterPid();
        }

        // Save current states for edge detection in the next loop cycle.
        prevDpadUp = dpadUp;
        prevDpadDown = dpadDown;
        prevLeftBumper = leftBumper;
    }

    /**
     * Safety helper to stop all feeder-related servos.
     *
     * This method checks for null so it is safe to call during lifecycle changes.
     */
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
