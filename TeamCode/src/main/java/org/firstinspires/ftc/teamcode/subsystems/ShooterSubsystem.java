package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * ShooterSubsystem (REV motor encoder fixed)
 *
 * Your REV motor spec: 28 counts/rev at the motor, free speed ~6000 RPM.
 * The FTC SDK motor config often reports the WRONG ticks/rev for some motors,
 * which makes your RPM look like ~250 even when it’s spinning fast.
 *
 * This class HARD-CODES ticks/rev so RPM math is correct.
 */
public class ShooterSubsystem {

    private final DcMotorEx left, right;

    // ===== IMPORTANT =====
    // Try 28 first. If your "ticks/sec" at full power is ~11200, switch to 112.
    private static final double TICKS_PER_REV = 28.0;

    // ===== Gains (live-tunable) =====
    public double kP = 0.002200;
    public double kI = 0.0;
    public double kD = 0;

    // Feedforward: power ~= kS + kV * RPM
    public double kV = 0.000197; // non-zero so FF-only can spin
    public double kS = 0.0;

    // ===== State =====
    private boolean enabled = false;
    private boolean tuningMode = false;      // true = FF-only (kV/kS), false = PIDF
    private boolean averageEncoders = true;  // safe now (averages ABS RPM)

    private double targetRPM = 0.0;

    // PID state
    private double iSum = 0.0;
    private double lastErr = 0.0;
    private long lastTimeNs = System.nanoTime();

    private double lastPower = 0.0;

    public ShooterSubsystem(DcMotorEx left, DcMotorEx right) {
        this.left = left;
        this.right = right;

        // Same-shaft setup: commonly one needs reversing depending on mounting.
        // If motors "fight" (growl / barely spin), flip ONE direction.
        left.setDirection(DcMotorSimple.Direction.REVERSE);
        right.setDirection(DcMotorSimple.Direction.FORWARD);

        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    // =====================
    // Basic controls
    // =====================

    public void setEnabled(boolean on) {
        enabled = on;
        if (!on) {
            left.setPower(0.0);
            right.setPower(0.0);
            lastPower = 0.0;
            resetPid();
        }
    }

    public boolean isEnabled() { return enabled; }

    public void setTargetRPM(double rpm) {
        targetRPM = MathUtil.clip(rpm, 0.0, 6500.0);
    }

    public double getTargetRPM() { return targetRPM; }

    public void setTuningMode(boolean on) {
        tuningMode = on;
        resetPid();
    }

    public boolean isTuningMode() { return tuningMode; }

    public void setAverageEncoders(boolean on) { averageEncoders = on; }
    public boolean isAverageEncoders() { return averageEncoders; }

    public void setRightMotorReversed(boolean reversed) {
        right.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    public void setLeftMotorReversed(boolean reversed) {
        left.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
    }

    // =====================
    // RPM / debug
    // =====================

    /** Raw motor velocity in ticks/sec (can be negative). */
    public double getLeftTicksPerSecRaw() { return left.getVelocity(); }

    /** Raw motor velocity in ticks/sec (can be negative). */
    public double getRightTicksPerSecRaw() { return right.getVelocity(); }

    public double getTicksPerRevUsed() { return TICKS_PER_REV; }

    /**
     * Returns POSITIVE RPM always.
     * If averageEncoders=true, averages ABS RPM from both motors so reversed motors won't cancel.
     */
    public double getRPM() {
        double leftRpmAbs = Math.abs((left.getVelocity() / TICKS_PER_REV) * 60.0);
        if (!averageEncoders) return leftRpmAbs;

        double rightRpmAbs = Math.abs((right.getVelocity() / TICKS_PER_REV) * 60.0);
        return (leftRpmAbs + rightRpmAbs) / 2.0;
    }

    public boolean atSpeed(double tolRpm) {
        return Math.abs(getRPM() - targetRPM) <= tolRpm;
    }

    public double getLastPower() { return lastPower; }

    // =====================
    // Main update
    // =====================

    public void update() {
        if (!enabled) {
            left.setPower(0.0);
            right.setPower(0.0);
            lastPower = 0.0;
            return;
        }

        double power;

        if (tuningMode) {
            // Feedforward ONLY (for tuning kV/kS)
            double ff = (kV * targetRPM) + (targetRPM > 50 ? kS : 0.0);
            power = MathUtil.clip(ff, 0.0, 1.0);
        } else {
            // PIDF
            power = pidf(targetRPM, getRPM());
            power = MathUtil.clip(power, 0.0, 1.0);
        }

        left.setPower(power);
        right.setPower(power);
        lastPower = power;
    }

    private double pidf(double target, double current) {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        if (dt <= 0) dt = 0.001;

        double err = target - current;

        iSum += err * dt;
        iSum = MathUtil.clip(iSum, -5000.0, 5000.0);

        double dErr = (err - lastErr) / dt;
        lastErr = err;

        double pid = (kP * err) + (kI * iSum) + (kD * dErr);
        double ff = (kV * target) + (target > 50 ? kS : 0.0);

        return pid + ff;
    }

    private void resetPid() {
        iSum = 0.0;
        lastErr = 0.0;
        lastTimeNs = System.nanoTime();
    }
}
