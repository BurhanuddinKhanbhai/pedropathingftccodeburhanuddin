package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.ShooterModel;

@TeleOp(name="TeleOp Main (Crash-Proof, No Angle Debug)", group="TeleOp")
public class TeleOp_MainSubsystems extends OpMode {

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;
    private LimelightSubsystem limelight;

    // Driver controls
    private boolean autoAimEnabled = true;
    private double manualTargetRPM = 4000;

    // Shooter gating (aim gate removed)
    private static final double RPM_TOL = 75;

    // Toggle edge detection (gamepad2)
    private boolean lastStart2 = false;
    private boolean lastBack2 = false;

    // Repeat-rate limiter for dpad tuning
    private long lastAdjustMs = 0;
    private static final long ADJUST_PERIOD_MS = 140;

    // Limelight safety / fallback
    private long llFaultUntilMs = 0;                  // if LL throws, ignore LL until this time
    private static final long LL_FAULT_COOLDOWN_MS = 600;
    private double lastGoodDistanceIn = 70;         // fallback distance for autoaim
    private long lastGoodDistanceMs = 0;
    private static final long GOOD_DIST_STALE_MS = 1000;

    @Override
    public void init() {
        // Drive motors
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        // Shooter motors
        DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class, "flywheel2");
        DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class, "flywheel");

        // Intake/feeder motors
        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        // Hood servo
        Servo hoodServo = hardwareMap.get(Servo.class, "hood");

        // Limelight
        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");

        drive = new DriveSubsystem(fl, fr, bl, br);
        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);
        limelight = new LimelightSubsystem(ll);

        // Pipeline 2
        limelight.init(2, 80);

        // Set your real robot values here
        limelight.targetHeightIn = 30;        // tag center height
        limelight.cameraHeightIn = 18;        // lens center height
        limelight.cameraMountAngleDeg = 0.0;  // tilt

        telemetry.addLine("TeleOp ready (Crash-Proof, No Angle Debug)");
        telemetry.addLine("GP1: drive, A=shoot hold, triggers=intake, B=feeder reverse, Y=override run both");
        telemetry.addLine("GP2: RB enable shooter, LB disable shooter, Start toggle AutoAim, Back toggle PIDF/FF");
        telemetry.update();
    }

    @Override
    public void loop() {
        // =======================
        // Safe Limelight update (never crash)
        // =======================
        safeLimelightUpdate();

        // Read Limelight safely (even if blocked)
        LimelightRead llr = safeLimelightRead();

        // =======================
        // Drive (gamepad1)
        // =======================
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = (Math.abs(y) < 0.05) ? 0 : y;
        x = (Math.abs(x) < 0.05) ? 0 : x;
        rx = (Math.abs(rx) < 0.05) ? 0 : rx;

        drive.drive(y, x, rx);

        // =======================
        // Shooter enable/disable (gamepad2)
        // =======================
        if (gamepad2.right_bumper) shooter.setEnabled(true);
        if (gamepad2.left_bumper)  shooter.setEnabled(false);

        // =======================
        // Toggle AutoAim + Tuning (gamepad2)
        // =======================
        boolean start2 = gamepad2.start;
        if (start2 && !lastStart2) autoAimEnabled = !autoAimEnabled;
        lastStart2 = start2;

        boolean back2 = gamepad2.back;
        if (back2 && !lastBack2) shooter.setTuningMode(!shooter.isTuningMode());
        lastBack2 = back2;

        // =======================
        // Dpad tuning + manual RPM adjust (gamepad2)
        // =======================
        handleLiveTuningAndRpmAdjust();

        // =======================
        // AutoAim setpoints (distance-based, using last good distance)
        // =======================
        double targetRPM;
        double hoodPos;

        if (autoAimEnabled) {
            boolean distFresh = (System.currentTimeMillis() - lastGoodDistanceMs) <= GOOD_DIST_STALE_MS;
            if (distFresh) {
                targetRPM = ShooterModel.rpmFromDistance(lastGoodDistanceIn);
                hoodPos   = ShooterModel.hoodFromDistance(lastGoodDistanceIn);
            } else {
                targetRPM = manualTargetRPM;
                hoodPos   = hood.getTargetPos();
            }
        } else {
            targetRPM = manualTargetRPM;
            hoodPos   = hood.getTargetPos();
        }

        shooter.setTargetRPM(targetRPM);
        hood.setTargetPos(hoodPos);

        // =======================
        // Intake/Feeder (gamepad1)
        // Aim gate removed: feeder depends only on RPM readiness.
        // =======================

        // Y override: force both forward no matter what
        if (gamepad1.y) {
            intakeFeeder.setIntakePower(0.75);
            intakeFeeder.setFeederPower(0.75);
        } else {

            // Hold A: shoot-hold (auto enable shooter, intake forward, feed when at speed)
            if (gamepad1.a) {
                shooter.setEnabled(true);

                intakeFeeder.setIntakePower(1.0);

                boolean allowFeed = shooter.atSpeed(RPM_TOL);
                intakeFeeder.setFeederPower(allowFeed ? 1.0 : 0.0);

            } else {
                // Normal intake: triggers
                double in = gamepad1.right_trigger;
                double out = gamepad1.left_trigger;

                if (in > 0.05) intakeFeeder.setIntakePower(in);
                else if (out > 0.05) intakeFeeder.setIntakePower(-out);
                else intakeFeeder.setIntakePower(0.0);

                // Feeder reverse: B
                if (gamepad1.b) intakeFeeder.setFeederPower(-1.0);
                else intakeFeeder.setFeederPower(0.0);
            }
        }

        // =======================
        // Update subsystems
        // =======================
        shooter.update();
        hood.update();
        intakeFeeder.update();

        // =======================
        // Telemetry (clean, no angle debug)
        // =======================
        telemetry.addData("AutoAim", autoAimEnabled);
        telemetry.addData("Shooter enabled", shooter.isEnabled());
        telemetry.addData("Mode", shooter.isTuningMode() ? "FF_ONLY (kV)" : "PIDF");
        telemetry.addData("RPM cur/target", "%.0f / %.0f", shooter.getRPM(), shooter.getTargetRPM());
        telemetry.addData("AtSpeed", shooter.atSpeed(RPM_TOL));
        telemetry.addData("Power", "%.3f", shooter.getLastPower());
        telemetry.addData("Hood", "%.3f", hood.getTargetPos());

        telemetry.addData("LL fault", (System.currentTimeMillis() < llFaultUntilMs));
        telemetry.addData("LL hasTarget", llr.hasTarget);
        telemetry.addData("LL pipeline", llr.pipeline);
        telemetry.addData("LL tx/ty", "%.2f / %.2f", llr.tx, llr.ty);
        telemetry.addData("Dist(lastGood)", "%.1f", lastGoodDistanceIn);

        telemetry.update();
    }

    // =========================
    // SAFE LIMELIGHT WRAPPERS
    // =========================

    private void safeLimelightUpdate() {
        long now = System.currentTimeMillis();
        if (now < llFaultUntilMs) return;

        try {
            limelight.update();
        } catch (Exception e) {
            llFaultUntilMs = now + LL_FAULT_COOLDOWN_MS;
        }
    }

    private LimelightRead safeLimelightRead() {
        LimelightRead out = new LimelightRead();

        long now = System.currentTimeMillis();
        if (now < llFaultUntilMs) {
            out.hasTarget = false;
            out.pipeline = -1;
            out.tx = 0;
            out.ty = 0;
            return out;
        }

        try {
            out.hasTarget = limelight.hasTarget();
            out.pipeline = limelight.getPipelineIndex();
            out.tx = limelight.getTxDeg();
            out.ty = limelight.getTyDeg();

            // Update last-good distance only when a target exists
            if (out.hasTarget) {
                double d = limelight.getDistanceInches();
                if (!Double.isNaN(d) && !Double.isInfinite(d) && d > 0.1 && d < 250.0) {
                    lastGoodDistanceIn = d;
                    lastGoodDistanceMs = now;
                }
            }
        } catch (Exception e) {
            llFaultUntilMs = now + LL_FAULT_COOLDOWN_MS;
            out.hasTarget = false;
            out.pipeline = -1;
            out.tx = 0;
            out.ty = 0;
        }

        return out;
    }

    private static class LimelightRead {
        boolean hasTarget = false;
        int pipeline = -1;
        double tx = 0.0;
        double ty = 0.0;
    }

    // =========================
    // TUNING / RPM ADJUST
    // =========================
    private void handleLiveTuningAndRpmAdjust() {
        long now = System.currentTimeMillis();
        if (now - lastAdjustMs < ADJUST_PERIOD_MS) return;

        boolean up = gamepad2.dpad_up;
        boolean down = gamepad2.dpad_down;
        if (!up && !down) return;

        int dir = up ? +1 : -1;

        boolean fine = gamepad2.left_trigger > 0.5;

        if (gamepad2.x) {
            double step = fine ? 0.000002 : 0.00001;
            shooter.kP = MathUtil.clip(shooter.kP + dir * step, 0.0, 0.01);
            lastAdjustMs = now;
            return;
        }

        if (gamepad2.y) {
            double step = fine ? 0.000002 : 0.00001;
            shooter.kD = MathUtil.clip(shooter.kD + dir * step, 0.0, 0.05);
            lastAdjustMs = now;
            return;
        }

        if (gamepad2.a) {
            double step = fine ? 0.0000002 : 0.000001;
            shooter.kV = MathUtil.clip(shooter.kV + dir * step, 0.0, 0.01);
            lastAdjustMs = now;
            return;
        }

        double rpmStep = fine ? 10 : 25;
        manualTargetRPM = MathUtil.clip(manualTargetRPM + dir * rpmStep, 0.0, 6500.0);
        lastAdjustMs = now;
    }
}