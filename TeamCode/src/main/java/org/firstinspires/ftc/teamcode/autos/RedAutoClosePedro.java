package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterModel;

@Autonomous(name = "RedAutoClose (Pedro)", group = "Pedro Pathing")
public class RedAutoClosePedro extends LinearOpMode {

    // =========================
    // RED SIDE POSES (EDIT THESE DIRECTLY)
    // These are the "mirrored" versions of your Blue poses, but now hard-coded so you can tweak freely.
    //
    // Notes:
    // - X values are negated vs Blue (your request).
    // - Headings are set to the proper mirrored headings (PI - theta), already converted into degrees below:
    //   47deg -> 133deg
    //   45deg -> 135deg
    //   90deg -> 90deg
    //   0deg  -> 180deg
    // =========================

    private static final Pose START = new Pose(-72, 72, Math.toRadians(180));

    private static final Pose FIRST_SHOT_POSE   = new Pose(-28, 37, Math.toRadians(133)); // from Blue (28,37,47)
    private static final Pose BACK_SHOT_POSE    = new Pose(-28, 32, Math.toRadians(133)); // from Blue (28,32,47)
    private static final Pose INTAKE_ROW1_POSE  = new Pose(-17, 82, Math.toRadians(90));  // from Blue (17,82,90)

    // Blue ROW2_START had x = -11, so red is x = +11
    private static final Pose ROW2_START_POSE   = new Pose(11, 32, Math.toRadians(90));   // from Blue (-11,32,90)
    private static final Pose INTAKE_ROW2_POSE  = new Pose(11, 84, Math.toRadians(90));   // from Blue (-11,84,90)

    private static final Pose FINAL_SHOT_POSE   = new Pose(-27, 27, Math.toRadians(135)); // from Blue (27,27,45)

    // =========================
    // Subsystems
    // =========================
    private Follower follower;

    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;
    private LimelightSubsystem limelight;

    // =========================
    // Shooting / feeding tuning
    // =========================
    private static final double RPM_TOL = 75;
    private static final long SPINUP_TIMEOUT_MS = 1800;

    private static final long FEED_MS = 850;
    private static final double INTAKE_FEED_PWR = 1.0;

    @Override
    public void runOpMode() {

        // ---------- Hardware ----------
        DcMotorEx fl = hardwareMap.get(DcMotorEx.class, "frontLeftDrive");
        DcMotorEx fr = hardwareMap.get(DcMotorEx.class, "frontRightDrive");
        DcMotorEx bl = hardwareMap.get(DcMotorEx.class, "backLeftDrive");
        DcMotorEx br = hardwareMap.get(DcMotorEx.class, "backRightDrive");

        DcMotorEx leftFly = hardwareMap.get(DcMotorEx.class, "flywheel2");
        DcMotorEx rightFly = hardwareMap.get(DcMotorEx.class, "flywheel");

        DcMotorEx intake = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        DcMotorEx feeder = hardwareMap.get(DcMotorEx.class, "intakeMotor2");

        Servo hoodServo = hardwareMap.get(Servo.class, "hood");

        Limelight3A ll = hardwareMap.get(Limelight3A.class, "limelight");

        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);
        limelight = new LimelightSubsystem(ll);

        // Pipeline 2 like your TeleOp
        limelight.init(2, 80);

        // HOME TEST VALUES (set to your real robot numbers)
        limelight.targetHeightIn = 30;
        limelight.cameraHeightIn = 18;
        limelight.cameraMountAngleDeg = 0.0;

        // ---------- Pedro follower ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        // ---------- Build paths ----------
        PathChain toFirstShot   = line(START, FIRST_SHOT_POSE);
        PathChain toBackShot    = line(FIRST_SHOT_POSE, BACK_SHOT_POSE);
        PathChain toIntakeRow1  = line(BACK_SHOT_POSE, INTAKE_ROW1_POSE);
        PathChain backToShoot1  = line(INTAKE_ROW1_POSE, BACK_SHOT_POSE);
        PathChain toRow2Start   = line(BACK_SHOT_POSE, ROW2_START_POSE);
        PathChain toIntakeRow2  = line(ROW2_START_POSE, INTAKE_ROW2_POSE);
        PathChain toFinalShot   = line(INTAKE_ROW2_POSE, FINAL_SHOT_POSE);

        telemetry.addLine("RedAutoClose (Pedro) ready");
        telemetry.addData("Start", poseStr(START));
        telemetry.addData("FirstShot", poseStr(FIRST_SHOT_POSE));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // AUTO SEQUENCE
        // =========================
        followAndWait(toFirstShot, FIRST_SHOT_POSE);
        shootBurst3();

        followAndWait(toBackShot, BACK_SHOT_POSE);

        setIntake(true);
        followAndWait(toIntakeRow1, INTAKE_ROW1_POSE);
        setIntake(false);

        followAndWait(backToShoot1, BACK_SHOT_POSE);
        shootBurst3();

        followAndWait(toRow2Start, ROW2_START_POSE);

        setIntake(true);
        followAndWait(toIntakeRow2, INTAKE_ROW2_POSE);
        setIntake(false);

        followAndWait(toFinalShot, FINAL_SHOT_POSE);
        shootBurst3();

        safeStopAll();
    }

    // =========================
    // Pedro helpers
    // Rotation fix: rotate from a.heading -> b.heading while driving
    // =========================
    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                .setLinearHeadingInterpolation(a.getHeading(), b.getHeading())
                .build();
    }

    private void followAndWait(PathChain path, Pose expectedEndPose) {
        follower.followPath(path);

        while (opModeIsActive() && follower.isBusy()) {
            follower.update();

            limelight.update();
            shooter.update();
            hood.update();
            intakeFeeder.update();

            Pose cur = follower.getPose();
            telemetry.addData("Cur XY", "(%.1f, %.1f)", cur.getX(), cur.getY());
            telemetry.addData("Cur H(deg)", "%.1f", Math.toDegrees(cur.getHeading()));
            telemetry.addData("Tgt H(deg)", "%.1f", Math.toDegrees(expectedEndPose.getHeading()));
            telemetry.update();

            idle();
        }
    }

    // =========================
    // Shooting / intake helpers
    // =========================
    private void shootBurst3() {
        shooter.setEnabled(true);

        ElapsedTime aimTimer = new ElapsedTime();
        aimTimer.reset();
        while (opModeIsActive() && aimTimer.milliseconds() < 350) {
            limelight.update();
            shooter.update();
            hood.update();
            idle();
        }

        double dist = limelight.getDistanceInches();
        double targetRPM = ShooterModel.rpmFromDistance(dist);
        double hoodPos = ShooterModel.hoodFromDistance(dist);

        shooter.setTargetRPM(targetRPM);
        hood.setTargetPos(hoodPos);

        ElapsedTime spin = new ElapsedTime();
        spin.reset();
        while (opModeIsActive() && spin.milliseconds() < SPINUP_TIMEOUT_MS) {
            limelight.update();
            shooter.update();
            hood.update();

            double cur = shooter.getRPM();
            boolean ready = shooter.atSpeed(RPM_TOL);

            telemetry.addData("Shoot dist(in)", "%.1f", dist);
            telemetry.addData("RPM cur/target", "%.0f / %.0f", cur, targetRPM);
            telemetry.addData("Hood", "%.3f", hoodPos);
            telemetry.addData("Ready", ready);
            telemetry.update();

            if (ready) break;
            idle();
        }

        ElapsedTime feedT = new ElapsedTime();
        feedT.reset();
        while (opModeIsActive() && feedT.milliseconds() < FEED_MS) {
            intakeFeeder.setIntakePower(INTAKE_FEED_PWR);
            intakeFeeder.setFeederPower(INTAKE_FEED_PWR);

            shooter.update();
            hood.update();
            intakeFeeder.update();
            idle();
        }

        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();

        sleep(100);
    }

    private void setIntake(boolean on) {
        if (on) {
            intakeFeeder.setIntakePower(1.0);
            intakeFeeder.setFeederPower(0.0);
        } else {
            intakeFeeder.setIntakePower(0.0);
            intakeFeeder.setFeederPower(0.0);
        }
        intakeFeeder.update();
    }

    private void safeStopAll() {
        try { shooter.setEnabled(false); } catch (Exception ignored) {}

        try {
            intakeFeeder.setIntakePower(0.0);
            intakeFeeder.setFeederPower(0.0);
            intakeFeeder.update();
        } catch (Exception ignored) {}
    }

    private String poseStr(Pose p) {
        return String.format("(%.1f, %.1f, %.1fdeg)", p.getX(), p.getY(), Math.toDegrees(p.getHeading()));
    }
}