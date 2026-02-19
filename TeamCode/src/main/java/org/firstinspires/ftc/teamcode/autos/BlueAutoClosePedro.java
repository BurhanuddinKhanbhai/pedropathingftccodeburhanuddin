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
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HoodSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeFeederSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.util.ShooterModel;

@Autonomous(name = "BlueAutoClose (Pedro)", group = "Pedro Pathing")
public class BlueAutoClosePedro extends LinearOpMode {

    // =========================
    // 1) START POSE (YOU SAID THIS)
    // =========================
    private static final Pose START = new Pose(72, 72, Math.toRadians(0));

    // =========================
    // 2) WAYPOINTS (ported from your old RR route)
    // Assumption: RR (0,0) == Pedro (72,72)
    // Change these if your coordinate mapping differs.
    // =========================
    private static final Pose FIRST_SHOT_POSE   = new Pose(28, 37, Math.toRadians(47)); // RR(-44,-35)
    private static final Pose BACK_SHOT_POSE    = new Pose(28, 32, Math.toRadians(47)); // RR(-44,-40)
    private static final Pose INTAKE_ROW1_POSE  = new Pose(17, 82, Math.toRadians(90)); // RR(-55,10)
    private static final Pose ROW2_START_POSE   = new Pose(-11, 32, Math.toRadians(90)); // RR(-83,-40)
    private static final Pose INTAKE_ROW2_POSE  = new Pose(-11, 84, Math.toRadians(90)); // RR(-83,12)
    private static final Pose FINAL_SHOT_POSE   = new Pose(27, 27, Math.toRadians(45)); // RR(-45,-45)

    // =========================
    // Subsystems
    // =========================
    private Follower follower;

    private DriveSubsystem drive; // not required for auto, but harmless to keep if you want manual overrides later
    private ShooterSubsystem shooter;
    private HoodSubsystem hood;
    private IntakeFeederSubsystem intakeFeeder;
    private LimelightSubsystem limelight;

    // =========================
    // Shooting / feeding tuning
    // =========================
    private static final double RPM_TOL = 75;
    private static final long SPINUP_TIMEOUT_MS = 1800;

    // How long to run feeder to dump ~3 pixels (tune on field)
    private static final long FEED_MS = 850;
    private static final double INTAKE_FEED_PWR = 1.0;

    @Override
    public void runOpMode() {

        // ---------- Hardware (same names as your TeleOp_MainSubsystems) ----------
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

        drive = new DriveSubsystem(fl, fr, bl, br);
        shooter = new ShooterSubsystem(leftFly, rightFly);
        hood = new HoodSubsystem(hoodServo);
        intakeFeeder = new IntakeFeederSubsystem(intake, feeder);
        limelight = new LimelightSubsystem(ll);

        // Pipeline 2 like your TeleOp
        limelight.init(2, 80);

        // HOME TEST VALUES (set these to your real robot numbers)
        limelight.targetHeightIn = 16;        // tag center height
        limelight.cameraHeightIn = 18;        // lens center height
        limelight.cameraMountAngleDeg = 0.0;  // tilt angle

        // ---------- Pedro follower ----------
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(START);

        // ---------- Build paths ----------
        PathChain toFirstShot = line(START, FIRST_SHOT_POSE);
        PathChain toBackShot = line(FIRST_SHOT_POSE, BACK_SHOT_POSE);
        PathChain toIntakeRow1 = line(BACK_SHOT_POSE, INTAKE_ROW1_POSE);
        PathChain backToShoot1 = line(INTAKE_ROW1_POSE, BACK_SHOT_POSE);
        PathChain toRow2Start = line(BACK_SHOT_POSE, ROW2_START_POSE);
        PathChain toIntakeRow2 = line(ROW2_START_POSE, INTAKE_ROW2_POSE);
        PathChain toFinalShot = line(INTAKE_ROW2_POSE, FINAL_SHOT_POSE);

        telemetry.addLine("BlueAutoClose (Pedro) ready");
        telemetry.addData("Start", poseStr(START));
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // =========================
        // AUTO SEQUENCE
        // =========================

        // 1) Go to first shot spot
        followAndWait(toFirstShot);
        shootBurst3();

        // 2) Go where you were shooting from in the RR file (-44,-40)
        followAndWait(toBackShot);

        // 3) Intake first row
        setIntake(true);
        followAndWait(toIntakeRow1);
        setIntake(false);

        // 4) Back and shoot again
        followAndWait(backToShoot1);
        shootBurst3();

        // 5) Go to second row start, intake second row
        followAndWait(toRow2Start);
        setIntake(true);
        followAndWait(toIntakeRow2);
        setIntake(false);

        // 6) Final shot spot + shoot
        followAndWait(toFinalShot);
        shootBurst3();

        // Stop everything
        safeStopAll();
    }

    // =========================
    // Pedro helpers
    // =========================

    private PathChain line(Pose a, Pose b) {
        return follower.pathBuilder()
                .addPath(new BezierLine(a, b))
                // Keep constant heading per segment (simple + reliable).
                // If you want the robot to rotate while moving, swap to linear heading interpolation.
                .setConstantHeadingInterpolation(a.getHeading())
                .build();
    }

    private void followAndWait(PathChain path) {
        follower.followPath(path);
        while (opModeIsActive() && follower.isBusy()) {
            // Update follower + subsystems continuously
            follower.update();
            limelight.update();
            shooter.update();
            hood.update();
            intakeFeeder.update();

            telemetry.addData("Pose", follower.getPose());
            telemetry.update();
        }
    }

    // =========================
    // Shooting / intake helpers
    // =========================

    private void shootBurst3() {
        // Enable shooter
        shooter.setEnabled(true);

        // Try to get a fresh Limelight result for distance
        ElapsedTime aimTimer = new ElapsedTime();
        aimTimer.reset();
        while (opModeIsActive() && aimTimer.milliseconds() < 350) {
            limelight.update();
            shooter.update();
            hood.update();
            idle();
        }

        double dist = limelight.getDistanceInches(); // filtered distance
        double targetRPM = ShooterModel.rpmFromDistance(dist);
        double hoodPos = ShooterModel.hoodFromDistance(dist);

        shooter.setTargetRPM(targetRPM);
        hood.setTargetPos(hoodPos);

        // Spinup wait (with timeout)
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

        // Feed burst (tune FEED_MS to get exactly 3)
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

        // stop feeder
        intakeFeeder.setIntakePower(0);
        intakeFeeder.setFeederPower(0);
        intakeFeeder.update();

        // keep shooter running (optional; keeps next shot consistent)
        // shooter.setEnabled(true); // already enabled

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
        try {
            shooter.setEnabled(false);
        } catch (Exception ignored) {}

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
