package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.util.MathUtil;

/**
 * LimelightSubsystem (Distance-first, minimal, reliable)
 *
 * What it does:
 * - Starts Limelight on the pipeline you choose
 * - Updates latest valid result
 * - Provides tx, ty
 * - Calculates distance from ty
 *
 * What it DOES NOT do (on purpose):
 * - No AprilTag ID reading
 * - No fiducial parsing
 *
 * Why:
 * - You already confirmed pipeline 2 sees tags and gives working tx/ty.
 * - Tag ID reading is SDK-version dependent and was blocking your progress.
 */
public class LimelightSubsystem {

    private final Limelight3A limelight;

    private LLResult latest = null;
    private long lastGoodMs = 0;

    // ===== YOU SET THESE =====
    // Measure camera lens CENTER height from floor
    public double cameraHeightIn = 8.5;

    // Home: set to your tag CENTER height (you said ~18")
    // Field: change to real tag center height
    public double targetHeightIn = 18.0;

    // Angle UP from horizontal:
    // straight forward = 0
    // tilted up = + degrees
    // tilted down = - degrees
    public double cameraMountAngleDeg = 0.0;

    // ===== Behavior =====
    public long staleMs = 250;
    public double distanceFilterAlpha = 0.85;

    private double filteredDistanceIn = 0.0;

    public LimelightSubsystem(Limelight3A limelight) {
        this.limelight = limelight;
    }

    /** Call once in OpMode.init() */
    public void init(int pipelineIndex, int pollRateHz) {
        limelight.setPollRateHz(pollRateHz);
        limelight.pipelineSwitch(pipelineIndex);
        limelight.start();
    }

    /** Call every loop */
    public void update() {
        LLResult r = limelight.getLatestResult();
        if (r != null && r.isValid()) {
            latest = r;
            lastGoodMs = System.currentTimeMillis();
        }
    }

    public boolean hasTarget() {
        if (latest == null) return false;
        return (System.currentTimeMillis() - lastGoodMs) <= staleMs;
    }

    public int getPipelineIndex() {
        return hasTarget() ? latest.getPipelineIndex() : -1;
    }

    public double getTxDeg() {
        return hasTarget() ? latest.getTx() : 0.0;
    }

    public double getTyDeg() {
        return hasTarget() ? latest.getTy() : 0.0;
    }

    /**
     * Distance in inches using:
     * dist = |(targetHeight - cameraHeight) / tan(cameraAngle + ty)|
     *
     * NOTE: We use ABS() on purpose so it cannot get stuck at 0 because of sign issues.
     * Once you're stable, you can remove ABS and make signs perfect — but this gets you working NOW.
     */
    public double getDistanceInches() {
        if (!hasTarget()) return filteredDistanceIn;

        double angleDeg = cameraMountAngleDeg + getTyDeg();
        double angleRad = Math.toRadians(angleDeg);

        double tan = Math.tan(angleRad);
        if (Math.abs(tan) < 1e-6) return filteredDistanceIn;

        double deltaH = targetHeightIn - cameraHeightIn;
        double dist = Math.abs(deltaH / tan);

        dist = MathUtil.clip(dist, 0.0, 250.0);
        filteredDistanceIn = MathUtil.lowPass(filteredDistanceIn, dist, distanceFilterAlpha);
        return filteredDistanceIn;
    }
}
