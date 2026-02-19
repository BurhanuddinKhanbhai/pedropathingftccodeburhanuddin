package org.firstinspires.ftc.teamcode.util;

public class MathUtil {
    public static double clip(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }

    public static double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    /** Simple low-pass filter: higher alpha = smoother, slower */
    public static double lowPass(double prev, double now, double alpha) {
        return alpha * prev + (1.0 - alpha) * now;
    }
}
