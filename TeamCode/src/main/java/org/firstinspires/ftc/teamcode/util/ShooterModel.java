package org.firstinspires.ftc.teamcode.util;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class ShooterModel {

    // Fill these with YOUR tested points (distance in inches)
    // Keep arrays same length, sorted by distance ascending.
    private static double[] D = { 0, 30, 40, 50, 60, 70, 80};
    private static double[] RPM = { 2600, 2650, 2700, 2800, 2950, 3600, 3901 };
    private static  double[] HOOD = { 1, 1, 1, 1, .8, .8, .8 };

    public static double rpmFromDistance(double distIn) {
        return interp(distIn, D, RPM);
    }

    public static double hoodFromDistance(double distIn) {
        return MathUtil.clip(interp(distIn, D, HOOD), 0.0, 1.0);
    }

    private static double interp(double x, double[] xs, double[] ys) {
        if (xs.length == 0) return 0;
        if (x <= xs[0]) return ys[0];
        if (x >= xs[xs.length - 1]) return ys[ys.length - 1];

        for (int i = 0; i < xs.length - 1; i++) {
            double x0 = xs[i], x1 = xs[i + 1];
            if (x >= x0 && x <= x1) {
                double t = (x - x0) / (x1 - x0);
                return MathUtil.lerp(ys[i], ys[i + 1], t);
            }
        }
        return ys[ys.length - 1];
    }
}