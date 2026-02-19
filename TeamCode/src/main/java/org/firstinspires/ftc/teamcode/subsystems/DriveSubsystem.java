package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.MathUtil;

public class DriveSubsystem {

    private final DcMotorEx fl, fr, bl, br;

    // 🔥 MAX DRIVE SPEED (change this anytime)
    private static final double MAX_DRIVE_POWER = 1;

    public DriveSubsystem(DcMotorEx fl, DcMotorEx fr, DcMotorEx bl, DcMotorEx br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;

        // Keep your original motor directions
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);
        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(double y, double x, double rx) {

        // Standard mecanum normalization
        double denom = Math.max(1.0, Math.abs(y) + Math.abs(x) + Math.abs(rx));

        double fL = (y + x + rx) / denom;
        double bL = (y - x + rx) / denom;
        double fR = (y - x - rx) / denom;
        double bR = (y + x - rx) / denom;

        // Clip just in case
        fL = MathUtil.clip(fL, -1, 1);
        fR = MathUtil.clip(fR, -1, 1);
        bL = MathUtil.clip(bL, -1, 1);
        bR = MathUtil.clip(bR, -1, 1);

        // 🔥 APPLY 50% SPEED LIMIT
        fL *= MAX_DRIVE_POWER;
        fR *= MAX_DRIVE_POWER;
        bL *= MAX_DRIVE_POWER;
        bR *= MAX_DRIVE_POWER;

        fl.setPower(fL);
        fr.setPower(fR);
        bl.setPower(bL);
        br.setPower(bR);
    }
}
