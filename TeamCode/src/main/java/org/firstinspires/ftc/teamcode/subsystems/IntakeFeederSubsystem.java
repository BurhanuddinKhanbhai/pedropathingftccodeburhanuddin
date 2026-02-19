package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.util.MathUtil;

public class IntakeFeederSubsystem {
    private final DcMotorEx intake, feeder;

    private double intakePower = 0.0;
    private double feederPower = 0.0;

    public IntakeFeederSubsystem(DcMotorEx intake, DcMotorEx feeder) {
        this.intake = intake;
        this.feeder = feeder;
    }

    public void setIntakePower(double p) { intakePower = MathUtil.clip(p, -1, 1); }
    public void setFeederPower(double p) { feederPower = MathUtil.clip(p, -1, 1); }

    public void update() {
        intake.setPower(intakePower);
        feeder.setPower(feederPower);
    }
}
