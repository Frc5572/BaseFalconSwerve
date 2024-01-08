package frc.lib.util;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class CanCoderToSparkMaxInternal {

    private CANCoder absoluteEncoder;
    private RelativeEncoder relativeEncoder;
    private double gearRatio;
    // @formatter:off
    /*
     * AbsoluteEnc + runtimeOffset = gearedRelativeEnc
     */
    // @formatter:on
    private double runtimeOffset = 0.0;

    public CanCoderToSparkMaxInternal(CANCoder absoluteEncoder, RelativeEncoder relativeEncoder,
        double gearRatio) {
        this.absoluteEncoder = absoluteEncoder;
        this.relativeEncoder = relativeEncoder;
        this.gearRatio = gearRatio;

        align();
    }

    public double getRelativeEncoderUngearedDegreesRaw() {
        return relativeEncoder.getPosition() * 360.0;
    }

    public double getRelativeEncoderDegreesRaw() {
        return getRelativeEncoderUngearedDegreesRaw() / gearRatio;
    }

    public double getRelativeEncoderDegrees() {
        // @formatter:off
        /*
        AbsoluteEnc + runtimeOffset = gearedRelativeEnc.
        AbsoluteEnc = gearedRelativeEnc - runtimeOffset.
        */
        // @formatter:on
        return getRelativeEncoderDegreesRaw() - runtimeOffset;
    }

    public double getAbsoluteEncoderDegrees() {
        return absoluteEncoder.getPosition();
    }

    public void align() {
        // @formatter:off
        /*
        AbsoluteEnc + runtimeOffset = gearedRelativeEnc.
        runtimeOffset = gearedRelativeEnc - AbsoluteEnc.
        */
        // @formatter:on
        runtimeOffset = getRelativeEncoderDegreesRaw() - getAbsoluteEncoderDegrees();
    }

    public double absoluteAngleToRelativeEncoderRawValue(double absoluteAngle) {
        // @formatter:off
        /*
        AbsoluteEnc + runtimeOffset = gearedRelativeEnc.
        gearedRelativeEnc = RelativeEncDeg / gearRatio.
        RelativeEncDeg = RelativeEnc * 360.0.
        RelativeEnc = RelativeEncDeg / 360.0.
        RelativeEncDeg = gearedRelativeEnc * gearRatio.
        RelativeEnc = gearedRelativeEnc * gearRatio / 360.0.
        RelativeEnc = (AbsoluteEnc + runtimeOffset) * gearRatio / 360.0.
        */
        // @formatter:on
        return (absoluteAngle + runtimeOffset) * gearRatio / 360.0;
    }

    public void debugWriteToSmartDashboard(String key) {
        SmartDashboard.putNumber(key + ".runtimeOffset", runtimeOffset);
        SmartDashboard.putNumber(key + ".gearRatio", gearRatio);
        SmartDashboard.putNumber(key + ".getRelativeEncoderUngearedDegreesRaw()",
            this.getRelativeEncoderUngearedDegreesRaw());
        SmartDashboard.putNumber(key + ".getRelativeEncoderDegreesRaw()",
            this.getRelativeEncoderDegreesRaw());
        SmartDashboard.putNumber(key + ".getRelativeEncoderDegrees()",
            this.getRelativeEncoderDegrees());
        SmartDashboard.putNumber(key + ".getAbsoluteEncoderDegrees()",
            this.getAbsoluteEncoderDegrees());
        SmartDashboard.putNumber(
            key + ".absoluteAngleToRelativeEncoderRawValue(getAbsoluteEncoderDegrees())",
            this.absoluteAngleToRelativeEncoderRawValue(this.getAbsoluteEncoderDegrees()));
    }

}
