package frc.robot.util;

public class DoubleLookupLerp extends LookupLerp<Double, Double> {
    @Override
    protected Double lerp(Double x1, Double y1, Double x2, Double y2, Double key) {
        return y1 + (y2 - y1) * (key - x1) / (x2 - x1);
    }
}
