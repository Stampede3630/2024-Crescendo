package frc.robot.util;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.generated.TunerConstants;

import java.util.function.Supplier;

@SuppressWarnings("unused")
public class AllianceDependent<T> implements Supplier<T> {

    private final T blueAllianceValue;
    private final T redAllianceValue;

    public AllianceDependent(T blueAllianceValue, T redAllianceValue) {
        this.blueAllianceValue = blueAllianceValue;
        this.redAllianceValue = redAllianceValue;
    }

    private static DriverStation.Alliance getAlliance() {
        return TunerConstants.DriveTrain.getCurrentAlliance();
    }

    public T getBlueAllianceValue() {
        return blueAllianceValue;
    }

    public T getRedAllianceValue() {
        return redAllianceValue;
    }

    @Override
    public T get() {
        return getAlliance() == DriverStation.Alliance.Red ? redAllianceValue : blueAllianceValue;
    }
}
