package frc.robot;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LaserCanSwitch {
    private static final LaserCanSwitch instance = new LaserCanSwitch();
    private final LaserCan m_lc = new LaserCan(3);

    private LaserCanSwitch() {
        
    }

    public static LaserCanSwitch getInstance() {
        return instance;
    }

    public boolean laserCanNotTripped() {
        return m_lc.getMeasurement().distance_mm >= 60;
    }

    public Trigger open() {
        return new Trigger(() -> m_lc.getMeasurement().distance_mm >= 80).debounce(.2, DebounceType.kRising);
    }

    public Trigger closed() {
        return new Trigger(() -> m_lc.getMeasurement().distance_mm < 80).debounce(.2, DebounceType.kRising);
    }

    public double laserCan() {
        return m_lc.getMeasurement().distance_mm;
    }

}
