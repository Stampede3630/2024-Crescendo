package frc.robot;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.Config;
import frc.robot.util.Configable;

public class LaserCanSwitch implements Configable {
    private static final LaserCanSwitch instance = new LaserCanSwitch();
    private LaserCan m_lc;
    @Config(name = "LC Enabled?")
    private boolean lcEnabled;
    private Measurement m = new Measurement(0, 0, 0, false, 0, null);

    private LaserCanSwitch() {
        try {
            m_lc = new LaserCan(17);
            m_lc.setRangingMode(LaserCan.RangingMode.SHORT);
            m_lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.err.println("FAILED TO CONFIGURE LASER CAN");
            m_lc = null;
        }
    }

    public static LaserCanSwitch getInstance() {
        return instance;
    }

    public Trigger fullyOpen() {
        return new Trigger(() -> {return laserCan() >= 180 && lcEnabled;
        }).debounce(.2, DebounceType.kRising);
    }

    public Trigger fullyClosed() {
        return new Trigger(() -> {
            return laserCan() < 80 && lcEnabled;
        }).debounce(.2, DebounceType.kRising);
    }

    public Trigger transientState() {
        return fullyOpen().or(fullyClosed()).negate();
    }

    public double laserCan() {
        if (m_lc == null)
            return 0;

        Measurement a = m_lc.getMeasurement();
        if (a != null) m = a;
        return m.distance_mm;
    }

}
