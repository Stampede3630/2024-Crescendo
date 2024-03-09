package frc.robot;

import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.Measurement;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class LaserCanSwitch {
    private static final LaserCanSwitch instance = new LaserCanSwitch();
    private final LaserCan m_lc = new LaserCan(3);

    private Measurement m = new Measurement(0, 0, 0, false, 0, null);
    private LaserCanSwitch() {
        
    }

    public static LaserCanSwitch getInstance() {
        return instance;
    }

    public Trigger fullyOpen() {
        return new Trigger(() -> {
            if (m_lc == null)
                return false;
            
            Measurement a = m_lc.getMeasurement();
            if (a!=null) m=a;
            return m.distance_mm >= 180;
        }).debounce(.2, DebounceType.kRising);
    }

    public Trigger fullyClosed() {
        return new Trigger(() -> {
            if (m_lc == null)
                return false;
            
            Measurement a = m_lc.getMeasurement();
            if (a!=null) m=a;
            return m.distance_mm < 80;
        }).debounce(.2, DebounceType.kRising);
    }

    public Trigger transientState() {
        return fullyOpen().or(fullyClosed()).negate();
    }

    public double laserCan() {
        if (m_lc == null)
                return 0;
            
            Measurement a = m_lc.getMeasurement();
            if (a!=null) m=a;
        return m.distance_mm;
    }

}
