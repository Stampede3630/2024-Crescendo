package frc.robot;

import au.grapplerobotics.LaserCan;

public class LaserCanSwitch {
    private static final LaserCanSwitch instance = new LaserCanSwitch();
    private final LaserCan m_lc = new LaserCan(3);

    private LaserCanSwitch() {
    }

    public static LaserCanSwitch getInstance() {
        return instance;
    }

    public boolean laserCanTripped() {
        return m_lc.getMeasurement().distance_mm < 180;
    }

    public double laserCan() {
        return m_lc.getMeasurement().distance_mm;
    }

}
