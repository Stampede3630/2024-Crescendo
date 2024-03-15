package frc.robot.util;

import java.util.function.BooleanSupplier;

public class TimeElapsedTrigger implements BooleanSupplier {
    private long startTime;
    private long timerLengthMs;

    public void start() {
        startTime = System.currentTimeMillis();
    }

    public TimeElapsedTrigger(long timerLengthMs) {
        this.timerLengthMs = timerLengthMs;
    }
    @Override
    public boolean getAsBoolean() {
        // TODO Auto-generated method stub
        return System.currentTimeMillis() - startTime > timerLengthMs;
    }
    
}
