package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.REVDigitBoard;
import monologue.Annotations;
import monologue.Logged;

public class I2CDisplay extends SubsystemBase {
    private static final I2CDisplay instance = new I2CDisplay();
    private final REVDigitBoard board = new REVDigitBoard();

    private I2CDisplay() {
        board.display(12.40);
    }

    public static I2CDisplay getInstance() {
        return instance;
    } 
    // @Annotations.Log.NT
    public double getPot() {
        return board.getPot().getAverageVoltage();
    }

    public void display(String s) {
        board.display(s);
    }
}
