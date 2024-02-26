package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.REVDigitBoard;
import monologue.Annotations;
import monologue.Logged;

public class I2CDisplay extends SubsystemBase implements Logged {
    private REVDigitBoard board = new REVDigitBoard();
    public I2CDisplay() {

    }
    @Annotations.Log.NT
    public double getPot() {
        return board.getPot().getAverageVoltage();
    }

    public void display(String s) {
        board.display(s);
    }
}
