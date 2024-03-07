package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.LaserCanSwitch;
import frc.robot.subsystems.*;

@SuppressWarnings("unused")
public class AutoCommands {
    private static final Pivot m_pivot = Pivot.getInstance();
    private static final Shooter m_shooter = Shooter.getInstance();
    private static final Intake m_intake = Intake.getInstance();
    private static final Indexer m_indexer = Indexer.getInstance();
    private static final LEDs m_leds = LEDs.getInstance();
    private static final Pneumatics m_pneumatics = Pneumatics.getInstance();
    private static final SideBySide m_sideBySide = SideBySide.getInstance();
    private static final I2CDisplay m_display = I2CDisplay.getInstance();

    private AutoCommands() {
    }

    // 26 degrees, subwoofer
    // 7.9 degrees, 1st note
    // 7.9 degrees, 2nd note
    public static Command shootSub() {
        return shootCustom(26, 0.25);
    }

    public static Command shootPod() {
        return shootCustom(7.4, .5);
    }

    public static Command shootTwo() {
        return shootCustom(8.2, .5);
    }
    public static Command shootThree() {
        return shootCustom(26.0, .75); //10.2
    }

    public static Command shootAmp() {
        return shootCustom(10.8, .5);
    }

    public static Command shootcr4() {
        return shootCustom(4.65, .5);
    }

    public static Command shootInitial() {
        return shootCustom(26.0, 1); //needs 20.85 initial angle for legal start
    }

    public static Command pivotSub() {
        return pivotCustom(26.0);
    }

    private static Command shootCustom(double angle, double timeout) {
        
        return Commands.parallel(
                m_shooter.run(),
                m_pivot.angleCommand(() -> angle),
                Commands.waitUntil(() -> m_shooter.upToSpeed() && m_pivot.atPosition())
                        .withTimeout(1)
                        .andThen(m_indexer.run()
                                .alongWith(m_sideBySide.run())),
                        Commands.print("Shooting")
        )
                .until(LaserCanSwitch.getInstance().fullyOpen())
        .withTimeout(timeout)
                .andThen(Commands.parallel(m_shooter.autoIdle(), m_indexer.stop(), m_sideBySide.stop())).withTimeout(timeout);
    }

    private static Command pivotCustom(double angle){
        return (m_pivot.angleCommand(()->angle));
    }


    public static Command intake() {
        return m_intake.run()
                .alongWith(m_indexer.run()).withTimeout(2).andThen(Commands.parallel(m_indexer.stop(),m_intake.stop())).withTimeout(2);
    }
}
