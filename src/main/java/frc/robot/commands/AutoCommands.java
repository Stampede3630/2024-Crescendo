package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.*;

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
        return Commands.parallel(
            m_shooter.run(),
            m_pivot.angleCommand(()->26.0),
            Commands.waitUntil(() -> m_shooter.upToSpeed() && m_pivot.atPosition())
                    .withTimeout(1)
                    .andThen(m_indexer.run()
                            .alongWith(m_sideBySide.run())),
            Commands.print("SHooting0")
            ).withTimeout(2).andThen(Commands.parallel(m_shooter.idle(), m_indexer.stop(), m_sideBySide.stop())).withTimeout(2);
    }
    
    public static Command shootPod() {
        return Commands.parallel(
            m_shooter.run(),
            m_pivot.angleCommand(()->7.4),
            Commands.waitUntil(() -> m_shooter.upToSpeed() && m_pivot.atPosition())
                    .withTimeout(1)
                    .andThen(m_indexer.run()
                            .alongWith(m_sideBySide.run())),
            Commands.print("SHooting1")
            ).withTimeout(1.5).andThen(Commands.parallel(m_shooter.idle(), m_indexer.stop(), m_sideBySide.stop())).withTimeout(1.5);
    }

    public static Command shootTwo() {
        return Commands.parallel(
            m_shooter.run(),
            m_pivot.angleCommand(()->8.2),
            Commands.waitUntil(() -> m_shooter.upToSpeed() && m_pivot.atPosition())
                    .withTimeout(1)
                    .andThen(m_indexer.run()
                            .alongWith(m_sideBySide.run())),
            Commands.print("SHooting2")
            ).withTimeout(1.5).andThen(Commands.parallel(m_shooter.idle(), m_indexer.stop(), m_sideBySide.stop())).withTimeout(1.5);
    }

    public static Command shoot() {
        return Commands.none(); // dummy command
    }
    

    public static Command intake() {
        return m_intake.run()
        .alongWith(m_indexer.run()).withTimeout(2).andThen(m_indexer.stop()).withTimeout(2);
    }
}
