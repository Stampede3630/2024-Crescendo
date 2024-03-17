package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
        return shootCustom(26, 1);
    }

    public static Command shootPod() {
        return shootCustom(8.0, 2);
    }

    public static Command shootTwo() {
        return shootCustom(8.7, 1);
    }

    public static Command shootThree() {
        return shootCustom(26.0, 1); //10.2
    }

    public static Command shootAmp() {
        return shootCustom(10.8, 1);
    }

    public static Command shootcr4() {
        return shootCustom(4.65, 1);
    }

    public static Command cmfiveShoot() {
        return shootCustom(0, 1);
    }

    public static Command shootInitial() {
        return shootCustomInitial(26.0, 2.0); //needs 20.85 initial angle for legal start
    }

    public static Command pivotSub() {
        return pivotCustom(26.0);
    }

    public static Command pivotPod() {
        return pivotCustom(8.4);
    }

    public static Command pivotCMFThree(){ //put back to 10!!
        return pivotCustom(10.0);
    }

    private static Command shootCustom(double angle, double timeout) {

        return Commands.parallel(
                m_pneumatics.down(),
                m_shooter.run(),
                // m_pivot.angleCommand(() -> angle),
                Commands.waitUntil(() -> m_shooter.upToSpeed()).withTimeout(1) // the max time we wait for shooter to spin up
                    .andThen(
                        Commands.parallel(
                            m_indexer.run(),
                            m_sideBySide.run()
                        )),
                Commands.print("Shooting")
            ).until(LaserCanSwitch.getInstance().fullyOpen()).withTimeout(timeout) // the max time we wait for spin up + shooting before moving on
            .andThen(
                Commands.parallel(
                    // m_shooter.stop(),
                    m_indexer.stop(),
                    m_sideBySide.stop()
                ).withTimeout(.01)
            );
    }

    private static Command shootCustomInitial(double angle, double timeout) { //this one does pivot stuff

        return Commands.parallel(
                m_pneumatics.down(),
                m_shooter.run(),
                m_pivot.angleCommand(() -> angle),
                Commands.waitUntil(() -> m_shooter.upToSpeed()).withTimeout(1) // the max time we wait for shooter to spin up
                    .andThen(
                        Commands.parallel(
                            m_indexer.run(),
                            m_sideBySide.run()
                        )),
                Commands.print("Shooting")
            ).until(LaserCanSwitch.getInstance().fullyOpen()).withTimeout(timeout) // the max time we wait for spin up + shooting before moving on
            .andThen(
                Commands.parallel(
                    // m_shooter.stop(),
                    m_indexer.stop(),
                    m_sideBySide.stop()
                ).withTimeout(.01)
            );
    }


    private static Command pivotCustom(double angle) {
        return (m_pivot.angleCommand(() -> angle));
    }


    public static Command intake() {
        return m_intake.run()
            .alongWith(m_indexer.run())
            .until(LaserCanSwitch.getInstance().fullyClosed()).withTimeout(3)
            .andThen(
                Commands.parallel(m_indexer.stop(), m_intake.stop())
                .withTimeout(.01)
            );
    }
}
