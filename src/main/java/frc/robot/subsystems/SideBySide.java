package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import java.util.function.DoubleSupplier;

public class SideBySide extends SubsystemBase implements Configable {
    private final TalonFX m_motor = new TalonFX(15, "CANIVORE");
    @Config(name = "sideBySide velocity")
    private double velocity = .3;
    public SideBySide() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
        );

        setDefaultCommand(stop());
    }
    public Command velocityCommand(DoubleSupplier _velocity) {
        return Commands.startEnd(() -> m_motor.setControl(new DutyCycleOut(_velocity.getAsDouble())), () -> {}, this);
    }

    public Command run() {
        return velocityCommand(() -> velocity);
    }
    public Command stop() {
        return velocityCommand(() -> 0);
    }
}
