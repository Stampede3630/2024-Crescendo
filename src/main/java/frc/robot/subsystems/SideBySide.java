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
    private double dutyCycle = .65;
    // private double sideDutyCycle = .8;
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0,true,false,false,false);
    public SideBySide() {
        m_motor.getConfigurator().apply(new TalonFXConfiguration()
                .withMotorOutput(new MotorOutputConfigs()
                        .withNeutralMode(NeutralModeValue.Brake)
                        .withInverted(InvertedValue.CounterClockwise_Positive))
        );

        setDefaultCommand(stop());
    }
    public void setDutyCycle(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }


    public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
        return startEnd(() -> m_motor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {});
    }

    public Command run() {
        return dutyCycleCommand(() -> dutyCycle);

    }

    public Command stop() {
        return dutyCycleCommand(() -> 0);
    }
}
