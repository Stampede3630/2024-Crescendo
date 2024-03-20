// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SB_TAB;

public class Shooter extends SubsystemBase implements Configable {
    /**
     * Creates a new Shooter.
     */
    private final TalonFX m_shootMotor = new TalonFX(14, "CANIVORE");
    @Config(name = "Shooter Velocity (62)")
    private double velocitySp = 62;
    private static final Shooter instance = new Shooter();
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0, true, false, false, false);
    private final VelocityTorqueCurrentFOC m_velocityOut = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false); // TODO: tune this
    private final TorqueCurrentFOC m_sysidControl = new TorqueCurrentFOC(0);
    private final StatusSignal<Double> m_velocity;
    private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            Volts.per(Second).of(5),         // Default ramp rate is acceptable
            Volts.of(40), // Reduce dynamic voltage to 4 to prevent motor brownout
            null,          // Default timeout is acceptable
            // Log state with Phoenix SignalLogger class
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (Measure<Voltage> volts) -> {
                m_shootMotor.setControl(m_sysidControl.withOutput(volts.in(Volts)));
            },
            null,
            this));

    private Shooter() {

        m_shootMotor.getConfigurator().apply(new TalonFXConfiguration() // TODO: Tune PID and add to slot 0
            .withMotorOutput(new MotorOutputConfigs()
                .withNeutralMode(NeutralModeValue.Coast)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withSlot0(new Slot0Configs()
                .withKA(1.0475)
                .withKV(0.29)
                .withKS(12)
                .withKP(5)
            ).withSlot1(new Slot1Configs()
                .withKA(1.0475)
                .withKS(9.6705)
                .withKV(0.15)
                .withKP(8)
            )
        );
        m_velocity = m_shootMotor.getVelocity();
        m_velocity.setUpdateFrequency(250);

        SB_TAB.addDouble("shooterSpeed", m_velocity::getValue);
        SB_TAB.addBoolean("shooterUpToSpeed", this::upToSpeed);

        super.setDefaultCommand(stop());

    }

    public static Shooter getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        m_velocity.refresh();
    }

    public Command dynamic(Direction direction) {
        return m_sysIdRoutine.dynamic(direction).alongWith(Commands.print("D"));
    }

    public Command quasistatic(Direction direction) {
        return m_sysIdRoutine.quasistatic(direction).alongWith(Commands.print("Q"));
    }

    public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
        return startEnd(() -> m_shootMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {
        });
    }

    public Command velocityCommand(DoubleSupplier _velocity) {
        return startEnd(() -> {
            if (_velocity.getAsDouble() < 0) {
                m_shootMotor.setControl(m_velocityOut.withVelocity(_velocity.getAsDouble()).withSlot(1));
            } else {
                m_shootMotor.setControl(m_velocityOut.withVelocity(_velocity.getAsDouble()).withSlot(0));
            }
        }, () -> {
        });
    }

    public Command run() {
        return velocityCommand(() -> velocitySp);
    }

    public Command idle() {
        return velocityCommand(() -> 20);
    }

    public Command reverse() {
        return dutyCycleCommand(() -> -.4);
    }


    public Command stop() {
        return velocityCommand(() -> 0);
    }

    public boolean upToSpeed() {
        return Math.abs(m_velocity.refresh().getValue()) > velocitySp - 5;
    }

    public Command autoIdle() {
        return dutyCycleCommand(() -> .8);
    }
}
