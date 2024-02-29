// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import java.util.function.DoubleSupplier;


public class Intake extends SubsystemBase implements Configable {
    private static final Intake instance = new Intake();
    private final TalonFX m_intakeMotor = new TalonFX(16, "CANIVORE");
    private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0, true, false, false, false);
    @Config(name = "Intake Velocity")
    private double dutyCycle = .8;

    /**
     * Creates a new Intake.
     */

    private Intake() {
        m_intakeMotor.getConfigurator().apply(new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Coast)
            .withInverted(InvertedValue.CounterClockwise_Positive)));
        m_intakeMotor.setNeutralMode(NeutralModeValue.Coast);
        super.setDefaultCommand(stop());

    }

    public static Intake getInstance() {
        return instance;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    //  @Config.NumberSlider(defaultValue = .654)
    public void setDutyCycle(double dutyCycle) {
        this.dutyCycle = dutyCycle;
    }


    public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
        return startEnd(() -> m_intakeMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {
        });
    }

    public Command outtake() {
        return dutyCycleCommand(() -> -dutyCycle);
    }

    public Command run() {
        return dutyCycleCommand(() -> dutyCycle);
    }

    public Command stop() {
        return dutyCycleCommand(() -> 0);
    }
}
