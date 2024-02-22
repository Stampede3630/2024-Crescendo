// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase implements Configable {
  /** Creates a new Shooter. */
  private final TalonFX m_shootMotor = new TalonFX(14, "CANIVORE");
  @Config(name = "Shooter velocity")
  private double dutyCycle = .62;
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0,true,false,false,false);
  private VoltageOut m_sysidControl;
  private SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
          new SysIdRoutine.Config(
                  null,         // Default ramp rate is acceptable
                  Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
                  null,          // Default timeout is acceptable
                  // Log state with Phoenix SignalLogger class
                  (state)-> SignalLogger.writeString("state", state.toString())),
          new SysIdRoutine.Mechanism(
                  (Measure<Voltage> volts)-> m_shootMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
                  null,
                  this));
  public Shooter() {
    m_shootMotor.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.CounterClockwise_Positive))
    );
        super.setDefaultCommand(stop());


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//  @Config.NumberSlider(defaultValue = -.62)
public void setDutyCycle(double dutyCycle) {
  this.dutyCycle = dutyCycle;
}


  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_shootMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {});
  }

  public Command run() {
    return dutyCycleCommand(() -> dutyCycle);
    
  }

  public Command stop() {
    return dutyCycleCommand(() -> 0);
  }
}
