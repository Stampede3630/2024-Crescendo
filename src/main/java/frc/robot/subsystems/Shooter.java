// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Configable;

public class Shooter extends SubsystemBase implements Configable {
  /** Creates a new Shooter. */
  private TalonFX m_shootMotor = new TalonFX(14, "CANIVORE");
  private double velocity = 10;
  public Shooter() {
    m_shootMotor.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Brake)
        .withInverted(InvertedValue.Clockwise_Positive))
    );

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//  @Config.NumberSlider(defaultValue = -.62)
  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }


  public Command velocityCommand(DoubleSupplier _velocity) {
    return Commands.startEnd(() -> m_shootMotor.setControl(new DutyCycleOut(_velocity.getAsDouble())), () -> {});
  }

  public Command run() {
    return velocityCommand(() -> velocity);
    
  }

  public Command stop() {
    return velocityCommand(() -> 0);
  }
}
