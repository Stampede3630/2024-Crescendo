// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.Configable;

public class Pivot extends SubsystemBase implements Configable {
  /** Creates a new pivot. */

  private TalonFX m_pivotMotor = new TalonFX(17, "CANIVORE");
  @Config(name = "Pivot position")
  private double position = 5;
  private PositionDutyCycle m_positionDutyCycle = new PositionDutyCycle(position);
    public Pivot() {
      m_pivotMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
          .withInverted(InvertedValue.Clockwise_Positive))
      );

    }
 
    public Command positionCommand(DoubleSupplier _position) {
      return Commands.startEnd(() -> m_pivotMotor.setControl(m_positionDutyCycle.withPosition(_position.getAsDouble())), () -> {}, this);
    }

    public Command run() {
      return positionCommand(() -> position);
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
