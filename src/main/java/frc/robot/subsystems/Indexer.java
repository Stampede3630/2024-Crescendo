// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.Configable;


public class Indexer extends SubsystemBase implements Configable {
      private TalonFX m_indexMotor = new TalonFX(13, "CANIVORE");
      @Config(name = "Indexer velocity")
    private double velocity = .3;

  /** Creates a new Indexer. */
  public Indexer() {
    m_indexMotor.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
    );

    setDefaultCommand(stop());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

//  @Config.NumberSlider(defaultValue = .65)
  public void setVelocity(double velocity) {
    this.velocity = velocity;
  }


  public Command velocityCommand(DoubleSupplier _velocity) {
    return startEnd(() -> m_indexMotor.setControl(new DutyCycleOut(_velocity.getAsDouble())), () -> {});
  }


  public Command run() {
//    return Commands.startEnd(()-> {
//      m_indexMotor.setControl(new DutyCycleOut(velocity));
//  },() -> {}, this
//    );
    return velocityCommand(() -> velocity);
  }
  public Command stop() {
    return velocityCommand(() -> 0);
  }
}
