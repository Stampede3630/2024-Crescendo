// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import io.github.oblarg.oblog.annotations.Config;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private TalonFX m_shootMotor = new TalonFX(2, "CANIVORE");
  public Shooter() {
    m_shootMotor.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast))
    );




  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Config.NumberSlider
  public void setVelocity(double velocity) {
    m_shootMotor.setControl(new VelocityTorqueCurrentFOC(velocity));
  }
}
