// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Config;
import frc.robot.util.Configable;


public class Indexer extends SubsystemBase implements Configable {
    private final TalonFX m_indexMotor = new TalonFX(13, "CANIVORE");
  private static final Indexer instance = new Indexer();
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0, true, false, false, false);
  @Config(name = "Indexer velocity")
  private double dutyCycle = .25; //CHANGE INDEX SPEED

  /** Creates a new Indexer. */
  private Indexer() {
    m_indexMotor.getConfigurator().apply(new TalonFXConfiguration()
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
    );

    setDefaultCommand(stop());
  }

  public static Indexer getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }


//  @Config.NumberSlider(defaultValue = .65)
  public void setDutyCycle(double dutyCycle) {
    this.dutyCycle = dutyCycle;
  }


  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_indexMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {});
  }


  public Command run() {

    return dutyCycleCommand(() -> dutyCycle);
  }
  public Command stop() {
    return dutyCycleCommand(() -> 0);
  }

  public Command reverse() {
    return dutyCycleCommand(() -> -dutyCycle);
  }
}
