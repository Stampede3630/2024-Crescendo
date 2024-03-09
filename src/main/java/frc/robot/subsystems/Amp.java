// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import static edu.wpi.first.units.Units.Volts;

import java.util.function.DoubleSupplier;

public class Amp extends SubsystemBase implements Configable{
  private static final Amp instance = new Amp();
  /** Creates a new Amp. */
  private final TalonFX m_ampMotor = new TalonFX(18, "CANIVORE");
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0,true,false,false,false);

  @Config
  private double dutyCycle = 1;
  public Amp() {

    super.setDefaultCommand(stop());

  }

  public static Amp getInstance(){
    return instance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setDutyCycle(double dutyCycle){
    this.dutyCycle = dutyCycle;
  }

  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_ampMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {});
  }

  public Command run(){
    return dutyCycleCommand(()->.5);
  }

  public Command stop(){
    return dutyCycleCommand(()->0.0);
  }
}
