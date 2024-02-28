// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.controls.DutyCycleOut;
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
  private final CANSparkFlex m_ampMotor = new CANSparkFlex(1,MotorType.kBrushless);
  private double dutyCycle = .4;
  public Amp() {

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

  public Command run(){
    return Commands.runOnce(()->m_ampMotor.set(dutyCycle));
  }

  public Command stop(){
    return Commands.runOnce(()->m_ampMotor.set(0));
  }
}
