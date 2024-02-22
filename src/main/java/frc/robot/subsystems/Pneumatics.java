// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  /** Creates a new Pneumatics. */
  private PneumaticHub m_ph = new PneumaticHub(2);
  private DoubleSolenoid m_lift = m_ph.makeDoubleSolenoid(0, 2);
  public Pneumatics() {
    m_lift.set(DoubleSolenoid.Value.kOff);
    m_ph.enableCompressorDigital();
    super.setDefaultCommand(off());
  }

  public Command up() {
    return Commands.runOnce(() -> m_lift.set(DoubleSolenoid.Value.kForward), this);
  }
  public Command down() {
    return Commands.runOnce(() -> m_lift.set(DoubleSolenoid.Value.kReverse), this);
  }
  public Command off() {
    return Commands.runOnce(() -> m_lift.set(DoubleSolenoid.Value.kOff), this);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
