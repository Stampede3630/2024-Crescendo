// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Config;
import frc.robot.util.Configable;
import monologue.Logged;
import monologue.Annotations.Log;

public class Pivot extends SubsystemBase implements Configable, Logged {
  /** Creates a new pivot. */

  private TalonFX m_pivotMotor = new TalonFX(17, "CANIVORE");

  // private SysIdRoutine
  @Config(name = "Pivot position")
  private double position = 5;
  @Config(name = "Num Pivot DutyCycle beans (0 to 1)")
  private double dutyCycle = .2;
  private PositionDutyCycle m_positionDutyCycle = new PositionDutyCycle(position);
  private DutyCycleOut m_dutyCycle = new DutyCycleOut(0, true, false, false, false);

  // PIVOT RANGE OF MOTION IS 58.17431640625 pm 1.0ish
  public Pivot() {
    m_pivotMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)));
  }

  private Function<Double, Double> rollDegreesToPosition = (angle) -> 0.001761*angle*angle + 0.1123*angle -4.363 + 5.817; // TODO, recalibrate pigeon s.t. our 0 angle is at one of the hard stops or parallel with robot frame

  public Command resetToPigeon() {
    return Commands.runOnce(() -> m_pivotMotor.setPosition(rollDegreesToPosition.apply(pigeonRoll())));
  }
  @Log
  public double pigeonRoll() {
    return Math.toDegrees(TunerConstants.DriveTrain.getPigeon2().getRotation3d().getX());
  }

  public Command save() {
    return Commands.runOnce(() -> {
      Quaternion q = TunerConstants.DriveTrain.getPigeon2().getRotation3d().getQuaternion();
      log("wowie", q.getW()+","+q.getX()+","+q.getY()+","+q.getZ()+","+m_pivotMotor.getPosition());
    });
  }

  @Log.NT
  public double getPosition() {
    return m_pivotMotor.getPosition().refresh().getValueAsDouble();
  }

  public Command resetToZero() {
    return runOnce(() -> m_pivotMotor.setPosition(0)).andThen(Commands.print("RESET pivot position"));
  }
  public Command positionCommand(DoubleSupplier _position) {
    return Commands.startEnd(() -> m_pivotMotor.setControl(m_positionDutyCycle.withPosition(_position.getAsDouble())),
        () -> {
        }, this);
  }

  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_pivotMotor.setControl(m_dutyCycle.withOutput(_dutyCycle.getAsDouble())), () -> {});
  }

  public Command left() {
    return dutyCycleCommand(() -> dutyCycle);
  }
  public Command right() {
    return dutyCycleCommand(() -> -dutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
