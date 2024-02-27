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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Config;
import frc.robot.util.Configable;
import monologue.Logged;
import monologue.Annotations.Log;

public class Pivot extends SubsystemBase implements Configable {
  /** Creates a new pivot. */
  private static final Pivot instance = new Pivot();
  private final TalonFX m_pivotMotor = new TalonFX(17, "CANIVORE");
  @Config(name = "Pivot position")
  private double position = 5;
  @Config(name = "Num Pivot DutyCycle beans (0 to 1)")
  private double dutyCycle = .2;
  private final PositionDutyCycle m_positionDutyCycle = new PositionDutyCycle(position);
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0, true, false, false, false);
  /* YPR adjustments = 90.00006866455078, -27.517465591430664, 180.00013732910156 */
  private final Function<Double, Double> rollDegreesToPosition = (angle) -> 0.0180932 * angle * angle + 1.11426 * angle - 47.0389; // TODO, recalibrate pigeon s.t. our 0 angle is at one of the hard stops or parallel with robot frame
  private StringLogEntry myStringLog = new StringLogEntry(DataLogManager.getLog(), "/pivot/angle");
  // PIVOT RANGE OF MOTION IS 58.17431640625 pm 1.0ish
  private Pivot() {
    m_pivotMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
            .withInverted(InvertedValue.Clockwise_Positive)));
    myStringLog.append("W,X,Y,Z,Yaw,Pitch,Roll,Pos");

  }

  public static Pivot getInstance() {
    return instance;
  }


  public Command resetToPigeon() {
    return Commands.runOnce(() -> m_pivotMotor.setPosition(rollDegreesToPosition.apply(pigeonRoll())));
  }
  @Log
  public double pigeonRoll() {
    return Math.toDegrees(TunerConstants.DriveTrain.getPigeon2().getRotation3d().getX());
  }

  public Command seedToPigeon() {
    return Commands.runOnce(() -> {
      m_pivotMotor.setPosition(rollDegreesToPosition.apply(TunerConstants.DriveTrain.getPigeon2().getRoll().refresh().getValue()));
    });
  }
  public Command save() {
    return Commands.runOnce(() -> {
      Rotation3d rot = TunerConstants.DriveTrain.getPigeon2().getRotation3d();
      Quaternion q = rot.getQuaternion();
      myStringLog.append(q.getW()+","+q.getX()+","+q.getY()+","+q.getZ()+","+rot.getZ()+","+rot.getY()+","+rot.getX()+","+m_pivotMotor.getPosition().getValue());
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
