// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Config;
import frc.robot.util.Configable;
import frc.robot.util.DoubleLookupLerp;
import monologue.Annotations.Log;

import java.util.function.DoubleSupplier;
import java.util.function.Function;

public final class Pivot extends SubsystemBase implements Configable {
  /** Creates a new pivot. */
  private static final Pivot instance = new Pivot();
  private final TalonFX m_pivotMotor = new TalonFX(17, "CANIVORE");
  @Config(name = "Pivot position")
  private double position = 5;
  @Config(name = "Num Pivot DutyCycle beans (0 to 1)")
  private double dutyCycle = .2;
  private final PositionTorqueCurrentFOC m_positionControl = new PositionTorqueCurrentFOC(-5, 0, 0, 0, false, false, false);
  private final DutyCycleOut m_dutyCycle = new DutyCycleOut(0, true, false, false, false);
  /* YPR adjustments = 90.00006866455078, -27.517465591430664, 180.00013732910156 */

  private final Function<Double, Double> rollDegreesToPosition = (angle) -> 0.0180932 * angle * angle + 1.11426 * angle - 47.0389; // TODO, recalibrate pigeon s.t. our 0 angle parallel with robot frame

  private final StringLogEntry myStringLog = new StringLogEntry(DataLogManager.getLog(), "/pivot/angle");
  private double desiredPos = -10;
  // PIVOT RANGE OF MOTION IS 58.17431640625 pm 1.0ish
  private Pivot() {
    m_pivotMotor.getConfigurator().apply(new TalonFXConfiguration()
        .withMotorOutput(new MotorOutputConfigs()
            .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.Clockwise_Positive))
            .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
                    .withForwardSoftLimitEnable(true)
                    .withForwardSoftLimitThreshold(2)
                    .withReverseSoftLimitEnable(true)
                    .withReverseSoftLimitThreshold(-54.26))
            .withSlot0(new Slot0Configs()
                    .withKS(130)
                    .withKP(200))
            .withFeedback(new FeedbackConfigs()
                    .withSensorToMechanismRatio(1))
    );
    m_pivotMotor.setPosition(rollDegreesToPosition.apply(TunerConstants.DriveTrain.getPigeon2().getRoll().refresh().getValue()));
    myStringLog.append("W,X,Y,Z,Roll,Pos");

  }

  public static Pivot getInstance() {
    return instance;
  }


  public Command resetToPigeon() {
    return Commands.runOnce(() -> m_pivotMotor.setPosition(rollDegreesToPosition.apply(pigeonRoll())));
  }
  @Log
  public double pigeonRoll() {
    return Math.toDegrees(TunerConstants.DriveTrain.getPigeon2().getRoll().refresh().getValueAsDouble());
  }

  public Command seedToPigeon() {
    return Commands.runOnce(() -> {
      m_pivotMotor.setPosition(rollDegreesToPosition.apply(TunerConstants.DriveTrain.getPigeon2().getRoll().refresh().getValue()));
    });
  }
  public Command save() {
    return Commands.runOnce(() -> {
      double roll = TunerConstants.DriveTrain.getPigeon2().getRoll().refresh().getValueAsDouble();
      Quaternion q = TunerConstants.DriveTrain.getPigeon2().getRotation3d().getQuaternion();
      myStringLog.append(q.getW() + "," + q.getX() + "," + q.getY() + "," + q.getZ() + "," + roll + "," + m_pivotMotor.getPosition().getValue());
    });
  }

  @Log.NT
  public double getPosition() {
    return m_pivotMotor.getPosition().refresh().getValueAsDouble();
  }

  private Command resetToZero() {
    return runOnce(() -> m_pivotMotor.setPosition(0)).andThen(Commands.print("RESET pivot position")).ignoringDisable(true);
  }
  public Command positionCommand(DoubleSupplier _position) {
    desiredPos = _position.getAsDouble();
    return startEnd(() -> m_pivotMotor.setControl(m_positionControl.withPosition(_position.getAsDouble())), () -> {
    });
  }

  public Command angleCommand(DoubleSupplier _degrees) {
    return positionCommand(() -> rollDegreesToPosition.apply(_degrees.getAsDouble()));
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

  public boolean atPosition() {
    return m_pivotMotor.getPosition().refresh().getValueAsDouble()-desiredPos < .5;
  }
}
