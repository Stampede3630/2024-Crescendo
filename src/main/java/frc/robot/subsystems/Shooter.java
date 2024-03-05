// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

// import au.grapplerobotics.ConfigurationFailedException;
// import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.Config;
import frc.robot.util.Configable;

import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase implements Configable {
  /** Creates a new Shooter. */
  private final TalonFX m_shootMotor = new TalonFX(14, "CANIVORE");
  @Config(name = "Shooter duty cycle")
  private double dutyCycle = .8;
  private static final Shooter instance = new Shooter();
  private final DutyCycleOut m_dutyCycleOut = new DutyCycleOut(0,true,false,false,false);
  private final VelocityTorqueCurrentFOC m_velocityOut = new VelocityTorqueCurrentFOC(0, 0, 0, 0, false, false, false); // TODO: tune this
  private VoltageOut m_sysidControl;
  // private final SysIdRoutine m_sysIdRoutine = new SysIdRoutine(
  //         new SysIdRoutine.Config(
  //                 null,         // Default ramp rate is acceptable
  //                 Volts.of(4), // Reduce dynamic voltage to 4 to prevent motor brownout
  //                 null,          // Default timeout is acceptable
  //                 // Log state with Phoenix SignalLogger class
  //                 (state)-> SignalLogger.writeString("state", state.toString())),
          // new SysIdRoutine.Mechanism(
          //         (Measure<Voltage> volts)-> m_shootMotor.setControl(m_sysidControl.withOutput(volts.in(Volts))),
          //         null,
          //         this));
  @Config(name="Shooter idle on?")
  private boolean idleEnable = true;
  // private final LaserCan m_lc = new LaserCan(23); // TODO: GET THIS ID

  private Shooter() {
    m_shootMotor.getConfigurator().apply(new TalonFXConfiguration() // TODO: Tune PID and add to slot 0
      .withMotorOutput(new MotorOutputConfigs()
        .withNeutralMode(NeutralModeValue.Coast)
        .withInverted(InvertedValue.Clockwise_Positive))
    );
    super.setDefaultCommand(idle());
    // config lasercan
//     try {
//       m_lc.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
//       m_lc.setRangingMode(LaserCan.RangingMode.SHORT);
// //          m_lc.setRegionOfInterest(new LaserCan.RegionOfInterest());
//     } catch (ConfigurationFailedException e) {
//       throw new RuntimeException(e);
//     }
  }

  public static Shooter getInstance() {
    return instance;
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setDutyCycle(double dutyCycle) {
    this.dutyCycle = dutyCycle;
  }


  public Command dutyCycleCommand(DoubleSupplier _dutyCycle) {
    return startEnd(() -> m_shootMotor.setControl(m_dutyCycleOut.withOutput(_dutyCycle.getAsDouble())), () -> {});
  }

  public Command velocityCommand(DoubleSupplier _velocity) {
    return startEnd(() -> m_shootMotor.setControl(m_velocityOut.withVelocity(_velocity.getAsDouble())), () -> {
    });
  }

  public Command run() {
    return dutyCycleCommand(() -> dutyCycle);
    
  }

  public Command idle(){
    return dutyCycleCommand(()->idleEnable ? 0.2 : 0);
  }

  public Command reverse() {
    return dutyCycleCommand(() -> -0.65);
  }

  public Command stop() {
    return dutyCycleCommand(() -> 0);
  }

  public boolean upToSpeed() {
    return m_shootMotor.getVelocity().refresh().getValue()>50;
  }
}
