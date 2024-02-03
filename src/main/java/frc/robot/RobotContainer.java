// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.Logger;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final LEDs m_leds = LEDs.getInstance(0, 10);
  private final Pneumatics m_lift = new Pneumatics();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    Logger.configureLoggingAndConfig(this, false);
    m_leds.setRGB(0,0,255);
    m_leds.setMode(LEDMode.SOLID);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new Trigger(DriverStation::isEnabled)
      .onTrue(Commands.runOnce(() -> {m_leds.setRGB(255, 90, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("ENABLED")))
      .onFalse(Commands.runOnce(() -> {m_leds.setRGB(0, 255, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("DISABLED")).ignoringDisable(true));


   

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // these should NOT need to be run "reapeatedly"
    m_driverController.rightTrigger()
      .whileTrue(m_shooter.run())
      .whileFalse(m_shooter.stop());
    
    m_driverController.leftTrigger()
      .whileTrue(m_intake.run().alongWith(m_indexer.run()))
      .whileFalse(m_intake.stop().alongWith(m_indexer.stop()));
    
  }
  

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
