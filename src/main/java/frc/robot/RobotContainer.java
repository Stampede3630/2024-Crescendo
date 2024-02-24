// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.util.ConfigManager;
import monologue.Logged;
import monologue.Monologue;
import frc.robot.subsystems.*;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged{
  private double MaxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private double MaxAngularRate = 1.5 * Math.PI;
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Pivot m_pivot = new Pivot();
  private final Shooter m_shooter = new Shooter();
  private final Intake m_intake = new Intake();
  private final Indexer m_indexer = new Indexer();
  private final LEDs m_leds = LEDs.getInstance(0, 10);
  private final Pneumatics m_pneumatics = new Pneumatics();
  private final SideBySide m_sideBySide = new SideBySide();
  private final I2CDisplay m_display = new I2CDisplay();
  private ConfigManager cm;

  // private final Pneumatics m_lift = new Pneumatics();
  // private final PhotonVision m_pv = new PhotonVision("Camera_Module_v1", new Transform3d(0,0.02241,0.05026, new Rotation3d()));
    // private final PhotonVision m_pv2 = new PhotonVision("PV2", new Transform3d());


  // Replace with CommandPS4Controller or Commandm_driverController if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`


    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    new Trigger(DriverStation::isEnabled)
      .onTrue(Commands.runOnce(() -> {m_leds.setRGB(255, 90, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("ENABLED")))
      .onFalse(Commands.runOnce(() -> {m_leds.setRGB(0, 255, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("DISABLED")).ignoringDisable(true));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    m_driverController.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);  

    m_driverController.x().whileTrue(m_pivot.run());

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
    m_driverController.leftTrigger()
      .whileTrue(
                m_shooter.run().raceWith(Commands.waitSeconds(2))
                        .andThen(
                                m_indexer.run()
                                .alongWith(m_sideBySide.run())

                        )
      );
//      .whileFalse(m_shooter.stop());
    m_driverController.povLeft().whileTrue(m_pivot.left()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    m_driverController.povRight().whileTrue(m_pivot.right()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    m_driverController.povUp().whileTrue(m_pivot.resetToZero());

    m_driverController.rightTrigger()
      .whileTrue(
              m_intake.run()
              .alongWith(m_indexer.run()
      ));
//      .whileFalse(m_intake.stop().alongWith(m_indexer.stop()));

    m_driverController.y()
      .whileTrue(m_pneumatics.up());
    m_driverController.x()
      .whileTrue(m_pneumatics.down());
    
  }

  
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
//    Logger.configureLoggingAndConfig(this, false);
    m_leds.setRGB(0,0,255);
    m_leds.setMode(LEDMode.SOLID);
    cm = new ConfigManager("HelloTable");
    cm.configure(this);    
    Monologue.setupMonologue(this, "/Robot", false, false);

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.Commandm_driverController Flight
   * m_driverControllers}.
   */

    


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
