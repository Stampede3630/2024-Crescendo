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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.util.ConfigManager;
import monologue.Logged;
import monologue.Monologue;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer implements Logged{
  private final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private final double maxAngularRate = 1.5 * Math.PI;
  private final Pivot m_pivot = Pivot.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Indexer m_indexer = Indexer.getInstance();
  private final LEDs m_leds = LEDs.getInstance();
  private final Pneumatics m_pneumatics = Pneumatics.getInstance();
  private final SideBySide m_sideBySide = SideBySide.getInstance();
  private final I2CDisplay m_display = I2CDisplay.getInstance();
  private ConfigManager cm;
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeed * 0.1).withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle();
  private final Telemetry logger = new Telemetry(maxSpeed);
  /** The container for the robot. Contains subsystems, OI devices, and commands. */

  private void configureBindings() {
    new Trigger(DriverStation::isEnabled)
      .onTrue(Commands.runOnce(() -> {m_leds.setRGB(255, 90, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("ENABLED")))
      .onFalse(Commands.runOnce(() -> {m_leds.setRGB(0, 255, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("DISABLED")).ignoringDisable(true));

    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() -> drive.withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-m_driverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
        ));

    // m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
    // m_driverController.b().whileTrue(drivetrain
    //     .applyRequest(() -> point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))));

    // reset the field-centric heading on left bumper press
    m_driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);  

    m_driverController.b().whileTrue(
      m_intake.outtake()
              .alongWith(m_indexer.reverse(), m_sideBySide.reverse(), m_shooter.reverse())
    );

    m_driverController.rightStick() // back right button, align to the source-
            .whileTrue(drivetrain.applyRequest(() -> faceAngle.withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
                    .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
            .withTargetDirection(DriverStation.getAlliance().orElse(Alliance.Red).equals(Alliance.Red) ? Rotation2d.fromDegrees(60) : Rotation2d.fromDegrees(300))
        ));
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.leftTrigger()
      .whileTrue(
                m_shooter.run()
                .alongWith(
                  Commands.waitUntil(m_shooter::upToSpeed)
                    .withTimeout(3)
                  .andThen(m_indexer.run()
                    .alongWith(m_sideBySide.run()))
                )
      );
//      .whileFalse(m_shooter.stop());
    m_driverController.povUp().whileTrue(m_pivot.left()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    m_driverController.povDown().whileTrue(m_pivot.right()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    m_driverController.povLeft().whileTrue(m_pivot.resetToZero());

    m_driverController.rightTrigger()
      .whileTrue(
              m_intake.run()
              .alongWith(m_indexer.run()
      ));
//      .whileFalse(m_intake.stop().alongWith(m_indexer.stop()));

    m_driverController.a().whileTrue(m_shooter.reverse().alongWith(m_sideBySide.run(), m_indexer.run(), m_intake.run()));
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
    // return new PathPlannerAuto("New Auto");
    // return m_shooter.run()
    //             .alongWith(Commands.waitUntil(m_shooter::upToSpeed).withTimeout(3).
    //             andThen(m_indexer.run()
    //                             .alongWith(m_sideBySide.run()))
    //             ).andThen(Commands.waitSeconds(5));
    return Commands.none();
  }
}
