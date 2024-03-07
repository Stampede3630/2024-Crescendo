// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs.LEDMode;
import frc.robot.util.AutoCommandFinder;
import frc.robot.util.ConfigManager;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged{
  private final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
  private final double maxAngularRate = 1.5 * Math.PI;
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

//DRIVETRAIN subsystem  
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  private final SwerveRequest.FieldCentric drive = 
    new SwerveRequest.FieldCentric()
      .withDeadband(maxSpeed * 0.1)
      .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop
      
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final SwerveRequest.FieldCentricFacingAngle faceAngle = new SwerveRequest.FieldCentricFacingAngle();
  private final Telemetry logger = new Telemetry(maxSpeed);
  
//ALL teh "UHver" subsystems      
  private final Pivot m_pivot = Pivot.getInstance();
  private final Shooter m_shooter = Shooter.getInstance();
  private final Intake m_intake = Intake.getInstance();
  private final Indexer m_indexer = Indexer.getInstance();
  private final LEDs m_leds = LEDs.getInstance();
  private final Pneumatics m_pneumatics = Pneumatics.getInstance();
  private final SideBySide m_sideBySide = SideBySide.getInstance();
  private final I2CDisplay m_display = I2CDisplay.getInstance();
  private final Amp m_amp = Amp.getInstance();
  private final LaserCanSwitch m_lc = LaserCanSwitch.getInstance();
  private final SendableChooser<Command> autoChooser;


  public RobotContainer() {
    configureBindings();
    //Logger.configureLoggingAndConfig(this, false);
    m_leds.setRGB(0,0,255);
    m_leds.setMode(LEDMode.SOLID);

    ConfigManager cm = new ConfigManager("HelloTable");
    cm.configure(this);
    Monologue.setupMonologue(this, "/Robot", false, false);
    try (PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev)) {
      pdh.setSwitchableChannel(true);
    }
    AutoCommandFinder.addAutos();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);



  }

  private void configureBindings() {
    new Trigger(DriverStation::isEnabled)
      .onTrue(Commands.runOnce(() -> {m_leds.setRGB(255, 90, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("ENABLED")))
      .onFalse(Commands.runOnce(() -> {m_leds.setRGB(0, 255, 0);m_leds.setMode(LEDMode.SOLID);}, m_leds).alongWith(Commands.print("DISABLED")).ignoringDisable(true));
    
//DRIVETRAIN COMMANDS
    drivetrain.setDefaultCommand(drivetrain.applyRequest(() -> drive        // Drivetrain will execute this command periodically
      .withVelocityX(-m_driverController.getLeftY() * maxSpeed)             // Drive forward with negative Y (forward)
      .withVelocityY(-m_driverController.getLeftX() * maxSpeed)             // Drive left with negative X (left)
      .withRotationalRate(-m_driverController.getRightX() * maxAngularRate) // Drive counterclockwise with negative X (left)
      ));

    // reset the field-centric heading on left bumper press
    //TODO: as we get close to the competition, make this a dashboard button rather than a joystick button
    m_driverController.leftBumper()
            .onTrue(drivetrain.runOnce(drivetrain::seedFieldRelative));
    
    //face speaker while shooting sub shot
    m_driverController.rightStick().whileTrue(drivetrain.applyRequest(() -> 
      faceAngle
        .withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
              .withTargetDirection(drivetrain.getCurrentAlliance().equals(Alliance.Red) ? Rotation2d.fromDegrees(60) : Rotation2d.fromDegrees(300))
        ));   // cancelling on release.

    drivetrain.registerTelemetry(logger::telemeterize);
    
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }

    m_driverController.start().whileTrue(AutoCommands.shootSub());

//PIVOT Commands
    //TODO: as we get close to the competition, make this a dashboard button rather than a joystick button
    m_driverController.start().whileTrue(m_pivot.seedToPigeon());
    m_driverController.povUp().whileTrue(m_pivot.left()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    m_driverController.povDown().whileTrue(m_pivot.right()).whileFalse(m_pivot.dutyCycleCommand(() -> 0));
    //m_driverController.povLeft().whileTrue(m_pivot.resetToZero());
    //m_driverController.rightBumper().onTrue(m_pivot.save().andThen(Commands.print("SAVED")));

//INTAKE Commands
    //standard intake
    //TODO: Use Commands.Parallel
    m_driverController.rightTrigger().whileTrue(
      m_intake.run()
      .alongWith(m_indexer.run()
    ));

    //reverse intake
    //TODO: Use Commands.Parallel
    m_driverController.b().whileTrue(
      m_intake.outtake().alongWith(
      m_indexer.reverse(), 
      m_sideBySide.reverse(), 
      m_shooter.reverse()
    ));

//SHOOTER Commands
    //Speaker Shot  
    //TODO: Use Commands.Parallel
    m_driverController.leftTrigger().whileTrue(
      m_shooter.run().alongWith(
      // m_pivot.angleCommand(()->26.0),
      Commands.waitUntil(m_shooter::upToSpeed).withTimeout(3)
        .andThen(m_indexer.run().alongWith(
        m_sideBySide.run()))
    ));

    // m_driverController.leftTrigger().whileTrue(
    //   m_pivot.angleCommand(()->7.9)
    // );
    // m_driverController.leftTrigger().whileTrue(AutoCommands.shoot());

    //Amp Shot
    //TODO: Use Commands.Parallel
    m_driverController.a().whileTrue(
      m_shooter.reverse().alongWith(
      m_pneumatics.up(),
      m_sideBySide.run(), 
      m_indexer.run(), 
      m_intake.run(), 
      m_amp.run()));

    //Amp air
    //TODO: this is probably temp code and will be added to one of the methods above
    m_driverController.y()
      .whileTrue(m_pneumatics.up());
    m_driverController.x()
      .whileTrue(m_pneumatics.down());

  }
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new PathPlannerAuto("New Auto");
    // return m_shooter.run()
    //             .alongWith(Commands.waitUntil(m_shooter::upToSpeed).withTimeout(3).
    //             andThen(m_indexer.run()
    //                             .alongWith(m_sideBySide.run()))
    //             ).andThen(Commands.waitSeconds(5));
    return autoChooser.getSelected();
  }
}
