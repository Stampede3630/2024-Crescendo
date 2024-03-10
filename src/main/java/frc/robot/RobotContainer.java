// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LEDs;
import frc.robot.util.AutoCommandFinder;
import frc.robot.util.ConfigManager;
import monologue.Logged;
import monologue.Monologue;

import static edu.wpi.first.units.Units.*;

@SuppressWarnings("unused")
public class RobotContainer implements Logged {
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
    private final Amp m_amp = frc.robot.subsystems.Amp.getInstance();
    private final LaserCanSwitch m_lc = LaserCanSwitch.getInstance();
    private final SendableChooser<Command> autoChooser;
    private final PhotonVision m_limelight = new PhotonVision("limelight", new Transform3d(Inches.of(10).in(Meters), Inches.of(8).in(Meters), Inches.of(12.5).in(Meters), new Rotation3d(0, Degrees.of(-28.8).in(Radians), 0)));
    private final PhotonVision m_gs = new PhotonVision("GS", new Transform3d());

    public RobotContainer() {
        configureBindings();

        ConfigManager cm = new ConfigManager("HelloTable");
        cm.configure(this);
        Monologue.setupMonologue(this, "/Robot", false, false);

        try (PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev)) {
            pdh.setSwitchableChannel(true);
        }

        AutoCommandFinder.addAutos();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        m_gs.setEnabled(false);
    }

    private void configureBindings() {
        new Trigger(DriverStation::isEnabled)
                .onTrue(m_leds.setSolidColor(() -> new Color(255, 90, 0)).alongWith(Commands.print("ENABLED")))
                .onFalse(m_leds.setSolidColor(() -> new Color(0, 255, 0)).alongWith(Commands.print("DISABLED")).ignoringDisable(true));

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

        m_driverController.leftStick().whileTrue(m_pneumatics.down());

        drivetrain.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        m_driverController.start().whileTrue(AutoCommands.shootSub());

//PIVOT Commands
        //TODO: as we get close to the competition, make this a dashboard button rather than a joystick button
        m_driverController.start()
                .whileTrue(m_pivot.seedToPigeon());

        m_driverController.povUp()
                .whileTrue(m_pivot.left())
                .whileFalse(m_pivot.dutyCycleCommand(() -> 0));
        m_driverController.povDown()
                .whileTrue(m_pivot.right())
                .whileFalse(m_pivot.dutyCycleCommand(() -> 0));


//INTAKE Commands
        //standard intake
        m_driverController.rightTrigger().whileTrue(
                Commands.parallel(
                                m_intake.run(),
                                m_indexer.run()
                        )
                        .until(m_lc.fullyClosed())
        );

        //reverse intake
        m_driverController.b().whileTrue(
                Commands.parallel(
                        m_intake.outtake(),
                        m_indexer.reverse(),
                        m_sideBySide.reverse(),
                        m_shooter.reverse()
                ));

// LASER CAN STATE
        new Trigger(m_lc.fullyClosed())
                .onTrue(
                        m_leds.blink(Color.kPurple, 3, 0.050, 0.200)
                                .andThen(m_leds.breathe(271, 2.5))
                ).onFalse(
                        m_leds.off()
                );
        new Trigger(m_lc.fullyOpen())
                .onTrue(
                        m_leds.blink(Color.kLime, 3, .050, .200)
                );
//SHOOTER Commands
        // TODO: DO we really want to use LC for this in tele?
        m_driverController.leftTrigger().whileTrue(
                Commands.parallel(m_shooter.run(),
                        Commands.waitUntil(m_shooter::upToSpeed).withTimeout(3)
                                .andThen(m_indexer.run().alongWith(
                                        m_sideBySide.run()))
                                .until(m_lc.fullyOpen())
                ));

        // m_driverController.povDown().whileTrue(m_shooter.quasistatic(Direction.kReverse));
        // m_driverController.povUp().whileTrue(m_shooter.quasistatic(Direction.kForward));
        // m_driverController.povLeft().whileTrue(m_shooter.dynamic(Direction.kReverse));
        // m_driverController.povRight().whileTrue(m_shooter.dynamic(Direction.kForward));

        // m_driverController.leftTrigger().whileTrue(
        //   m_pivot.angleCommand(()->7.9)
        // );
        // m_driverController.leftTrigger().whileTrue(AutoCommands.shoot());

        //Amp Shot
        m_driverController.a().whileTrue(
                Commands.parallel(
                        m_shooter.reverse(),
                        m_pneumatics.up(),
                        m_sideBySide.run(),
                        m_indexer.run(),
                        m_intake.run(),
                        m_amp.run(),
                        m_pivot.angleCommand(() -> 30.3)
                )
        );

        // Pod shot x
        m_driverController.y().whileTrue(
                Commands.parallel(
                        m_pneumatics.down(),
                        m_pivot.angleCommand(() -> 7.4)
                )
        );
        // sub shot y
        m_driverController.y().whileTrue(
                Commands.parallel(
                        m_pneumatics.down(),
                        m_pivot.angleCommand(() -> 26)
                )
        );


    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
