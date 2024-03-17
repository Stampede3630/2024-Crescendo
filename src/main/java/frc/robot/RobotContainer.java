// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.FaceAngleRequestBetter;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.util.AutoCommandFinder;
import frc.robot.util.ConfigManager;
import frc.robot.util.TimeElapsedTrigger;
import monologue.Logged;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.SB_TAB;
import static frc.robot.Constants.SB_TEST;

@SuppressWarnings("unused")
public class RobotContainer implements Logged {

    private final double maxSpeed = TunerConstants.kSpeedAt12VoltsMps;
    private final double maxAngularRate = 1.5 * Math.PI;
    private final CommandXboxController m_driverController = new CommandXboxController(
            OperatorConstants.kDriverControllerPort);

            private TimeElapsedTrigger intakeTimer = new TimeElapsedTrigger(60_000);

    // DRIVETRAIN subsystem
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.DriveTrain;
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(maxSpeed * 0.1)
            .withRotationalDeadband(maxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric driving in open loop

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final FaceAngleRequestBetter faceAngle = new FaceAngleRequestBetter();
    private final Telemetry logger = new Telemetry(maxSpeed);

    // ALL teh "UHver" subsystems
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
    private final PhotonVision m_limelight = new PhotonVision("limelight", new Transform3d(Inches.of(10).in(Meters),
            Inches.of(8).in(Meters), Inches.of(12.5).in(Meters), new Rotation3d(0, Degrees.of(-28.8).in(Radians), 0)));
    private final PhotonVision m_gs = new PhotonVision("GS",
            new Transform3d(Inches.of(10.25).in(Meters), Inches.of(-10).in(Meters), Inches.of(9.25).in(Meters),
                    new Rotation3d(0, Degrees.of(-24.09).in(Radians), Degrees.of(-30).in(Radians))));
    private final PhotonVision m_ar2 = new PhotonVision("AR2",
            new Transform3d(Inches.of(10.25).in(Meters), Inches.of(10).in(Meters), Inches.of(9.25).in(Meters),
                    new Rotation3d(0, Degrees.of(-24.09).in(Radians), Degrees.of(30).in(Radians))));

    public RobotContainer() {
        configureBindings();

        ConfigManager cm = new ConfigManager("HelloTable");
        cm.configure(this);
        PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

        AutoCommandFinder.addAutos();
        autoChooser = AutoBuilder.buildAutoChooser();
        SB_TAB.add("Auto Chooser", autoChooser);

        m_gs.setEnabled(false);

        m_ar2.setEnabled(false);
        m_limelight.setEnabled(false);

        SB_TAB.addNumber("Laser Can Value", m_lc::laserCan);
        SB_TAB.add(m_pneumatics.up().alongWith(m_pivot.angleCommand(() -> 17)).withName("Legal Start"));
        SB_TEST.add(m_leds.blinkConstant(Color.kRed, 1, 3).withName("Blink test leds"));
        m_leds.rainbow().ignoringDisable(true).schedule();
        SB_TAB.addBoolean("in amp region", () -> AMP_REGION.get().inRegion(m_drivetrain.getState().Pose.getTranslation()));
        SB_TEST.addNumber("PDH TOTAL CURRENT (A)", pdh::getTotalCurrent);
    }

    private void configureBindings() {
        new Trigger(DriverStation::isEnabled)
                .onTrue(
                        m_leds.setSolidColor(() -> new Color(255, 90, 0))
                                .alongWith(Commands.print("ENABLED")))
                .onFalse(
                        m_leds.setSolidColor(() -> new Color(0, 255, 0))
                                .alongWith(Commands.print("DISABLED"))
                                .ignoringDisable(true));
                // .whileFalse(m_drivetrain.setCoast());

        // DRIVETRAIN COMMANDS
        m_drivetrain.setDefaultCommand(m_drivetrain.applyRequest(() -> drive // Drivetrain will execute this command
                                                                             // periodically
                .withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                .withRotationalRate(-m_driverController.getRightX() * maxAngularRate) // Drive counterclockwise with
                                                                                      // negative X (left)
        ));

        // face speaker or ampdriveFaceAngle(PODIUM_HEADING)
        m_driverController.rightStick()
                .whileTrue(
                        Commands.either(
                                driveFaceAngle(AMP_ORIENTATION),
                                // Commands.none(),
                                // driveFaceAngle(() -> m_drivetrain.getState().Pose.getTranslation().minus(SPEAKER_POSITION.get().toTranslation2d()).getAngle()),
                        
                                driveFaceAngle(() -> m_limelight.robotToSpeaker().map(t -> t.getRotation().toRotation2d()).orElse(m_drivetrain.getState().Pose.getRotation())).onlyIf(m_limelight::seeTheSpeaker),
                                m_pneumatics.isUp()
                        )
                        // m_pivot.angleCommand(() -> {
                        // // TODO DO LOOKUP TABLE OR MATH OR SOMETHING
                        // return 20; // dummy number
                        // })
                        );

        // reset the field-centric heading on start button
        m_driverController.start()
                .onTrue(m_drivetrain.runOnce(m_drivetrain::seedFieldRelative));

        m_drivetrain.registerTelemetry(logger::telemeterize);

        if (Utils.isSimulation()) {
            m_drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
        }

        // PIVOT Commands
        // TODO: as we get close to the competition, make this a dashboard button rather
        // than a joystick button

        m_driverController.povUp()
                .whileTrue(m_pivot.left())
                .whileFalse(m_pivot.dutyCycleCommand(() -> 0));
        m_driverController.povDown()
                .whileTrue(m_pivot.right())
                .whileFalse(m_pivot.dutyCycleCommand(() -> 0));

        // INTAKE Commands
        // standard intake
        m_driverController.rightTrigger().whileTrue(
                Commands.parallel(
                        m_intake.run(),
                        m_indexer.run(), Commands.runOnce(() -> intakeTimer.start()))
                        .until(m_lc.fullyClosed().and(intakeTimer)));

        // reverse intake
        m_driverController.b().whileTrue(
                Commands.parallel(
                        m_intake.outtake(),
                        m_indexer.reverse(),
                        m_sideBySide.reverse(),
                        m_shooter.reverse()));

        // LASER CAN STATE
        m_lc.fullyClosed()
                .onTrue(
                        Commands.parallel(
                                m_leds.setSolidColor(() -> Color.kPurple),
                                m_limelight.blink().withTimeout(1.5), 
                                Commands.startEnd(() -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 1), () -> m_driverController.getHID().setRumble(RumbleType.kBothRumble, 0)).withTimeout(0.3)
                        )
                )
                .onFalse(
                        m_leds.setSolidColor(() -> Color.kOrange));
        // m_lc.fullyOpen()
                // .onTrue(
                        // m_leds.blink(Color.kLime, 3, .050, .200));
        new Trigger(() -> m_pivot.atPosition() && faceAngle.atTarget())
                .onTrue(
                        m_leds.blinkConstant(Color.kBlue, .050, .4));
        // SHOOTER Commands
        // TODO: DO we really want to use LC for this in tele?
        m_driverController.leftTrigger().whileTrue(
                Commands.parallel(m_shooter.run(),
                        Commands.waitUntil(m_shooter::upToSpeed).withTimeout(1)
                                .andThen(
                                        Commands.parallel(m_indexer.run(), m_sideBySide.run()))
                                .until(m_lc.fullyOpen())));

        // Amp Shot
        m_driverController.a().whileTrue(
                Commands.parallel(
                        m_shooter.reverse(),
                        m_pneumatics.up(),
                        m_sideBySide.run(),
                        m_indexer.run(),
                        m_intake.run(),
                        m_amp.run(),
                        m_pivot.angleCommand(() -> 30.3)));
        m_driverController.leftStick().whileTrue(m_pneumatics.down());
        // put pneumatics down when we leave the amp region
        new Trigger(() -> AMP_REGION.get().inRegion(m_drivetrain.getState().Pose.getTranslation()))
                .onFalse(m_pneumatics.down());

        // Pod shot left bumper
        m_driverController.leftBumper().whileTrue(
                Commands.parallel(
                        m_pneumatics.down(),
                    m_pivot.angleCommand(() -> 7.8),
                    driveFaceAngle(PODIUM_HEADING)
                ));
        // sub shot y
        m_driverController.y().whileTrue(
                Commands.parallel(
                        m_pneumatics.down(),
                        m_pivot.angleCommand(() -> 27)));
        
        // m_limelight.robotToSpeaker().ifPresent(t -> t.getRotation());
    }

    private Command driveFaceAngle(Supplier<Rotation2d> _rotation) {
        return m_drivetrain.applyRequest(() -> faceAngle
                .withVelocityX(-m_driverController.getLeftY() * maxSpeed) // Drive forward with negative Y (forward)
                .withVelocityY(-m_driverController.getLeftX() * maxSpeed) // Drive left with negative X (left)
                .withTargetDirection(_rotation.get()));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
