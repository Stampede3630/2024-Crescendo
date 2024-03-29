package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.*;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.generated.TunerConstants;

import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.SB_TAB;
import static frc.robot.Constants.SB_TEST;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean hasAppliedOperatorPerspective = false;
    private StatusSignal<Double> yaw;
    private StatusSignal<Double> pitch;
    private StatusSignal<Double> roll;
    private final SwerveRequest.SysIdSwerveTranslation translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveRotation rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();
    private Alliance currentAlliance = Alliance.Red;

    /* Use one of these sysidroutines for your particular test */
    private final SysIdRoutine sysIdRoutineTranslate = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(6),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> setControl(translationCharacterization.withVolts(volts)),
            null,
            this));

    private final SysIdRoutine sysIdRoutineRotation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(4),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> setControl(rotationCharacterization.withVolts(volts)),
            null,
            this));
    private final SysIdRoutine sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            Volts.of(7),
            null,
            (state) -> SignalLogger.writeString("state", state.toString())),
        new SysIdRoutine.Mechanism(
            (volts) -> setControl(steerCharacterization.withVolts(volts)),
            null,
            this));

    /* Change this to the sysid routine you want to test */
    private final SendableChooser<SysIdRoutine> sysIdRoutineSendableChooser = new SendableChooser<>();

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
                                   SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        initialize();
    }

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        initialize();
    }

    private void initialize() {
        getPigeon2().reset();
        // seedFieldRelative();
        configurePathPlanner();
        if (Utils.isSimulation()) {
            startSimThread();
        }
        yaw = getPigeon2().getYaw();
        pitch = getPigeon2().getPitch();
        roll = getPigeon2().getRoll();
        BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, pitch, roll);
        // BaseStatusSignal.setUpdateFrequencyForAll(250, )
        SB_TAB.addNumber("Yaw", () -> yaw.getValueAsDouble());
        SB_TAB.addNumber("Pitch", () -> pitch.getValueAsDouble());
        SB_TAB.addNumber("Roll", () -> roll.getValueAsDouble());
        SB_TAB.addNumber("X", ()-> TunerConstants.DriveTrain.m_odometry.getEstimatedPosition().getX());
        SB_TAB.addNumber("Y", ()-> TunerConstants.DriveTrain.m_odometry.getEstimatedPosition().getY());

        SB_TAB.addDoubleArray("CTRE pose", () -> new double[]{getState().Pose.getX(), getState().Pose.getY(), getState().Pose.getRotation().getRadians()});

        sysIdRoutineSendableChooser.addOption("translate", sysIdRoutineTranslate);
        sysIdRoutineSendableChooser.addOption("steer", sysIdRoutineSteer);
        sysIdRoutineSendableChooser.addOption("rotation", sysIdRoutineRotation);
        sysIdRoutineSendableChooser.setDefaultOption("translate", sysIdRoutineTranslate);
        SB_TEST.add(sysIdRoutineSendableChooser);
        SB_TEST.add("Static forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        SB_TEST.add("Static reverse", sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        SB_TEST.add("Dynamic forward", sysIdDynamic(SysIdRoutine.Direction.kForward));
        SB_TEST.add("Dynamic reverse", sysIdDynamic(SysIdRoutine.Direction.kReverse));
        SB_TEST.add("coast", setCoast());
        SB_TEST.addNumber("Translational Speed", () -> {ChassisSpeeds speeds = getState().speeds; return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);});
    }

    public StatusSignal<Double> getYaw() {
        return yaw;
    }

    public StatusSignal<Double> getPitch() {
        return pitch;
    }

    public StatusSignal<Double> getRoll() {
        return roll;
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /*
     * Both the sysid commands are specific to one particular sysid routine, change
     * which one you're trying to characterize
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return sysIdRoutineSendableChooser.getSelected().quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutineSendableChooser.getSelected().dynamic(direction);
    }

    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return m_kinematics.toChassisSpeeds(getState().ModuleStates);
    }

    public Command setCoast() {
        return startEnd(()-> setCoastHelper(), ()-> {}).ignoringDisable(true);
    }

    public void setCoastHelper() {
        // TunerConstants.DriveTrain.getModule(0).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(1).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(2).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(3).getDriveMotor().setNeutralMode(NeutralModeValue.Coast);

        // TunerConstants.DriveTrain.getModule(0).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(1).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(2).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        // TunerConstants.DriveTrain.getModule(3).getSteerMotor().setNeutralMode(NeutralModeValue.Coast);
        List.of(Modules).forEach(m -> m.configNeutralMode(NeutralModeValue.Coast));
    }

    public void setBrakeHelper() {
        // TunerConstants.DriveTrain.getModule(0).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(1).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(2).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(3).getDriveMotor().setNeutralMode(NeutralModeValue.Brake);

        // TunerConstants.DriveTrain.getModule(0).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(1).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(2).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        // TunerConstants.DriveTrain.getModule(3).getSteerMotor().setNeutralMode(NeutralModeValue.Brake);
        List.of(Modules).forEach(m -> m.configNeutralMode(NeutralModeValue.Brake));
    }


    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
            () -> this.getState().Pose, // Supplier of current robot pose
            this::seedFieldRelative, // Consumer for seeding pose against auto
            this::getCurrentRobotChassisSpeeds,
            (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
            // robot
            new HolonomicPathFollowerConfig(new PIDConstants(10, 0, 0),
                new PIDConstants(10, 0, 0),
                TunerConstants.kSpeedAt12VoltsMps,
                driveBaseRadius,
                new ReplanningConfig()),
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red
                // alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                Optional<Alliance> alliance = DriverStation.getAlliance();
                return alliance.filter(value -> value == Alliance.Red).isPresent();
            }, // Change this if the path needs to be flipped on red vs blue
            this); // Subsystem for requirements
    }

    @Override
    public void periodic() {

        /* Periodically try to apply the operator perspective */
        /*
         * If we haven't applied the operator perspective before, then we should apply
         * it regardless of DS state
         */
        /*
         * This allows us to correct the perspective in case the robot code restarts
         * mid-match
         */
        /*
         * Otherwise, only check and apply the operator perspective if the DS is
         * disabled
         */
        /*
         * This ensures driving behavior doesn't change until an explicit disable event
         * occurs during testing
         */
        if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent((allianceColor) -> {
                currentAlliance = allianceColor;
                this.setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red ? RedAlliancePerspectiveRotation
                        : BlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
            });
        }
    }

    public Alliance getCurrentAlliance() {
        return currentAlliance;
    }
}
