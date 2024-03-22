// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.generated.TunerConstants;
import frc.robot.util.Config;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;
import java.util.function.Function;

public class PhotonVision extends SubsystemBase{
    private static final ShuffleboardTab SB_PV_TAB = Shuffleboard.getTab("Photon Vision");
    @Config(name = "PhotonVision Enabled")
    private static boolean dashboardVisionEnabled = true;
    /**
     * Creates a new PhotonVision.
     */
    private final PhotonCamera camera;
    private final AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
    private final Transform3d camToRobot;
    private final PhotonPoseEstimator photonPoseEstimator;
    private Pose3d m_cachedPose = new Pose3d();
    private double[] akitPose = {0, 0, 0};
    private boolean visionEnabled = true;
    private Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevFunction;
    private int targetsUsed = 0;
    private PhotonTrackedTarget tag;

    public PhotonVision(String camName, Transform3d camToRobot, Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevFunction) {
        camera = new PhotonCamera(camName);
        this.camToRobot = camToRobot;
        this.stdDevFunction = stdDevFunction;
        photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, camToRobot);
        SB_PV_TAB.addDoubleArray("akitPose, " + camera.getName(), () -> akitPose);
        SB_PV_TAB.addInteger("Targets Used," + camera.getName(), () -> targetsUsed);
    }

    public Command blink() {
        return startEnd(() -> camera.setLED(VisionLEDMode.kBlink), () -> camera.setLED(VisionLEDMode.kOff));
    }
    public PhotonVision(String camName, Transform3d camToRobot) {
        this(camName, camToRobot, ep -> {
            if (ep.targetsUsed.size() > 1)
                return VecBuilder.fill(1.5, 1.5, 1000000000);
            else {
                double poseAmbiguity = ep.targetsUsed.stream().mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).average().orElse(-1);
                if (poseAmbiguity > .3 || poseAmbiguity < 0)
                    return VecBuilder.fill(100000000, 100000000, 100000000);
                return VecBuilder.fill(.9, .9, 1000000000);
            }
        });
    }


    public Function<EstimatedRobotPose, Matrix<N3, N1>> getStdDevFunction() {
        return stdDevFunction;
    }

    public void setStdDevFunction(Function<EstimatedRobotPose, Matrix<N3, N1>> stdDevFunction) {
        this.stdDevFunction = stdDevFunction;
    }

    @Override
    public void periodic() {
        // TODO: make this a daemon
        // This method will be called once per scheduler run
        if (dashboardVisionEnabled) {

            photonPoseEstimator.update().ifPresent(ep -> {
                m_cachedPose = ep.estimatedPose;
                Pose2d pose = m_cachedPose.toPose2d();
                akitPose[0] = pose.getX();
                akitPose[1] = pose.getY();
                akitPose[2] = pose.getRotation().getRadians();
                if (visionEnabled) {
                    // TunerConstants.DriveTrain.setVisionMeasurementStdDevs(new Matrix<>(Nat.N3(),Nat.N1(),new double[]{1,2,100000000}));
                    this.targetsUsed = ep.targetsUsed.size();
                    TunerConstants.DriveTrain.addVisionMeasurement(pose, ep.timestampSeconds, stdDevFunction.apply(ep)); // TODO: do stuff with the stddevs

                    if (DriverStation.isDisabled()) {
                        TunerConstants.DriveTrain.getPigeon2().setYaw(pose.getRotation().getDegrees());
                        TunerConstants.DriveTrain.seedFieldRelative(pose);
                    }

                }
            });
        }
        tag = camera.getLatestResult().targets.stream().filter(t -> t.getFiducialId() == 4 || t.getFiducialId() == 7).findFirst().orElse(null);


    }

    public Pose3d getPose() {
        return m_cachedPose;
    }

    public Optional<Transform3d> robotToSpeaker() {
        // PhotonTrackedTarget tag = camera.getLatestResult().targets.stream().filter(t -> t.getFiducialId() == 4 || t.getFiducialId() == 7).findFirst().orElse(null);
        if (tag == null) {
            return Optional.empty();
        } else
            return Optional.of(tag.getBestCameraToTarget().plus(camToRobot));
    }

    public boolean seeTheSpeaker() {
        return tag != null;
 
    }
    public void setEnabled(boolean enabled) {
        this.visionEnabled = enabled;
    }
}