// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.generated.TunerConstants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  private PhotonCamera camera;

  private Pose2d prevEstimatedRobotPose = new Pose2d();
  private AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();
  private Transform3d robotToCam;
  private PhotonPoseEstimator photonPoseEstimator;
  // private final Field2d m_field = new Field2d();
  public double[] akitPose = {0,0,0};
  private boolean visionEnabled = true;

  public PhotonVision(String camName, Transform3d robotToCam) {
     camera = new PhotonCamera(camName);
     this.robotToCam = robotToCam;
     photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camera, robotToCam);

    
// StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
//     .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();

          // SmartDashboard.putData("Field", m_field);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
      photonPoseEstimator.update().ifPresent(ep -> {
          Pose2d pose = ep.estimatedPose.toPose2d();
          akitPose[0] = pose.getX();
          akitPose[1] = pose.getY();
          akitPose[2] = pose.getRotation().getRadians();
          SmartDashboard.putNumberArray("akitPose, " + camera.getName(), akitPose);
          if (visionEnabled) {
            // TunerConstants.DriveTrain.setVisionMeasurementStdDevs(new Matrix<>(Nat.N3(),Nat.N1(),new double[]{1,2,100000000}));
            SmartDashboard.putNumber("stuff", ep.targetsUsed.size());
            TunerConstants.DriveTrain.addVisionMeasurement(pose, ep.timestampSeconds, VecBuilder.fill(1.5,1.5,1000000000)); // TODO: do stuff with the stddevs

            if (DriverStation.isDisabled()) {
              TunerConstants.DriveTrain.getPigeon2().setYaw(pose.getRotation().getDegrees());
              TunerConstants.DriveTrain.seedFieldRelative(pose);
            }

          }
      });

}

public void setEnanbled(boolean enabled) {
  this.visionEnabled = enabled;
}
}