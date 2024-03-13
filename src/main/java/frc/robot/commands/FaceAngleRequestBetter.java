package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class FaceAngleRequestBetter extends SwerveRequest.FieldCentricFacingAngle {
    public FaceAngleRequestBetter() { // TODO TUNE THIS
        HeadingController.setP(10);
        HeadingController.setI(0);
        HeadingController.setD(0);

    }

    public boolean atTarget() {
        return Math.abs(TargetDirection.minus(TunerConstants.DriveTrain.getState().Pose.getRotation()).getDegrees()) < 3; // 3 degree leeway
    }

    public Trigger inPositionTrigger() {
        return new Trigger(this::atTarget);
    }
}
