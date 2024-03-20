package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

public class FaceAngleRequestBetter extends SwerveRequest.FieldCentricFacingAngle {
    public FaceAngleRequestBetter() { // TODO TUNE THIS
        super();
        HeadingController.setP(10);
        HeadingController.setI(0);
        HeadingController.setD(0.5);
        Deadband = Math.toRadians(3);
    }

    public boolean atTarget() {
        return Math.abs(TargetDirection.minus(TunerConstants.DriveTrain.getState().Pose.getRotation()).getDegrees()) < Deadband; 
    }

    public Trigger inPositionTrigger() {
        return new Trigger(this::atTarget);
    }
}
