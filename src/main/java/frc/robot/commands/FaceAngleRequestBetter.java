package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.generated.TunerConstants;

import static frc.robot.Constants.SB_TEST;

public class FaceAngleRequestBetter extends SwerveRequest.FieldCentricFacingAngle {
    public FaceAngleRequestBetter() { // TODO TUNE THIS
        super();
        SB_TEST.add("FaceAngleRequest", HeadingController);
        HeadingController.setP(6);
        HeadingController.setI(0);
        HeadingController.setD(0);
        HeadingController.enableContinuousInput(-Math.PI, Math.PI);
        RotationalDeadband = Math.toRadians(.5);

    }

    public boolean atTarget() {
        return HeadingController.atSetpoint();
    }

    @Override
    public FaceAngleRequestBetter withDeadband(double deadband) {
        this.Deadband = deadband;
        return this;
    }

    public Trigger inPositionTrigger() {
        return new Trigger(this::atTarget);
    }
}
