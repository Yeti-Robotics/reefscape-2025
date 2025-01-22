package frc.robot.commands;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class TurnToPoint extends SwerveRequest.FieldCentricFacingAngle {
    public Translation2d targetPoint;

    @Override
    public StatusCode apply(
            SwerveDrivetrain.SwerveControlParameters parameters, SwerveModule...modulesToApply) {
        this.TargetDirection = targetPoint.minus(parameters.currentPose.getTranslation()).getAngle();
        if (ForwardPerspective == ForwardPerspectiveValue.OperatorPerspective) {
            this.TargetDirection = this.TargetDirection.minus(parameters.operatorForwardDirection);
        }
        return super.apply(parameters, modulesToApply);
    }

    @Override
    public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
        return this;
    }

    public void setTargetPoint(Translation2d point){
        targetPoint = point;
    }
}
