package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final SwerveRequest.FieldCentricFacingAngle poseAimReq;
    private final DoubleSupplier xVelSupplier;
    private final DoubleSupplier yVelSupplier;

    private final AprilTagSubsystem reefCam;
    private Pose2d currPose;
    private AprilTagDetection detection;
    private Pose2d tagPose;
    private Transform2d targetTransform = FieldConstants.RIGHT_BRANCH_TRANSFORM;
    private boolean isLeft;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam,
            DoubleSupplier yVelocitySupplier,
            DoubleSupplier xVelocitySupplier,
            BooleanSupplier isLeft) {

        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam = reefCam;
        this.xVelSupplier = xVelocitySupplier;
        this.yVelSupplier = yVelocitySupplier;
        this.isLeft = isLeft.getAsBoolean();

        addRequirements(this.commandSwerveDrivetrain);
        poseAimReq = new SwerveRequest.FieldCentricFacingAngle();
        poseAimReq.HeadingController.setPID(5, 0, 0);
        poseAimReq.HeadingController.setTolerance(0.07);
        poseAimReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        System.out.println("Reef cmd init");
        this.currPose = commandSwerveDrivetrain.getState().Pose;
        Optional<AprilTagDetection> aprilTagDetectionOpt = reefCam.getBestDetection();
        SmartDashboard.putBoolean("ATag present", aprilTagDetectionOpt.isPresent());
        if (aprilTagDetectionOpt.isEmpty()) {
            cancel();
            return;
        }

        if (isLeft){
            targetTransform = FieldConstants.LEFT_BRANCH_TRANSFORM;
        }
        else{
            targetTransform = FieldConstants.RIGHT_BRANCH_TRANSFORM;
        }

        detection = aprilTagDetectionOpt.get();
        tagPose = detection.getTargetPose();
    }

    Field2d field = new Field2d();

    @Override
    public void execute() {
        System.out.println("tag id: " + detection.getFiducialID());
        field.setRobotPose(
                tagPose.transformBy(targetTransform));
        SmartDashboard.putData("ADetection Pose", field);

        // Apply drive control with joystick inputs
        commandSwerveDrivetrain.setControl(
                poseAimReq
                        .withTargetDirection(
                                tagPose.transformBy(targetTransform)
                                        .getTranslation()
                                        .minus(currPose.getTranslation())
                                        .getAngle()
                                        .minus(Rotation2d.fromDegrees(270)))
                        .withVelocityX(
                                -xVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude())
                        .withVelocityY(
                                -yVelSupplier.getAsDouble()
                                        * TunerConstants.kSpeedAt12Volts.magnitude()));
        SmartDashboard.putNumber(
                "ADetection Error", poseAimReq.HeadingController.getPositionError());
    }

    public enum Branches{
        LEFT(true),
        RIGHT(false);

        private final boolean isLeft;

        Branches(final boolean isLeft) {
            this.isLeft = isLeft;
        }
        public boolean getBoolean() {
            return isLeft;
        }

    }
}
