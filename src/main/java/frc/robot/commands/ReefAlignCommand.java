package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final Transform2d leftBranchTransform =
            new Transform2d(
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(8.04)),
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(6.47)),
                    Rotation2d.kZero);
    private final Transform2d rightBranchTransform =
            new Transform2d(
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(8.04)),
                    edu.wpi.first.units.Units.Meters.of(Units.inchesToMeters(-6.47)),
                    Rotation2d.kZero);
    private final SwerveRequest.RobotCentricFacingAngle swerveReq =
            new SwerveRequest.RobotCentricFacingAngle()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1);
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    AprilTagDetection lockedOnAprilTag;
    boolean isLeftBranch = false;
    boolean isFinished = false;
    PIDController movementXPIDController = new PIDController(3, 0, 0);
    PIDController movementYPIDController = new PIDController(3, 0, 0);
    Pose2d initialBranchPos;
    DoubleSupplier joyX;
    DoubleSupplier joyY;
    DoubleSupplier rotation;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2,
            DoubleSupplier joyX,
            DoubleSupplier joyY,
            DoubleSupplier rotation) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;
        this.joyX = joyX;
        this.joyY = joyY;
        this.rotation = rotation;

        swerveReq.HeadingController.setPID(6, 0, 0);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        movementXPIDController.setTolerance(0.07);
        movementYPIDController.setTolerance(0.07);
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        Optional<AprilTagDetection> reefCam1Detection = reefCam1.getBestDetection();
        Optional<AprilTagDetection> reefCam2Detection = reefCam2.getBestDetection();

        if (reefCam1Detection.isPresent() && reefCam2Detection.isPresent()) {
            double reefCam1Distance =
                    reefCam1Detection.get().getRobotToTargetPose().getTranslation().getNorm();
            double reefCam2Distance =
                    reefCam2Detection.get().getRobotToTargetPose().getTranslation().getNorm();

            return reefCam1Distance > reefCam2Distance ? reefCam2Detection : reefCam1Detection;
        }
        return reefCam1Detection.or(() -> reefCam2Detection);
    }

    @Override
    public void initialize() {
        lockedOnAprilTag = null;

        Optional<AprilTagDetection> reefCamDetection = getReefCamDetection();

        if (reefCamDetection.isEmpty()) {
            isFinished = true;
            return;
        }

        lockedOnAprilTag = reefCamDetection.get();
    }

    Field2d field = new Field2d();
    Field2d field2 = new Field2d();

    @Override
    public void execute() {
        Optional<AprilTagDetection> reefCamDetectionOpt = getReefCamDetection();
        if (reefCamDetectionOpt.isEmpty()) {
            return;
        }

        AprilTagDetection reefCamDetection = reefCamDetectionOpt.get();
        if (lockedOnAprilTag == null // probably will never happen, but who knows?
                || reefCamDetection.getFiducialID() != lockedOnAprilTag.getFiducialID()) {
            isFinished = true;
            return;
        }

        Pose2d targetVisionPose = reefCamDetection.getRobotToTargetPose();
        Pose2d targetBranchPose =
                targetVisionPose
                        .transformBy(isLeftBranch ? leftBranchTransform : rightBranchTransform)
                        .transformBy(new Transform2d(0.05, 0, Rotation2d.kZero));
        field.setRobotPose(
                reefCamDetection
                        .getRobotInFieldPose()
                        .transformBy(
                                new Transform2d(
                                        targetBranchPose.getTranslation(),
                                        targetBranchPose.getRotation())));
        field2.setRobotPose(targetVisionPose);
        Pose2d drivetrainPose = commandSwerveDrivetrain.getState().Pose;
        SmartDashboard.putData("ATarget Branch Pose", field);
        SmartDashboard.putData("ATarget Vision Pose", field2);
        SmartDashboard.putNumber("ErrorX", movementXPIDController.getError());
        SmartDashboard.putNumber("ErrorY", movementYPIDController.getError());

        double degreeAprilTag = lockedOnAprilTag.getRobotToTargetPose().getRotation().getDegrees();
        boolean isRightFacingReef = Math.abs(degreeAprilTag - 90) > Math.abs(degreeAprilTag + 90);

        // right cam, 90 | left cam, -90
        Rotation2d visionTargetAngularDistance =
                Rotation2d.fromDegrees(isRightFacingReef ? 90 : -90)
                        .minus(targetVisionPose.getRotation());

        Rotation2d driveTargetDirection =
                drivetrainPose.getRotation().minus(visionTargetAngularDistance);

        double veloX = movementXPIDController.calculate(0, targetBranchPose.getX());
        double veloY = movementYPIDController.calculate(0, targetBranchPose.getY());

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(veloX)
                        .withVelocityY(veloY)
                        .withTargetDirection(driveTargetDirection));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        lockedOnAprilTag = null;
        isFinished = false;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
