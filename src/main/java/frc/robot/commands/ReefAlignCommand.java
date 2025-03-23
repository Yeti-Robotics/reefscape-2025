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
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.coral.CoralManipulatorState;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.AllianceFlipUtil;

import java.util.Map;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final CoralManipulatorSystem coralManipulatorSystem;

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

    private Pose2d reefTargetPose;

    public ReefAlignCommand(
            CommandSwerveDrivetrain commandSwerveDrivetrain,
            CoralManipulatorSystem coralManipulatorSystem,
            AprilTagSubsystem reefCam1,
            AprilTagSubsystem reefCam2) {
        this.commandSwerveDrivetrain = commandSwerveDrivetrain;
        this.coralManipulatorSystem = coralManipulatorSystem;
        this.reefCam1 = reefCam1;
        this.reefCam2 = reefCam2;


        swerveReq.HeadingController.setPID(6, 0, 0);
        swerveReq.HeadingController.setTolerance(0.07);
        swerveReq.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        movementXPIDController.setTolerance(0.07);
        movementYPIDController.setTolerance(0.07);
    }

    public Optional<AprilTagDetection> getReefCamDetection() {
        return reefCam1.getBestDetection().or(reefCam2::getBestDetection);
    }

    public Pose2d getBranchPoseFromTagID(int id) {
        boolean isRedAlliance = AllianceFlipUtil.shouldFlip();

        int branchPoseIndex = id - (isRedAlliance ? 18 : 7);

        if (branchPoseIndex < 0) {
            branchPoseIndex = isRedAlliance ? 6 : 17;
        }

        Map<FieldConstants.ReefLevel, Pose2d> poseMap = FieldConstants.Reef.branchPositions2d.get(branchPoseIndex);

        CoralManipulatorState queuedState = coralManipulatorSystem.getQueuedState();

        FieldConstants.ReefLevel reefLevelPose = switch (queuedState) {
            case L1 -> FieldConstants.ReefLevel.L1;
            case L2 -> FieldConstants.ReefLevel.L2;
            case L3 -> FieldConstants.ReefLevel.L3;
            default -> FieldConstants.ReefLevel.L4;
        };

        return poseMap.get(reefLevelPose);
    }

    @Override
    public void initialize() {
        Optional<AprilTagDetection> detectionOpt = getReefCamDetection();

        if (detectionOpt.isEmpty()) {
            isFinished = true;
            return;
        }

        int fiducialId = detectionOpt.get().getFiducialID();

        reefTargetPose = getBranchPoseFromTagID(fiducialId);
    }

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
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
