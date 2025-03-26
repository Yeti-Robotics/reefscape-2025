package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.FieldConstants.Reef;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.drivetrain.TunerConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.AllianceFlipUtil;
import java.util.Optional;

public class ReefAlignCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;

    private static final int MAX_RETRIES = 5;
    private final AprilTagSubsystem reefCam1;
    private final AprilTagSubsystem reefCam2;
    private final CoralManipulatorSystem coralManipulatorSystem;

    private final SwerveRequest.FieldCentricFacingAngle swerveReq =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(TunerConstants.MAX_VELOCITY_METERS_PER_SECOND * 0.1)
                    .withRotationalDeadband(TunerConstants.MaFxAngularRate * 0.1);
    private final SwerveRequest.Idle stopReq = new SwerveRequest.Idle();
    AprilTagDetection lockedOnAprilTag;

    boolean isLeftBranch = false;
    boolean isFinished = false;

    PIDController movementXPIDController = new PIDController(3, 0, 0);
    PIDController movementYPIDController = new PIDController(3, 0, 0);

    private Pose2d reefTargetPose;
    private int retries = 0;

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
        boolean isRedAllianceReef = AllianceFlipUtil.shouldFlip();
        int branchPoseIndex = id - (isRedAllianceReef ? 18 : 7);

        if (branchPoseIndex > 5) {
            isFinished = true;
            return new Pose2d();
        }

        if (branchPoseIndex < 0) {
            branchPoseIndex = Reef.centerFaces.length - 1;
        }

        Pose2d reefTargetPose = Reef.centerFaces[branchPoseIndex];

        return AllianceFlipUtil.apply(reefTargetPose);
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
        Pose2d drivetrainPose = commandSwerveDrivetrain.getState().Pose;

        double veloX =
                movementXPIDController.calculate(drivetrainPose.getX(), reefTargetPose.getX());

        double veloY =
                movementXPIDController.calculate(drivetrainPose.getY(), reefTargetPose.getY());

        commandSwerveDrivetrain.setControl(
                swerveReq
                        .withVelocityX(veloX)
                        .withVelocityY(veloY)
                        .withTargetDirection(reefTargetPose.getRotation()));
    }

    @Override
    public void end(boolean interrupted) {
        commandSwerveDrivetrain.setControl(stopReq);
        retries = 0;
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    public Command toggleBranchSelection() {
        return Commands.runOnce(() -> isLeftBranch = !isLeftBranch);
    }
}
