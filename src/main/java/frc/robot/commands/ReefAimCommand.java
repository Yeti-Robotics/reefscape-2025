package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagConstants;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.LimelightHelpers;
import java.util.Optional;
import java.util.function.DoubleSupplier;

public class ReefAimCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private int currentTag;

    public ReefAimCommand(
            CommandSwerveDrivetrain drivetrain,
            AprilTagSubsystem aprilTagSubsystem,
            DoubleSupplier xVelSupplier,
            DoubleSupplier yVelSupplier) {
        this.commandSwerveDrivetrain = drivetrain;
        this.aprilTagSubsystem = aprilTagSubsystem;
        this.xVelSupplier = xVelSupplier;
        this.yVelSupplier = yVelSupplier;

        addRequirements(drivetrain);

        poseAimRequest = new TurnToPoint();
        poseAimRequest.HeadingController.setPID(5, 0, 0);
        poseAimRequest.HeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void initialize() {
        Optional<AprilTagDetection> possibleDetection = aprilTagSubsystem.getBestDetection();

        if (possibleDetection.isPresent()) {
            currentTag = possibleDetection.get().getFiducialID();
            Translation2d targetPosition = possibleDetection.get().getTargetTranslation();
            poseAimRequest.setTargetPoint(targetPosition);
        } else {
            poseAimRequest.setTargetPoint(FieldConstants.Reef);
        }
    }

    @Override
    public void execute() {
        if (LimelightHelpers.getFiducialID("yetilime") == currentTag) {
            commandSwerveDrivetrain.setControl(
                    poseAimRequest
                            .withVelocityX(xVelSupplier.getAsDouble() * 1.5)
                            .withVelocityY(yVelSupplier.getAsDouble() * 1.5));
        } else {
            end(true);
        }
        SmartDashboard.putNumber("Limelight tag", LimelightHelpers.getFiducialID(AprilTagConstants.LIMELIGHT_NAME));
        SmartDashboard.putNumber("currentag", currentTag);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {}
}
