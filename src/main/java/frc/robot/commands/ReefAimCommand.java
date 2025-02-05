package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.impl.photon.PhotonAprilTagSystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class ReefAimCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final PhotonAprilTagSystem photonSubsystem;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private int currentTag;

    public ReefAimCommand(CommandSwerveDrivetrain drivetrain, PhotonAprilTagSystem photonSubsystem, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {

            this.commandSwerveDrivetrain = drivetrain;
            this.photonSubsystem = photonSubsystem;
            this.xVelSupplier = xVelSupplier;
            this.yVelSupplier = yVelSupplier;

            addRequirements(drivetrain);

            poseAimRequest = new TurnToPoint();
            poseAimRequest.HeadingController.setPID(5,0,0);
            poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
    }
/*
    @Override
    public void initialize() {

        if (currentTag == ) {
            Translation2d targetPosition = photonSubsystem.findDetection(currentTag).get().getTargetTranslation();
            poseAimRequest.setTargetPoint(targetPosition);
        }
    }*/
    @Override
    public void execute() {
        if (LimelightHelpers.getFiducialID("limelight") == currentTag){
commandSwerveDrivetrain.setControl(poseAimRequest.withVelocityX(xVelSupplier.getAsDouble() * 1.5).withVelocityY(yVelSupplier.getAsDouble() * 1.5));
        } else {
            end(true);
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
