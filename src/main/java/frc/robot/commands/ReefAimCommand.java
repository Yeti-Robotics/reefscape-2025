package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.vision.apriltag.AprilTagDetection;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;

import java.util.Optional;
import java.util.function.DoubleSupplier;


public class ReefAimCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final AprilTagSubsystem aprilTagSubsystem;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private int currentTag;

    public ReefAimCommand(CommandSwerveDrivetrain drivetrain, AprilTagSubsystem aprilTagSubsystem1, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
            this.commandSwerveDrivetrain = drivetrain;
            this.aprilTagSubsystem = aprilTagSubsystem1;
            this.xVelSupplier = xVelSupplier;
            this.yVelSupplier = yVelSupplier;

            addRequirements(drivetrain);

            poseAimRequest = new TurnToPoint();
            poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    @Override
    public void initialize() {
        Optional<AprilTagDetection> possibleDetection = aprilTagSubsystem.getBestDetection();

        if (possibleDetection.isPresent()){
            currentTag = possibleDetection.get().getFiducialID();
            Translation2d targetPosition = possibleDetection.get().getTargetTranslation();
            poseAimRequest.setTargetPoint(targetPosition);
        }
        else {
            poseAimRequest.setTargetPoint(FieldConstants.Reef);
        }



    }

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
