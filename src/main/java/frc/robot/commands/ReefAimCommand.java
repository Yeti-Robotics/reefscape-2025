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
    private final AprilTagSubsystem aprilTagSubsystem1;
    private final AprilTagSubsystem aprilTagSubsystem2;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private int currentTag;

    public ReefAimCommand(CommandSwerveDrivetrain drivetrain, AprilTagSubsystem aprilTagSubsystem1, AprilTagSubsystem aprilTagSubsystem2, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {
            this.commandSwerveDrivetrain = drivetrain;
            this.aprilTagSubsystem1 = aprilTagSubsystem1;
            this.aprilTagSubsystem2 = aprilTagSubsystem2;
            this.xVelSupplier = xVelSupplier;
            this.yVelSupplier = yVelSupplier;

            addRequirements(drivetrain);

            poseAimRequest = new TurnToPoint();
            poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    @Override
    public void initialize() {
        Optional<AprilTagDetection> possibleDetection1 = aprilTagSubsystem1.getBestDetection();
        Optional<AprilTagDetection> possibleDetection2 = aprilTagSubsystem2.getBestDetection();

        if (possibleDetection1.isPresent()){
            currentTag = possibleDetection1.get().getFiducialID();
            Translation2d targetPosition = possibleDetection1.get().getTargetTranslation();
            poseAimRequest.setTargetPoint(targetPosition);
        }
        if (possibleDetection2.isPresent()){
            currentTag = possibleDetection2.get().getFiducialID();
            Translation2d targetPosition = possibleDetection2.get().getTargetTranslation();
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
