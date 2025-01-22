package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.util.LimelightHelpers;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.LimelightHelpers;

import java.util.function.DoubleSupplier;


public class ReefAimCommand extends Command {
    private final CommandSwerveDrivetrain commandSwerveDrivetrain;
    private final DoubleSupplier xVelSupplier;
    private final TurnToPoint poseAimRequest;
    private final DoubleSupplier yVelSupplier;
    private double currentTag;

    public ReefAimCommand(CommandSwerveDrivetrain drivetrain, DoubleSupplier xVelSupplier, DoubleSupplier yVelSupplier) {

            this.commandSwerveDrivetrain = drivetrain;
            this.xVelSupplier = xVelSupplier;
            this.yVelSupplier = yVelSupplier;

            addRequirements(drivetrain);

            poseAimRequest = new TurnToPoint();
            poseAimRequest.HeadingController.setPID(5,0,0);
            poseAimRequest.HeadingController.enableContinuousInput(-Math.PI,Math.PI);
    }

    @Override
    public void initialize() {

        Translation2d reef = AllianceFlipUtil.apply(FieldConstants.Reef);
        poseAimRequest.setTargetPoint(reef);
    }
    @Override
    public void execute() {

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
