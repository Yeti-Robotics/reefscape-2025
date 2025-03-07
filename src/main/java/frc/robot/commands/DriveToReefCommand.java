package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.subsystems.vision.apriltag.AprilTagSubsystem;


public class DriveToReefCommand extends Command {
    private final CommandSwerveDrivetrain drivetrain;
    private final AprilTagSubsystem aprilTagSubsystem1;
    private final AprilTagSubsystem aprilTagSubsystem2;

    public DriveToReefCommand(CommandSwerveDrivetrain drivetrain, AprilTagSubsystem aprilTagSubsystem1, AprilTagSubsystem aprilTagSubsystem2) {
        this.drivetrain = drivetrain;
        this.aprilTagSubsystem1 = aprilTagSubsystem1;
        this.aprilTagSubsystem2 = aprilTagSubsystem2;
        addRequirements(this.drivetrain);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

    }
}
