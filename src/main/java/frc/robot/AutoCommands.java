package frc.robot;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.coral.CoralManipulatorSystem;
import frc.robot.subsystems.drivetrain.CommandSwerveDrivetrain;
import frc.robot.util.PathPlannerUtils;
import java.util.Optional;

public class AutoCommands {
    private final CoralManipulatorSystem coralManipulator;
    private final CommandSwerveDrivetrain drivetrain;

    public AutoCommands(
            CoralManipulatorSystem coralManipulator, CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.coralManipulator = coralManipulator;
    }

    public Command driveForward() {
        return drivetrain.applyRequest(
                () -> new SwerveRequest.ApplyRobotSpeeds().withSpeeds(new ChassisSpeeds(1, 0, 0)));
    }

    public Command lolli() {
        Optional<PathPlannerPath> AtoLolli2 = PathPlannerUtils.loadPathByName("AtoLolli2");

        return AtoLolli2.isEmpty() ? Commands.none() : AutoBuilder.followPath(AtoLolli2.get());
    }

    public SendableChooser<Command> buildAutoCommandChooser() {
        SendableChooser<Command> auto = new SendableChooser<>();

        auto.addOption("Drive forward", driveForward());

        auto.setDefaultOption("Drive forward", driveForward());
        return auto;
    }
}
