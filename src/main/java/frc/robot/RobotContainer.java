// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.constants.Constants;
import frc.robot.subsystems.algae.AlgaeIntake;
import frc.robot.subsystems.elevator.ElevatorSubsystem;
import frc.robot.subsystems.led.LEDSubsystem;
import frc.robot.subsystems.led.ProgressBar;
import frc.robot.subsystems.tray.Tray;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    CommandXboxController xboxController;
    ElevatorSubsystem elevatorSubsystem;
    Tray tray;
    AlgaeIntake algaeIntake;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        elevatorSubsystem = new ElevatorSubsystem();
        tray = new Tray();
        algaeIntake = new AlgaeIntake();
//        if (tray.isCoralInTray()) {
//            Robot.progressBar.addProgress();
//        } else {
//            Robot.progressBar.subtractProgress();
//        }
//        if (elevatorSubsystem.getHeight() == 0) {
//            Robot.progressBar.addProgress();
//        } else {
//            Robot.progressBar.subtractProgress();
//        }
//        if (swerveDrivetrain.isZeroed()) {
//            Robot.progressBar.addProgress();
//        } else {
//            Robot.progressBar.subtractProgress();
//        }
//        if (climber.isStowed()) {
//            Robot.progressBar.addProgress();
//        } else {
//            Robot.progressBar.subtractProgress();
//        }
//        if (algaeIntake.isStowed()) {
//            Robot.progressBar.addProgress();
//        } else {
//            Robot.progressBar.subtractProgress();
//        }
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
