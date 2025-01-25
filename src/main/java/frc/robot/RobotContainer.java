// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.constants.Constants;
import frc.robot.subsystems.arm.ArmSubsystem;
import java.util.function.BooleanSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

    CommandXboxController xboxController;
    ArmSubsystem arm;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        xboxController = new CommandXboxController(Constants.XBOX_CONTROLLER_PORT);
        arm = new ArmSubsystem();
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
    private void configureBindings() {

        BooleanSupplier rangeLimit = () -> false;

        xboxController
                .a()
                .and(xboxController.leftBumper())
                .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kForward).until(rangeLimit));
        xboxController
                .b()
                .and(xboxController.leftBumper())
                .whileTrue(arm.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).until(rangeLimit));
        xboxController
                .x()
                .and(xboxController.leftBumper())
                .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kForward).until(rangeLimit));
        xboxController
                .y()
                .and(xboxController.leftBumper())
                .whileTrue(arm.sysIdDynamic(SysIdRoutine.Direction.kReverse).until(rangeLimit));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return null;
    }
}
