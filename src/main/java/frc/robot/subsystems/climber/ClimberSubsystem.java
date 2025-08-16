package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConfig.*;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;

@Logged
public class ClimberSubsystem extends SubsystemBase {
    private final TalonFX climber;
    final MotionMagicTorqueCurrentFOC magicRequest;

    public final double climbSpeed = 0.75;
    public final double unClimbSpeed = -0.7;

    public ClimberSubsystem() {
        climber = new TalonFX(climberId, Constants.SYSCORE_0_BUS);
        magicRequest = new MotionMagicTorqueCurrentFOC(0);

        climber.getConfigurator().apply(climberTalonFXConfigs);
    }

    private void setClimberSpeed(double speed) {
        climber.set(speed);
    }

    private void stop() {
        climber.stopMotor();
    }

    public Command spinClimber(double speed) {
        return startEnd(() -> setClimberSpeed(speed), this::stop);
    }

    public void target(ClimberPosition position) {
        climber.setControl(magicRequest.withPosition(position.getAngle()));
    }
}
