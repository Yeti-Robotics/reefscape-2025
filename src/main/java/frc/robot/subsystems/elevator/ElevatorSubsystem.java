package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.RIO_BUS;
import static frc.robot.subsystems.elevator.ElevatorConfigs.primaryTalonFXConfigs;
import static frc.robot.subsystems.elevator.ElevatorConfigs.secondaryTalonFXConfigs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class ElevatorSubsystem extends SubsystemBase {
    private final TalonFX primaryElevatorMotor;
    private final TalonFX secondaryElevatorMotor;
    private final DigitalInput magSwitch;
    public final MotionMagicTorqueCurrentFOC magicRequest;

    public ElevatorSubsystem() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigs.primaryElevatorMotorID, RIO_BUS);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigs.secondaryElevatorMotorID, RIO_BUS);

        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfigs.primaryElevatorMotorID, true));

        magSwitch = new DigitalInput(ElevatorConfigs.magSwitchID);
        magicRequest = new MotionMagicTorqueCurrentFOC(0);
    }

    public void setPosition(ElevatorPosition position) {
        primaryElevatorMotor.setControl(magicRequest.withPosition(position.getHeight()));
    }

    public Command tuning(double output) {
        return startEnd(() -> primaryElevatorMotor.setControl(magicRequest), this::stop);
    }

    public void stop() {
        primaryElevatorMotor.stopMotor();
    }

    public boolean getMagSwitch() {
        return magSwitch.get();
    }

    public boolean isAtTarget(double target) {
        return Math.abs(primaryElevatorMotor.getPosition().getValueAsDouble() - target) < 0.05;
    }

    public Command moveTo(ElevatorPosition position) {
        return runOnce(() ->
                        primaryElevatorMotor.setControl(
                                magicRequest.withPosition(position.getHeight())))
                .until(() -> isAtTarget(position.getHeight()));
    }

    @Override
    public void periodic() {
        // Update logic if needed
    }
}
