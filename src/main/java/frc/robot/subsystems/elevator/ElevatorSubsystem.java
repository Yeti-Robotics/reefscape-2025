package frc.robot.subsystems.elevator;

import static frc.robot.constants.Constants.RIO_BUS;
import static frc.robot.subsystems.elevator.ElevatorConfigs.primaryTalonFXConfigs;
import static frc.robot.subsystems.elevator.ElevatorConfigs.secondaryTalonFXConfigs;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;

@Logged
public class ElevatorSubsystem extends SubsystemBase implements SimulatableMechanism {
    private final TalonFX primaryElevatorMotor;
    private final TalonFX secondaryElevatorMotor;
    private final DigitalInput magSwitch;
    public final MotionMagicVoltage magicRequest;

    public ElevatorSubsystem() {
        primaryElevatorMotor = new TalonFX(ElevatorConfigs.primaryElevatorMotorID, RIO_BUS);
        secondaryElevatorMotor = new TalonFX(ElevatorConfigs.secondaryElevatorMotorID, RIO_BUS);

        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfigs.primaryElevatorMotorID, true));

        magSwitch = new DigitalInput(ElevatorConfigs.magSwitchID);
        magicRequest = new MotionMagicVoltage(0);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
        }
    }

    public void setPosition(ElevatorPosition position) {
        primaryElevatorMotor.setControl(
                magicRequest.withPosition(position.getHeight()).withSlot(Robot.isReal() ? 0 : 1));
    }

    public void setPosition(double setpoint) {
        primaryElevatorMotor.setControl(
                magicRequest.withPosition(setpoint).withSlot(Robot.isReal() ? 0 : 1));
    }

    public Command setPositionCommand(double setpoint) {
        return runOnce(() -> setPosition(setpoint));
    }

    public void stop() {
        primaryElevatorMotor.stopMotor();
    }

    public boolean getMagSwitch() {
        return magSwitch.get();
    }

    public Command raiseLift() {
        return startEnd(() -> primaryElevatorMotor.set(0.5), this::stop);
    }

    public Command lowerLift() {
        return startEnd(() -> primaryElevatorMotor.set(-0.5), this::stop);
    }

    @Override
    public double updateMechPos() {
        return Units.inchesToMeters(primaryElevatorMotor.getPosition().getValueAsDouble());
    }
}
