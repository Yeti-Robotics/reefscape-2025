package frc.robot.subsystems.coral.elevator;

import static frc.robot.subsystems.coral.elevator.ElevatorConfig.*;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.util.sim.PhysicsSim;
import frc.robot.util.sim.SimulatableMechanism;
import frc.robot.util.state.StateUtils;
import frc.robot.util.state.StatefulSetpointSubsystem;

@Logged
public class ElevatorSubsystem
        extends StatefulSetpointSubsystem<ElevatorPosition, AngleUnit, Angle, MutAngle>
        implements SimulatableMechanism {
    private final TalonFX primaryElevatorMotor =
            new TalonFX(ElevatorConfig.primaryElevatorMotorID, Constants.SYSCORE_0_BUS);
    private final TalonFX secondaryElevatorMotor =
            new TalonFX(ElevatorConfig.secondaryElevatorMotorID, Constants.SYSCORE_0_BUS);
    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);
    private final NeutralOut neutralOut = new NeutralOut();
    private final MotionMagicTorqueCurrentFOC magicRequest =
            new MotionMagicTorqueCurrentFOC(0).withSlot(Robot.isReal() ? 0 : 1);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();
    private final StatusSignal<Double> elevatorTargetPosition =
            primaryElevatorMotor.getClosedLoopReference();

    public ElevatorSubsystem() {
        super(
                ElevatorPosition.HOLD,
                StateUtils.mutableRotationSetpoint(),
                Units.Rotations.of(ElevatorConfig.HEIGHT_TOLERANCE));
        primaryElevatorMotor.getConfigurator().apply(primaryTalonFXConfigs);
        secondaryElevatorMotor.getConfigurator().apply(secondaryTalonFXConfigs);
        secondaryElevatorMotor.setControl(
                new Follower(ElevatorConfig.primaryElevatorMotorID, true));

        new Trigger(this::getMagSwitch)
                .debounce(2)
                .onTrue(zeroPosition().andThen(transitionTo(ElevatorPosition.BOTTOM)));

        primaryElevatorMotor.setPosition(0);
        secondaryElevatorMotor.setPosition(0);

        primaryElevatorMotor.setControl(neutralOut);

        if (Robot.isSimulation()) {
            PhysicsSim.getInstance().addTalonFX(primaryElevatorMotor);
            PhysicsSim.getInstance().addTalonFX(secondaryElevatorMotor);
        }
    }

    @Override
    public void runPeriodic() {
        super.runPeriodic();
    }

    private Command zeroPosition() {
        return runOnce(() -> primaryElevatorMotor.setPosition(0));
    }

    public boolean getMagSwitch() {
        return !magSwitch.get();
    }

    @Override
    public StatusSignal<Angle> currentStateSignal() {
        return elevatorPosition;
    }

    @Override
    public Angle determineSetpoint(ElevatorPosition targetState) {
        return targetState == ElevatorPosition.HOLD
                ? elevatorPosition.getValue()
                : targetState.getHeight();
    }

    @Override
    public StatusCode moveTo(Angle setpoint) {
        return primaryElevatorMotor.setControl(magicRequest.withPosition(setpoint));
    }

    @Override
    public Angle getCurrentPosition() {
        return elevatorPosition.getValue();
    }

    @Override
    public Angle getTargetPosition() {
        return Units.Rotations.of(elevatorTargetPosition.getValue());
    }

    @Override
    protected boolean isTransitionFinished() {
        boolean transitionFinished = super.isTransitionFinished();
        if (transitionFinished
                && transitioningTo().isPresent()
                && transitioningTo().get().equals(ElevatorPosition.BOTTOM)) {
            primaryElevatorMotor.setControl(neutralOut);
        }
        return transitionFinished;
    }
}
