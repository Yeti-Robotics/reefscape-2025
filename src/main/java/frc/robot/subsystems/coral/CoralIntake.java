package frc.robot.subsystems.coral;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.StateManager;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw = new TalonFX(CoralConfigs.CLAW_ID, RIO_BUS);
    private final MotionMagicVelocityVoltage motorRequest = new MotionMagicVelocityVoltage(0);

    public DigitalInput traySensor = new DigitalInput(0);
    public DigitalInput clawSensor = new DigitalInput(0);


    public enum CoralState {
        ROLL_OUT,
        ROLL_IN,
        OFF,
        IDLE
    }

    public StateManager<CoralState> coralIntakeState = new StateManager<>(CoralState.OFF);

    public CoralIntake() {
        claw.getConfigurator().apply(CoralConfigs.coralMotorConfig);
    }

    public boolean isInTray() {
        return traySensor.get();
    }

    public boolean isInClaw() {
        return clawSensor.get();
    }

    @Override
    public void periodic() {
        switch (coralIntakeState.getState()) {
            case ROLL_IN:
                setClawVelocity(CoralConfigs.FORWARD_SPEED);
            case ROLL_OUT:
                setClawVelocity(CoralConfigs.BACKWARD_SPEED);
            case OFF:
                stopClaw();
            default:
                coralIntakeState.transitionTo(CoralState.IDLE);
                coralIntakeState.finishTransition();
        }
    }

    private void setClawVelocity(double velocity) {
        claw.setControl(motorRequest.withVelocity(velocity));
    }

    private void stopClaw() {
        claw.stopMotor();
    }

    public Command spinClawForward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_IN));
    }

    public Command spinClawBackward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_OUT));
    }
}
