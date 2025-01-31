package frc.robot.subsystems.coral;

import static frc.robot.constants.Constants.*;

import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.reduxrobotics.sensors.canandcolor.ColorData;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.StateManager;

public class CoralIntake extends SubsystemBase {
    private final TalonFX claw = new TalonFX(CoralConfigs.CLAW_ID, RIO_BUS);
    public final Canandcolor traySensor = new Canandcolor(0);
    public final Canandcolor clawSensor = new Canandcolor(1);
    private final MotionMagicVelocityVoltage motorRequest = new MotionMagicVelocityVoltage(0);

    public final Trigger intakeOccupiedTrigger;
    public final Trigger stopTrigger;
    public final Trigger trayOccupiedTrigger;


    public enum CoralState {
        ROLL_OUT,
        ROLL_IN,
        CORAL_HOLD,
        OFF
    }

    public StateManager<CoralState> coralIntakeState = new StateManager<>(CoralState.OFF);

    public CoralIntake() {
        claw.getConfigurator().apply(CoralConfigs.coralMotorConfig);

        intakeOccupiedTrigger = new Trigger(this::isCoralInIntake);
        stopTrigger = new Trigger(this::readyToStop);
        trayOccupiedTrigger = new Trigger(this::isCoralInTray);

        intakeOccupiedTrigger.onTrue(this.holdClaw());
        stopTrigger.onTrue(this.stopClaw());
        trayOccupiedTrigger.onTrue(this.spinClawForward());
    }

    @Override
    public void periodic() {
        switch (coralIntakeState.getState()) {
            case ROLL_IN:
                setClawVelocity(CoralConfigs.FORWARD_SPEED);
                break;
            case ROLL_OUT:
                setClawVelocity(CoralConfigs.BACKWARD_SPEED);
                break;
            case OFF:
            case CORAL_HOLD:
                stopClawSpin();
                break;
        }
    }


    private boolean readyToStop() {
        return isCoralInIntake() && coralIntakeState.getState() == CoralState.ROLL_IN
                || coralIntakeState.getState() == CoralState.ROLL_OUT && !isCoralInIntake();
    }

    public boolean isCoralInTray() {
        ColorData traySensorColor = traySensor.getColor();

        return traySensorColor.red() == CoralConfigs.coralColor.red()
                && traySensorColor.green() == CoralConfigs.coralColor.green()
                && traySensorColor.blue() == CoralConfigs.coralColor.blue();
    }

    public boolean isCoralInIntake() {
        ColorData clawSensorColor = clawSensor.getColor();

        return clawSensorColor.red() == CoralConfigs.coralColor.red()
                && clawSensorColor.green() == CoralConfigs.coralColor.green()
                && clawSensorColor.blue() == CoralConfigs.coralColor.blue();
    }

    private void setClawVelocity(double velocity) {
        claw.setControl(motorRequest.withVelocity(velocity));
    }

    private void stopClawSpin() {
        claw.stopMotor();
    }

    public Command spinClawForward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_IN));
    }

    public Command spinClawBackward() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.ROLL_OUT));
    }

    public Command holdClaw() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.CORAL_HOLD));
    }

    public Command stopClaw() {
        return runOnce(() -> coralIntakeState.transitionTo(CoralState.OFF));
    }
}
