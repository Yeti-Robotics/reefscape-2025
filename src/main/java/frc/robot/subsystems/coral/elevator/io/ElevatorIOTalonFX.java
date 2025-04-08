package frc.robot.subsystems.coral.elevator.io;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.constants.Constants;
import frc.robot.util.akit.device.can.talon.TalonFXDevice;

public class ElevatorIOTalonFX implements ElevatorIO {
    private final TalonFX primaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.primaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/PrimaryMotor")
                    .using(ElevatorConfig.primaryTalonFXConfigs)
                    .syncConfigs()
                    .getDevice();

    private final TalonFX secondaryElevatorMotor =
            TalonFXDevice.configure(ElevatorConfig.secondaryElevatorMotorID, Constants.CANIVORE_BUS)
                    .log("ElevatorIO/SecondaryMotor")
                    .using(ElevatorConfig.secondaryTalonFXConfigs)
                    .syncConfigs()
                    .oppose(primaryElevatorMotor)
                    .getDevice();

    private final MotionMagicTorqueCurrentFOC motionMagicReq = new MotionMagicTorqueCurrentFOC(0);
    private final StatusSignal<Angle> elevatorPosition = primaryElevatorMotor.getPosition();

    private final DigitalInput magSwitch = new DigitalInput(ElevatorConfig.magSwitchID);

    @Override
    public boolean isAtSetpoint(Angle setpoint) {
        return isAtSetpoint(setpoint, ElevatorConfig.HEIGHT_TOLERANCE);
    }

    @Override
    public boolean isAtSetpoint(Angle setpoint, Angle tolerance) {
        return getPosition().isNear(setpoint, tolerance);
    }

    @Override
    public Angle getPosition() {
        return elevatorPosition.getValue();
    }

    @Override
    public void setPosition(Angle value) {
        primaryElevatorMotor.setControl(motionMagicReq.withPosition(value));
    }

    @Override
    public void setControl(ControlRequest controlData) {
        primaryElevatorMotor.setControl(controlData);
    }

    @Override
    public boolean bottomSwitchTriggered() {
        return magSwitch.get();
    }
}
