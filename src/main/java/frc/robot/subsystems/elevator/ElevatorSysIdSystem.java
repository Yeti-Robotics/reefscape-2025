package frc.robot.subsystems.elevator;


import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.*;

public class ElevatorSysIdSystem extends SubsystemBase {
    private final TalonFX primaryElevatorMotor;
    StatusSignal<Angle> rotations;

    private final SysIdRoutine routine;
    public ElevatorSysIdSystem() {

        primaryElevatorMotor = new TalonFX(ElevatorConfigs.primaryElevatorMotorPort, "canivoreBUS");
        rotations = primaryElevatorMotor.getRotorPosition();

        StatusSignal<Voltage> motorVoltage = primaryElevatorMotor.getMotorVoltage();
        StatusSignal<AngularVelocity> motorRotationsVelo = primaryElevatorMotor.getRotorVelocity();
        StatusSignal<AngularAcceleration> accelerationStatusSignal = primaryElevatorMotor.getAcceleration();
        StatusSignal<Current> supplyCurrent = primaryElevatorMotor.getSupplyCurrent();
        StatusSignal<Current> statorCurrent = primaryElevatorMotor.getStatorCurrent();


        SysIdRoutine.Config config = new SysIdRoutine.Config();
        SysIdRoutine.Mechanism mechanism = new SysIdRoutine.Mechanism(
                this::applyOutput, log -> {
            motorVoltage.refresh();
            rotations.refresh();
            motorRotationsVelo.refresh();
            accelerationStatusSignal.refresh();
            supplyCurrent.refresh();
            statorCurrent.refresh();
            log.motor("elevator_motor")
                    .voltage(motorVoltage.getValue())
                    .angularPosition(rotations.getValue())
                    .angularVelocity(motorRotationsVelo.getValue())
                    .angularAcceleration(
                            accelerationStatusSignal.getValue())
                    .current(supplyCurrent.getValue())
                    .value(
                            "Stator current",
                            statorCurrent.getValueAsDouble(),
                            "Amps");
        }, this);
        routine = new SysIdRoutine(config, mechanism);
    }


    public void applyOutput(Voltage output) {
        TorqueCurrentFOC req =
                new TorqueCurrentFOC(output.in(Volts));
        primaryElevatorMotor.setControl(req);
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return routine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
    }

    public double getPosition() {
        return rotations.getValueAsDouble();
    }
}
