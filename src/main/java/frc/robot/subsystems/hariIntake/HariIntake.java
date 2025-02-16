package frc.robot.subsystems.hariIntake;

import static frc.robot.constants.Constants.*;
import static frc.robot.subsystems.hariIntake.HariIntakeConfigs.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class HariIntake extends SubsystemBase {
    private final TalonFX claw;

    public static final double FORWARD_SPEED = 1;
    public static final double BACKWARD_SPEED = -1;

    public HariIntake() {
        claw = new TalonFX(CLAW_ID, RIO_BUS);

        var clawConfigurator = claw.getConfigurator();
        var configs = new TalonFXConfiguration();
        configs.MotorOutput.Inverted = CLAW_INVERSION;
        configs.MotorOutput.NeutralMode = CLAW_NEUTRAL_MODE;
        clawConfigurator.apply(configs);
    }

    private void setBoxSpeed(double speed) {
        claw.set(speed);
    }

    private void stopBox() {
        claw.stopMotor();
    }

    // Spins claw at the set speed, can be both in and out depending on speed
    public Command spinBox(double speed) {
        return startEnd(() -> setBoxSpeed(speed), this::stopBox);
    }

    public Command spinClawForward() {
        return spinBox(FORWARD_SPEED);
    }

    public Command spinClawBackward() {
        return spinBox(BACKWARD_SPEED);
    }
}
