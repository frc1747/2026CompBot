package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFX motor;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    public Intake() {
        this.motor = new TalonFX(Constants.Intake.MOTOR_PORT);

        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);
        
        config.CurrentLimits
            .withStatorCurrentLimit(80)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(60)
            .withSupplyCurrentLowerLimit(60)
            .withSupplyCurrentLimitEnable(true);

        config.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
        motor.getConfigurator().apply(config);

    }
    public void intakeSpin(double power) {
        dutyCycle.Output = power;
        motor.setControl(dutyCycle);
    }

    public void setIntakePower(double wheelsPower) {
        this.motor.set(wheelsPower);
    }

    public Command spin() {

        return run( () -> intakeSpin(Constants.Intake.POWER));
    }

    public Command StopCommand() {

        return runOnce( () -> setIntakePower(0));
    }
    public Command spin(boolean reverse) {
        return runOnce(() -> setIntakePower((reverse ? -1 : 1) * Constants.Intake.POWER));
    }
}
