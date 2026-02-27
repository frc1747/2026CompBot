package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

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

    public Command SetIntakePowerCommand(){
        return runOnce( () -> intakeSpin(Constants.Intake.POWER));
    }

     public Command StopIntakePowerCommand(){
        return runOnce( () -> setIntakePower(0));
    }
}