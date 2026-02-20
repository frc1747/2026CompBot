package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private TalonFXS motorWheels;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0.0);

    public Intake() {
        this.motorWheels = new TalonFXS(Constants.Intake.MOTOR_PORT);
        
        TalonFXSConfiguration config = new TalonFXSConfiguration();
        
        config.Voltage
            .withPeakForwardVoltage(12)
            .withPeakReverseVoltage(-12);

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        motorWheels.getConfigurator().apply(config);

    }
    public void intakeSpin(double power) {
        dutyCycle.Output = power;
        motorWheels.setControl(dutyCycle);
    }

    public void SetIntakePower(double armPower, double wheelsPower) {
        this.motorWheels.set(wheelsPower);
    }
}