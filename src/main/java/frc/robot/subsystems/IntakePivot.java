package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakePivot extends SubsystemBase {
    private TalonFXS motorArm;
    private DutyCycleOut dutyCycle = new DutyCycleOut(0);

    public IntakePivot() {
        this.motorArm = new TalonFXS(Constants.Intake.MOTOR_ARM_PORT);
        
        var request = new PositionDutyCycle(.5);
        motorArm.setControl(request);

        TalonFXSConfiguration config = new TalonFXSConfiguration();
        
        config.Voltage
            .withPeakForwardVoltage(12);

        config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
        motorArm.getConfigurator().apply(config);

    }
    public void intakepivot(double tick) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'intakespin'");
    }

    public void SetIntakePower(double armPower, double wheelsPower) {
        this.motorArm.set(armPower);
    }
}