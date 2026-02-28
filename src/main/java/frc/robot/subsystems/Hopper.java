// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;


public class Hopper extends SubsystemBase {
    private TalonFX motor;
    private DutyCycleOut dutyControl = new DutyCycleOut(0.0);

    public Hopper() {
        motor = new TalonFX(Constants.Hopper.MOTOR_PORT);
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.withNeutralMode(NeutralModeValue.Coast);
        config.CurrentLimits
            .withStatorCurrentLimit(60)
            .withStatorCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLowerLimit(40)
            .withSupplyCurrentLimitEnable(true);
    
        motor.getConfigurator().apply(config);
        motor.getStatorCurrent().setUpdateFrequency(50);
        motor.getSupplyVoltage().setUpdateFrequency(50);
        motor.getVelocity().setUpdateFrequency(50);
        motor.getDutyCycle().setUpdateFrequency(100);
        motor.getMotorVoltage().setUpdateFrequency(50);
        motor.getSupplyCurrent().setUpdateFrequency(50);
        motor.optimizeBusUtilization();
    }

    public Command run() {
        return runOnce(() -> setPower(Constants.Hopper.MOTOR_POWER));
    }

    public Command stop() {
        return runOnce(() -> setPower(0.0));
    }
    
    public void setPower(double power) {
        dutyControl.Output = power;
        motor.setControl(dutyControl);
    }

    @Override
    public void periodic() {}

}


