// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hood extends SubsystemBase {
  private TalonFXS motor;
  private Encoder encoder;
  private double desiredAngle;
  private DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
  private PIDController pid = new PIDController(Constants.Hood.kP, Constants.Hood.kI, Constants.Hood.kD);


  public Hood() {
    SmartDashboard.putNumber("hood/Desired Angle", 25.0);
    motor = new TalonFXS(Constants.Hood.MOTOR_PORT);
    encoder = new Encoder(Constants.Hood.ENCODER_PORT_A, Constants.Hood.ENCODER_PORT_B);
    
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    config.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12);
    config.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;

    motor.getConfigurator().apply(config);
  }

  public Command setPowerCommand(boolean reverse) {
    return runOnce(() -> setPower((reverse ? -1 : 1) * Constants.Hood.MANUAL_MOTOR_POWER));
  }

  public Command stopCommand() {
    return runOnce(() -> setPower(0.0));
  }

  public Command goToAngleCommand(double angle) {
    return run(() -> goToAngle(angle));
  }

  public Command goToDesiredAngleCommand() {
    return run(() -> goToAngle(desiredAngle));
  }

  private void setPower(double power) {
    dutyCycle.Output = power;
    motor.setControl(dutyCycle);
  }

  public double countsToDegrees(double counts) {
    return ((counts) * Constants.Hood.TOTAL_HOOD_DEGREES / Constants.Hood.MAX_HEIGHT) + Constants.Hood.STARTING_ANGLE  ;
  }

  public double degreesToCounts(double degrees) {
    return ((degrees - Constants.Hood.STARTING_ANGLE ) /  Constants.Hood.TOTAL_HOOD_DEGREES) * Constants.Hood.MAX_HEIGHT ;
  }


  public double getCurrentAngle() {
    double counts = encoder.get();
    return countsToDegrees(counts);
  }


  public void resetEncoder() {
    encoder.reset();
  }


  public void goToAngle(double targetDegrees) {
    // don't go to an un-obtainable angle
   // targetDegrees = MathUtil.clamp(targetDegrees, 0.0, Constants.Hood.TOTAL_HOOD_DEGREES);
    dutyCycle.Output = pid.calculate(getCurrentAngle(), targetDegrees);

    motor.setControl(dutyCycle);
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("hood/encoder connected?", encoder.isConnected());
    SmartDashboard.putNumber("hood/encoder ticks", degreesToCounts(getCurrentAngle()));
    SmartDashboard.putNumber("hood/hood angle", getCurrentAngle());
    desiredAngle = SmartDashboard.getNumber("hood/Desired Angle", 25);
    SmartDashboard.putNumber("hood/PID", pid.calculate(getCurrentAngle(), desiredAngle));
    SmartDashboard.putNumber("hood/DutyCycle", dutyCycle.Output);
  }
}
