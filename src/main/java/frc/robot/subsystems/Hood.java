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
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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

  // Simulation attributes
  private EncoderSim encoderSim;
  private double simCountAccumulator = 0.0;
  
  public Hood() {
    SmartDashboard.putNumber("hood/Desired Angle", Constants.Hood.MIN_ANGLE);
    motor = new TalonFXS(Constants.Hood.MOTOR_PORT);
    encoder = new Encoder(Constants.Hood.ENCODER_PORT_A, Constants.Hood.ENCODER_PORT_B);
    TalonFXSConfiguration config = new TalonFXSConfiguration();

    config.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12);
    config.Commutation.MotorArrangement = MotorArrangementValue.Brushed_DC;

    motor.getConfigurator().apply(config);

    if (RobotBase.isSimulation()) {
      encoderSim = new EncoderSim(encoder);
    }
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
    return ((counts * Constants.Hood.TOTAL_HOOD_DEGREES) / Constants.Hood.MAX_HEIGHT) + Constants.Hood.MIN_ANGLE;
  }

  public double degreesToCounts(double degrees) {
    return Math.abs((degrees - Constants.Hood.MIN_ANGLE ) /  Constants.Hood.TOTAL_HOOD_DEGREES) * Constants.Hood.MAX_HEIGHT ;
  }


  public double getCurrentAngle() {
    double counts = encoder.get();
    return countsToDegrees(counts);
  }

  public boolean atAngle(double angle){
    pid.calculate(getCurrentAngle(), angle);
    return pid.atSetpoint();
  }


  public void resetEncoder() {
    encoder.reset();
  }


  public void goToAngle(double targetDegrees) {
    targetDegrees = MathUtil.clamp(
      targetDegrees,
      Constants.Hood.MIN_ANGLE,
      Constants.Hood.MIN_ANGLE + Constants.Hood.TOTAL_HOOD_DEGREES
    );

    double pidOutput = pid.calculate(getCurrentAngle(), targetDegrees);
    dutyCycle.Output = -MathUtil.clamp(pidOutput, 
      -Constants.Hood.MAX_PID_OUTPUT, 
      Constants.Hood.MAX_PID_OUTPUT
    );
    motor.setControl(dutyCycle);
  }

  public boolean isDown() {
    return getCurrentAngle() >= Constants.Hood.MIN_ANGLE && getCurrentAngle() <= Constants.Hood.MIN_ANGLE + Constants.Hood.ANGLE_TOLERANCE;
  }


  @Override
  public void periodic() {
    double currentAngle = getCurrentAngle();

    // Global kill. If any method is at or disobeying the bounds,
    // SHUT IT DOWN! 
    if ((currentAngle <= Constants.Hood.MIN_ANGLE && dutyCycle.Output < 0) ||
        (currentAngle >= Constants.Hood.MAX_ANGLE && dutyCycle.Output > 0)) {
      dutyCycle.Output = 0.0;
      motor.setControl(dutyCycle);
    }

    // Allow an input from Elastic
    desiredAngle = SmartDashboard.getNumber("hood/Desired Angle", Constants.Hood.MIN_ANGLE);

    SmartDashboard.putNumber("hood/encoder ticks", degreesToCounts(getCurrentAngle()));
    SmartDashboard.putNumber("hood/hood angle", getCurrentAngle());
    SmartDashboard.putNumber("hood/PID", pid.calculate(getCurrentAngle(), desiredAngle));
    SmartDashboard.putNumber("hood/DutyCycle", dutyCycle.Output);
    SmartDashboard.putBoolean("hood/hood down", isDown());
    //SmartDashboard.getNumber("hood/Hood Desired Angle")

    if (Math.abs((desiredAngle - getCurrentAngle()) / getCurrentAngle()) <= 0.01) {
      SmartDashboard.putBoolean("hood/hood angle Reached", true);
    } else {
      SmartDashboard.putBoolean("hood/hood angle Reached", false);
    }
  }

  @Override
  public void simulationPeriodic() {
    simCountAccumulator += dutyCycle.Output * Constants.Hood.SIM_COUNTS_PER_SECOND * 0.02;
    int wholeCounts = (int) simCountAccumulator;
    simCountAccumulator -= wholeCounts;
    encoderSim.setCount(encoder.get() + wholeCounts);
  }
}
