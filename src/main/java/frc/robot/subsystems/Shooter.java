// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Shooter extends SubsystemBase {
  // shooting dir is froward.
  private TalonFX motorLeft;
  private TalonFX motorRight;
  private DutyCycleOut dutyCycleShooter = new DutyCycleOut(0);
  private DutyCycleEncoder encoder;
  private final PIDController pid = new PIDController(Constants.Shooter.PID_P, Constants.Shooter.PID_I, Constants.Shooter.PID_D);

  public Shooter() {
    motorLeft = new TalonFX(Constants.Shooter.MOTOR_LEFT_PORT);
    motorRight = new TalonFX(Constants.Shooter.MOTOR_RIGHT_PORT);
    encoder = new DutyCycleEncoder(Constants.Shooter.ENCODER_PORT);
    // this is use to set the control to follow master motor
    motorLeft.setControl(new Follower(Constants.Shooter.MOTOR_RIGHT_PORT, MotorAlignmentValue.Opposed));

    // config for the shooter motors
    TalonFXConfiguration configShooter = new TalonFXConfiguration();

    configShooter.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12);
    

    // the config for all the motors we should do differnt ones
    TalonFXConfiguration configHood = new TalonFXConfiguration();
    
    motorLeft.getConfigurator().apply(configShooter);
    motorRight.getConfigurator().apply(configShooter);

    pid.enableContinuousInput(0.0, 360.0);
    pid.setTolerance(1.0);
  }


  public double getPowerNeededFromDistanceAndAngle(double x, double y){
    // Z = A + BX + CY + DX^2 + FY^2 + EXY is the quady E Z is power, X is distance, Y is hood angle
    return (Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_D*Math.pow(x,2) + Constants.Shooter.SURFACE_F*Math.pow(y,2) + Constants.Shooter.SURFACE_E*x*y)/100;
  }

  public double getdistanceNeededFromAngleAndPower(double y, double z ){
    double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_C*y + Constants.Shooter.SURFACE_F*Math.pow(y,2) +- z*100;
    double B =  Constants.Shooter.SURFACE_B + Constants.Shooter.SURFACE_E;
    double A = Constants.Shooter.SURFACE_D;
    double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
    if (aws > 0) return aws;
    return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
    // slove with the good old quady for
  }

  public double getAngleNeededFromDistanceAndPower(double x, double z ){
    double C = Constants.Shooter.SURFACE_A + Constants.Shooter.SURFACE_B*x + Constants.Shooter.SURFACE_D*Math.pow(x,2) +- z*100;
    double B =  Constants.Shooter.SURFACE_C + Constants.Shooter.SURFACE_E;
    double A = Constants.Shooter.SURFACE_F;
    double aws = (- B + Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A; // we need to see if it's postive or negative
    if (aws > 0) return aws;
    return (- B - Math.sqrt( Math.pow(B, 2) - 4*A*C))/2*A;
    // slove with the good old quady for
  }

  public double[] findSpeedAndAngleFromDistance(double Distance){
    double currentAngle = RobotContainer.hood.getCurrentAngle();
    double wantedPower = getPowerNeededFromDistanceAndAngle(Distance, currentAngle);
    double[] array = {-1,-1};
    // this could be refactor 
    if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
     double[] angleAndSpeed = {currentAngle, wantedPower};
      return angleAndSpeed;
  }
  // we are assuming that greater hood angle is a furtuer Shoot
    for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle <= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle ++ ){
      if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
      if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
        double[] angleAndSpeed = {currentAngle, wantedPower};
        return angleAndSpeed;}
    }
    for (currentAngle = RobotContainer.hood.getCurrentAngle() ; currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE ; currentAngle -- ){
      if (currentAngle >= Constants.Shooter.MAX_HOOD_ANGLE) return array;
      if (wantedPower <= Constants.Shooter.MAX_AUTOSHOOT_POWER) {
        double[] angleAndSpeed = {currentAngle, wantedPower};
        return angleAndSpeed;}
    }
    return array;

  }



  @Override
  public void periodic() {
  }
}
