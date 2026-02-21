// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Turret extends SubsystemBase {
  private TalonFXS motor;
  private Encoder encoder;
  // left from perspective of someone facing the turret sie of bot
  private DigitalInput leftLimitSwitch;
  private DigitalInput rightLimitSwitch;

  // optimization for not creating new control object 50/sec
  private DutyCycleOut dutyCycle = new DutyCycleOut(0);

  private final PIDController pid = new PIDController(Constants.Turret.PID_D, Constants.Turret.PID_D, Constants.Turret.PID_D);

  public Turret() {
    motor = new TalonFXS(Constants.Turret.MOTOR_PORT);
    TalonFXSConfiguration config = new TalonFXSConfiguration();
    
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.Voltage
      .withPeakForwardVoltage(12)
      .withPeakReverseVoltage(-12);
    
    config.Commutation.MotorArrangement = MotorArrangementValue.Minion_JST;
    motor.getConfigurator().apply(config);
    
    encoder = new Encoder(Constants.Turret.ENCODER_PORT_A, Constants.Turret.ENCODER_PORT_B);

    leftLimitSwitch = new DigitalInput(Constants.Turret.LEFT_LIMIT_PORT);
    rightLimitSwitch = new DigitalInput(Constants.Turret.RIGHT_LIMIT_PORT);

    pid.enableContinuousInput(0.0, 360.0);
    pid.setTolerance(1.0);
  }

  // left from perspective of a person facing turret side of robot
  public boolean getLeftLimitSwitchPressed() {
    // not (!) operator used because limit switch is normally open
    return !leftLimitSwitch.get();
  }

  // right from perspective of a person facing turret side of robot
  public boolean getRightLimitSwitchPressed() {
    // not (!) operator used because limit switch is normally open
    return !rightLimitSwitch.get();
  }

  // supplies power to spin turret but stops at limit switches
  // public void basicSpin(double power) {
  //  dutyCycle.Output = power;
  //  if (leftLimitSwitch.get()) {
  //    if (power > 0) { // > or <
  //      dutyCycle.Output = power;
  //    } else {
  //      dutyCycle.Output = 0.0;
  //    }
  //  } else if (rightLimitSwitch.get()) {
  //    if (power < 0) { // > or <
  //      dutyCycle.Output = power;
  //    } else {
  //      dutyCycle.Output = 0.0;
  //    }
  //  }
  //  motor.setControl(dutyCycle);
  //  return;
  // }

  public void basicSpin(double power) {
    if (getLeftLimitSwitchPressed()) {
      dutyCycle.Output = 0.0;
      motor.setControl(dutyCycle);
      return;
    } else if (getRightLimitSwitchPressed()) {
      dutyCycle.Output = 0.0;
      motor.setControl(dutyCycle);
      return;
    } else {
      if (getTurretAngle() < -20) {
        if (power > 0) {
          dutyCycle.Output = power;
          motor.setControl(dutyCycle);
          return;
        } else {
          dutyCycle.Output = 0.0;
          motor.setControl(dutyCycle);
          return;
        }
      } else if (getTurretAngle() > 20) {
        if (power < 0) {
          dutyCycle.Output = power;
          motor.setControl(dutyCycle);
          return;
        } else {
          dutyCycle.Output = 0.0;
          motor.setControl(dutyCycle);
          return;
        }
      }
      dutyCycle.Output = power;
      motor.setControl(dutyCycle);
      return;
    }
  }

  // currently incorrect because of gear ratio and absolute encoder degrees
  // also rename to getRelativeTurretAngle
  public double getTurretAngle() {
    return encoder.get() / 41.719;//Constants.Turret.TURRET_RATIO;
  }

  // returns pose of turret relative to field (absolute)
  public Pose2d getAbsTurretPose() {
    Pose2d robotPose = RobotContainer.drivetrain.getState().Pose;
    Rotation2d robotRotation = robotPose.getRotation();
    Translation2d robotLoc = robotPose.getTranslation();
    // unit vector pointing in the direction the robot is facing
    Translation2d robotDirVector = new Translation2d(Math.cos(robotRotation.getRadians()), Math.sin(robotRotation.getRadians()));
    // location of turret relative to bot center
    Translation2d relativeTurretLoc = robotDirVector.times(-Constants.Turret.DIST_TO_BOT_CENTER);
    // location of turret relative to field
    Translation2d absoluteTurretLoc = robotLoc.plus(relativeTurretLoc);
    // rotation of turret relative to field
    Rotation2d relativeTurretRotation = new Rotation2d(getTurretAngle() * Math.PI / 180.0); // how?
    // System.out.println("relativeTurretAngle: " + relativeTurretRotation);
    Rotation2d absoluteTurretRotation = robotRotation.plus(relativeTurretRotation);
    Pose2d absoluteTurretPose = new Pose2d(absoluteTurretLoc, absoluteTurretRotation);
    return absoluteTurretPose;
  }


  // TODO: Tune PID
  public void goToAngle(double targetAngle) {
    double currentAngle = getTurretAngle();
    double output = pid.calculate(currentAngle, targetAngle);

    // Safety
    output = MathUtil.clamp(output, Constants.Turret.GO_TO_ANGLE_LOWER_SAFETY, Constants.Turret.GO_TO_ANGLE_HIGHER_SAFETY);
    dutyCycle.Output = output;

    if ((Constants.Turret.LOWER_LIMIT <= targetAngle) && (targetAngle <= Constants.Turret.UPPER_LIMIT)){
      dutyCycle.Output = 0;
    }
    motor.setControl(dutyCycle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Turret/encoder value", encoder.get());
    SmartDashboard.putNumber("Turret/encoder angle", getTurretAngle());
    SmartDashboard.putNumber("Turret/turret degrees", getAbsTurretPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Left Limit Switch", getLeftLimitSwitchPressed());
    SmartDashboard.putBoolean("Right Limit Switch", getRightLimitSwitchPressed());
    //System.out.println("Turret Degrees: " + getAbsTurretPose().getRotation().getDegrees());
  }
}
