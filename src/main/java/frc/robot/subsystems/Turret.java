// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import com.ctre.phoenix6.sim.TalonFXSSimState;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
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
  private double targetPower;
  
  public double distanceToHub;

  // Fields required for simulation mode
  private DCMotorSim motorSim;
  private EncoderSim encoderSim;
  private TalonFXSSimState talonSimState;
  private DIOSim leftLimitSim;
  private DIOSim rightLimitSim;

  // optimization for not creating new control object 50/sec
  private DutyCycleOut dutyCycle = new DutyCycleOut(0);

  private final PIDController pid = new PIDController(Constants.Turret.PID_P, Constants.Turret.PID_I, Constants.Turret.PID_D);

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

    targetPower = 0.0;

    pid.enableContinuousInput(0.0, 360.0);
    pid.setTolerance(1.0);

    if (RobotBase.isSimulation()) {
      // Encoder is on the motor shaft side of the 15:110 reduction.
      // DCMotorSim needs the gearing FROM motor TO the simulated shaft.
      // Since encoder IS the motor shaft here, gearing = 1.0.
      // We use Minion motor (comparable to NEO 550 class) -- adjust if needed.
      motorSim = new DCMotorSim(
          LinearSystemId.createDCMotorSystem(
              DCMotor.getMinion(1),
              0.0001,  // moment of inertia (kg·m²) -- tune this
              1.0              // motor-to-encoder gear ratio (1:1, encoder on motor shaft)
          ),
          DCMotor.getMinion(1)
      );
      encoderSim = new EncoderSim(encoder);
      talonSimState = motor.getSimState();

      leftLimitSim = new DIOSim(leftLimitSwitch);
      rightLimitSim = new DIOSim(rightLimitSwitch);
    }

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
    // dutyCycle.Output = power;
    targetPower = power;
  }

  // TODO: also rename and refactor to getRelativeTurretAngle
  public double getTurretAngle() {
    // gear ratio 15 to 110, 110/15 ~= 7.333
    // 8196 pulses from encoder per rotation
    // 4x quadrature
    // 8196 / 4 = 2048
    // 2048 * 7.333 ~= 15018.667 resulting pulses per full turret rotation
    // 15018.667 / 360 degrees ~= 41.719
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

  public double getYawOffset(Translation2d targetLoc) {
    Pose2d turretPose = getAbsTurretPose();
      
    // difference between robot and april tag poses
    Translation2d diff = turretPose.getTranslation().minus(targetLoc);
        
    // yaw offset between target and robot vector pointing directly out from robot-front
    double phi = Math.atan2(diff.getY(), diff.getX());
    double yawOffset = phi - turretPose.getRotation().getRadians() - Math.PI;
    double wrappedYaw = Math.atan2(Math.sin(yawOffset), Math.cos(yawOffset));
    return wrappedYaw;
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
    dutyCycle.Output = targetPower;
    // soft limits and limit switches
    if (getLeftLimitSwitchPressed() && targetPower > 0) {
      dutyCycle.Output = 0.0;
    } 
    if (getRightLimitSwitchPressed() && targetPower < 0) {
      dutyCycle.Output = 0.0;
    }
    if (getTurretAngle() > Constants.Turret.TURRET_YAW_LIMIT && targetPower < 0) {
      dutyCycle.Output = 0.0;
    }
    if (getTurretAngle() < -Constants.Turret.TURRET_YAW_LIMIT && targetPower > 0) {
      dutyCycle.Output = 0.0;
    }
    motor.setControl(dutyCycle);

    // SmartDashboard
    SmartDashboard.putNumber("Turret/encoder value", encoder.get());
    SmartDashboard.putNumber("Turret/encoder angle", getTurretAngle());
    SmartDashboard.putNumber("Turret/turret degrees", getAbsTurretPose().getRotation().getDegrees());
    SmartDashboard.putBoolean("Left Limit Switch", getLeftLimitSwitchPressed());
    SmartDashboard.putBoolean("Right Limit Switch", getRightLimitSwitchPressed());

    Translation2d hubLoc = new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y);
    distanceToHub = getAbsTurretPose().getTranslation().getDistance(hubLoc);
    SmartDashboard.putNumber("Hub Distance From Turret", distanceToHub);
    //System.out.println("Turret Degrees: " + getAbsTurretPose().getRotation().getDegrees());
  }

  @Override
  public void simulationPeriodic() {
    // 1. Read what voltage the TalonFXS is commanding
    talonSimState.setSupplyVoltage(RobotController.getBatteryVoltage());
    double motorVoltage = talonSimState.getMotorVoltage();

    // 2. Step the physics simulation (20ms loop)
    motorSim.setInputVoltage(motorVoltage);
    motorSim.update(0.020);

    // 3. Push simulated position back to the EncoderSim.
    //    getTurretAngle() divides raw count by 41.719 to get degrees.
    //    Invert matches the negative sign in getTurretAngle().
    double motorRevolutions = motorSim.getAngularPositionRotations();
    // encoder CPR = 2048 (2048 raw pulses/rev in 4x quadrature mode)
    int rawCounts = (int) (-motorRevolutions * 1024);
    encoderSim.setCount(rawCounts);

    // 4. Also feed velocity so any rate-based logic stays consistent
    double motorRPS = motorSim.getAngularVelocityRPM() / 60.0;
    encoderSim.setRate(-motorRPS * 1024); // counts per second

    // 5. Update our limit switches
    double turretAngle = getTurretAngle();
    leftLimitSim.setValue(!(turretAngle <= -Constants.Turret.TURRET_YAW_LIMIT));
    rightLimitSim.setValue(!(turretAngle >= Constants.Turret.TURRET_YAW_LIMIT));
  }

}
