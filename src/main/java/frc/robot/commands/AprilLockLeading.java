// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

/*
 * This extra logic is needed in order to decide where to aim the 
 * turret while the robot is moving because the velocity of the 
 * bot will transfer to the fuel, meaning that if the turret only
 * aims directly at the target, the fuel will fly off further
 * in the direction of the velocity of the bot and miss
 */
// UNTESTED
public class AprilLockLeading extends Command {
  /** Creates a new FaceObject. */
  private final Turret turret;
  private final PIDController pid;
  private final Translation2d targetLoc;
  private Translation2d targetLocPrime;
  private Translation2d totalTurretVelocity;
  private Translation2d turretLoc;

  // TODO: fix starting pose of robot
  public AprilLockLeading(Turret turret) {
    this.turret = turret;
    this.pid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
    // actual location of the target the fuel should hit
    targetLoc = new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y);
    // location to aim the turret at
    targetLocPrime = targetLoc;
    totalTurretVelocity = new Translation2d();
    turretLoc = new Translation2d();
    addRequirements(turret);
  }

  // takes the previous approximation for location to target
  // with the turret in order for the ball to hit the final 
  // target, and returns the next improved approximation
  // as well as the calculated travel time to the previous
  // approximation
  private Pair<Translation2d, Double> getNextTargetApprox(Translation2d prevApprox) {
    // distance from  turret to target
    double dist = turretLoc.getDistance(prevApprox);

    // gets ideal hood angle and power given distance 
    // returns value of {-1, -1} if the 
    // turret is incapable of any shooter speed and hood angle
    // combination to reach the specified distance
    // must check for {-1, -1} before use in final code
    // getPowerAndAngleFromDistance still needs to be implemented 
    // fully once empirical measurements are made
    double[] hoodAngleAndShooterSpeed = RobotContainer.shooter.getPowerAndAngleFromDistance(dist);
    double hoodAngle = hoodAngleAndShooterSpeed[0];
    double shooterSpeed = hoodAngleAndShooterSpeed[1];
    if (hoodAngle == -1 || shooterSpeed == -1) return null;
    
    // predicted amount of time between when the fuel leaves the
    // turret and when it reaches the height of the fuel hub 
    // on its way down. should return null if the fuel
    // is predicted not to ever surpass the height of the top of
    // the hub, but the shooter speed and hood angle calculated 
    // should ensure that never happens
    // must check for null before use in final code
    Double travelTime = turret.getFuelTravelTime(hoodAngle, shooterSpeed);
    if (travelTime == null) return null;

    // next iteration adjusted targetting location
    Translation2d nextApprox = totalTurretVelocity.times(-1 * travelTime).plus(targetLoc);
    return new Pair<Translation2d, Double>(nextApprox, travelTime);
  }

  // takes a primary approximation of the point to aim the
  // turret at in order to shoot and hit the actual target
  // and a tolerance for difference in between calculated
  // travel times before the approximation is considered good,
  // then outputs a final approximation that should be reasonable
  // may need more or less iterations.
  // code is repeated to avoid loop usage, perhaps a better method
  // is available taking advantage of the scheduer, but that
  // may cause too much latency
  private Translation2d getTargetApprox(Translation2d startApprox, double travelTimeTolerance) {
    // may in choose to iterate the aproximation
    // several times in the future, using a travel
    // time tolerance to determine when to stop.
    // currently the approximation is applied only 
    // once per scheduler loop, travelTime ignored
    Translation2d nextApprox = getNextTargetApprox(startApprox).getFirst();
    // return startApprox input if calculated approximation is null
    if (nextApprox == null) return startApprox;
    // return the calculated approximation if not null
    return nextApprox;
  }

  // updates the stored velocity and location of the turret
  private void updateTurretVelAndLoc() {
    // may need to find latency and predict future velocity
    // velocity of bot center relative to field
    Translation2d robotVel = RobotContainer.drivetrain.getVelocity();
    // tangential velocity of turret relative to bot center, directed in field space
    Translation2d turretTangentialVel = turret.getTangentialVelocity();
    // total velocity vector of turret relative to field
    // could instead be made into a turret subsysyem get method
    totalTurretVelocity = robotVel.plus(turretTangentialVel);
    // may need to take latency into account and find where the
    // robot will be in a very short time
    // current location of turret relative to field
    turretLoc = turret.getAbsTurretPose().getTranslation();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialize turret velocity and location
    updateTurretVelAndLoc();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // update turret velocity and location
    updateTurretVelAndLoc();
    // TODO: move magic number to constants
    // first iteration of approximation of point to aim turret at
    targetLocPrime = getTargetApprox(targetLocPrime, Constants.Vision.LEADING_TRAVEL_TIME_TOLERANCE);
    
    // ensures the belly pan falls off in the middle of the match
    RobotContainer.bellyPan.fallOff();
    
    double yawOffset = turret.getYawOffset(targetLocPrime);
    // pid controlling rotation compensation
    double pidOutput = pid.calculate(yawOffset); 
    double clampPid = MathUtil.clamp(pidOutput, -Constants.Vision.APRIL_LOCK_PID_CLAMP, Constants.Vision.APRIL_LOCK_PID_CLAMP);

    SmartDashboard.putNumber("pidOutput", pidOutput);
    SmartDashboard.putNumber("clampPid", clampPid);
    SmartDashboard.putNumber("yawOffset", yawOffset);
    turret.basicSpin(clampPid);
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    turret.basicSpin(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
