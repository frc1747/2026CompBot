
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
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

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilLockLeading extends Command {
  /** Creates a new FaceObject. */
  private final Turret turret;
  private final PIDController pid;

  // TODO: fix starting pose of robot
  public AprilLockLeading(Turret turret) {
    this.turret = turret;
    this.pid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // UNFINISHED AND UNTESTED
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // location of target to shoot ball at
    Translation2d targetLoc = new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y);
    
    // may need to find latency and predict future velocity
    // velocity of bot center relative to field
    Translation2d robotVel = RobotContainer.drivetrain.getVelocity();
    // tangential velocity of turret relative to bot center, directed in field space
    Translation2d turretTangentialVel = turret.getTangentialVelocity();
    // total velocity vector of turret relative to field
    // could instead be made into a turret subsysyem get method
    Translation2d totalTurretVelocity = robotVel.plus(turretTangentialVel);

    // may need to take latency into account and find where the
    // robot will be in a very short time
    // current location of turret relative to field
    Translation2d turretLoc = turret.getAbsTurretPose().getTranslation();
    // distance from  turret to target
    double dist = turretLoc.getDistance(targetLoc);

    // gets ideal hood angle and power given distance 
    // returns value of {-1, -1} if the 
    // turret is incapable of any shooter speed and hood angle
    // combination to reach the specified distance
    // must check for {-1, -1} before use in final code
    // getPowerAndAngleFromDistance still needs to be implemented 
    // fully once empirical measurements are made
    double[] hoodAngleAndShooterSpeed = RobotContainer.shooter.getPowerAndAngleFromDistance(dist);
    Double hoodAngle = hoodAngleAndShooterSpeed[0];
    Double shooterSpeed =hoodAngleAndShooterSpeed[1];

    // predicted amount of time between when the fuel leaves the
    // turret and when it reaches the height of the fuel hub 
    // on its way down. should return null if the fuel
    // is predicted not to ever surpass the height of the top of
    // the hub, but the shooter speed and hood angle calculated 
    // should ensure that never happens
    // must check for null before use in final code
    // Double travelTime = turret.getFuelTravelTime(hoodAngle, shooterSpeed);

    // first iteration adjusted targetting location
    // Translation2d targetLocPrime = totalTurretVelocity.times(-1 * travelTime).plus(targetLoc);
    // take this new targetting location and plug back into the
    // above logic; get distance from bot, then shooter speed and // hood angle, then fuel travel time, then a new targetting   // location. repeat this process several times and the        // targetLocPrime and travelTime should converge onto certain // values close enough to the ideal to use.
    // use the change in calculated travel time to determine      // whether the values have converged closely enough.
    // once the values have converged enough, target the final
    // calculated targetting position with the turret

    // ensures the belly pan falls off in the middle of the match
    // bellypan.fallOff();

    // ---- previous working code ----
    
    double yawOffset = turret.getYawOffset(targetLoc);
    // pid controlling rotation compensation
    double pidOutput = pid.calculate(yawOffset); 
    double clampPid = pidOutput;
    if (clampPid > Constants.Vision.APRIL_LOCK_PID_CLAMP) {
      clampPid = Constants.Vision.APRIL_LOCK_PID_CLAMP;
    } else if (clampPid < -Constants.Vision.APRIL_LOCK_PID_CLAMP) {
      clampPid = -Constants.Vision.APRIL_LOCK_PID_CLAMP;
    }

    SmartDashboard.putNumber("pidOutput", pidOutput);
    SmartDashboard.putNumber("clampPid", clampPid);
    SmartDashboard.putNumber("yawOffset", yawOffset);
    // double clampPid = MathUtil.clamp(pidOutput, -Constants.Vision.APRIL_LOCK_PID_CLAMP, Constants.Vision.APRIL_LOCK_PID_CLAMP);
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
