
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
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilLock extends Command {
  /** Creates a new FaceObject. */
  private final Turret turret;
  private final PIDController pid;

  // TODO: fix starting pose of robot
  public AprilLock(Turret turret) {
    this.turret = turret;
    this.pid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(turret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double yawOffset = turret.getYawOffset(new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y));
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
