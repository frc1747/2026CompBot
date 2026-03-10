
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentric;

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
import frc.robot.Constants.Drivetrain;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AprilLockDrive extends Command {
  /** Creates a new FaceObject. */
  private final CommandSwerveDrivetrain drivetrain;
  DoubleSupplier translationSup;
  DoubleSupplier strafeSup;
  private final FieldCentric swerveRequest;
  private final PIDController pid;

  // TODO: fix starting pose of robot
  public AprilLockDrive(CommandSwerveDrivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup) {
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.swerveRequest = new FieldCentric();
    this.pid = new PIDController(Constants.Vision.APRIL_LOCK_DRIVE_P, 
                                 Constants.Vision.APRIL_LOCK_DRIVE_I, 
                                 Constants.Vision.APRIL_LOCK_DRIVE_D);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double yawOffset = drivetrain.getYawOffset(new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y));
    // pid controlling rotation compensation
    double pidOutput = pid.calculate(yawOffset);
    double clampPid = MathUtil.clamp(pidOutput, -Constants.Vision.APRIL_LOCK_PID_CLAMP, Constants.Vision.APRIL_LOCK_PID_CLAMP);

    SmartDashboard.putNumber("pidOutputDrive", pidOutput);
    SmartDashboard.putNumber("clampPidDrive", clampPid);
    SmartDashboard.putNumber("yawOffsetDrive", yawOffset);
      
    drivetrain.setControl(
      swerveRequest
        .withVelocityX(-translationSup.getAsDouble() * Constants.Drivetrain.MAX_SPEED)
        .withVelocityY(-strafeSup.getAsDouble() * Constants.Drivetrain.MAX_SPEED)
        .withRotationalRate(clampPid) // May want to make a percent to multiply by max power
    );
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.setControl(
      swerveRequest
        .withVelocityX(0.0)
        .withVelocityY(0.0)
        .withRotationalRate(0.0)
    );
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
