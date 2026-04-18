// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.teleop;

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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.TargetPoses;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimeLight;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CompAprilLock extends Command {
  /** Creates a new FaceObject. */
  private final CommandSwerveDrivetrain drivetrain;
  private final DoubleSupplier translationSup;
  private final DoubleSupplier strafeSup;
  private final PIDController pid;
  private final SwerveRequest.FieldCentric swerveRequest;

  // TODO: fix starting pose of robot
  public CompAprilLock(CommandSwerveDrivetrain drivetrain, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup) {
    this.drivetrain = drivetrain;
    this.translationSup = translationSup;
    this.strafeSup = strafeSup;
    this.pid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
    this.swerveRequest = new SwerveRequest.FieldCentric();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // TODO: reformat to make more readable
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      // apply deadzone
      // translation value is forward/backward on left joystick
      double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.Controller.STICK_DEADBAND);
      // strafe value is left/right on left joystick
      double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.Controller.STICK_DEADBAND);
        
      // pose of apriltag on field, rotation represents angle of its normal vector
      //Pose2d apriltagPose = new Pose2d(new Translation2d(Constants.Vision.FIELD_CENTER_X, Constants.Vision.FIELD_CENTER_Y), new Rotation2d(0.0));
      Pose2d target = TargetPoses.getTargetPose();
      Pose2d robotPose = drivetrain.getState().Pose;
      
      // difference between robot and april tag poses
      Translation2d diff = robotPose.getTranslation().minus(target.getTranslation());
      // System.out.println("Diff: " + diff);
      // angle between diff and from vector(1, 0, 0)
      double diffAngle = Math.atan2(diff.getY(), diff.getX());
      // strafe angle is angle of a vector perpendicular to diff
      double strafeAngle = diffAngle + Math.PI / 2;
      // unit vector for direction of strafe at this moment
      Translation2d strafeDir = new Translation2d(Math.cos(strafeAngle), Math.sin(strafeAngle));
      // unit vector for direction towards target at this moment
      Translation2d translationDir = new Translation2d(-Math.cos(diffAngle), -Math.sin(diffAngle));
        
      // yaw offset between april tag normal vector and robot vector pointing directly out from camera
      double phi = Math.atan2(diff.getY(), diff.getX());
      double yawOffset = phi - robotPose.getRotation().getRadians() - Math.PI;
      double wrappedYaw = Math.atan2(Math.sin(yawOffset), Math.cos(yawOffset));
      // System.out.println("robotPose: " + robotPose);
      // System.out.println("yawOffset: " + yawOffset);
      // System.out.println("wrappedyaw: " + wrappedYaw);

      // pid controlling rotation compensation
      double pidOutput = -1 * pid.calculate(wrappedYaw); // not sure why it needs to be multiplied by -1
      double clampPid = pidOutput > Constants.Vision.APRIL_LOCK_PID_CLAMP ? Constants.Vision.APRIL_LOCK_PID_CLAMP : pidOutput;
      clampPid = clampPid < -Constants.Vision.APRIL_LOCK_PID_CLAMP ? -Constants.Vision.APRIL_LOCK_PID_CLAMP : clampPid;
      // System.out.println("pidOutput: " + pidOutput);
      // System.out.println("clampPid: " + clampPid);

      // TODO: check max speed math
      // strafe component of x component of final field oriented translation
      double strafeX = strafeDir.getX() * strafeVal * Constants.Drivetrain.MAX_SPEED * 0.5;
      // strafe component of y component of final field oriented translation
      double strafeY = strafeDir.getY() * strafeVal * Constants.Drivetrain.MAX_SPEED * 0.5;
      // forward/backward component of x component of final field oriented translation
      double translationX = translationDir.getX() * translationVal * Constants.Drivetrain.MAX_SPEED * 0.5;
      // forward/backward component of y component of final field oriented translation
      double translationY = translationDir.getY() * translationVal * Constants.Drivetrain.MAX_SPEED * 0.5;
      // rotation compensation power
      double rotation = clampPid * Constants.Drivetrain.MAX_ANGULAR_VELOCITY;
      
      // make drivetrain drive
      drivetrain.setControl(
        swerveRequest
          .withVelocityX(strafeX + translationX)
          .withVelocityY(strafeY + translationY)
          .withRotationalRate(rotation)
      );
  } 

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}