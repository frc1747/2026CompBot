// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  // Attributes for simulation purposes
  private boolean simPoseInitialized = false;
  private boolean prevDSAttached = false;

  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run(); 
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().cancel(m_autonomousCommand);
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override

  public void simulationPeriodic() {
    boolean dsAttached = DriverStation.isDSAttached();

    if (!prevDSAttached && dsAttached) {
      simPoseInitialized = false;
    }
    prevDSAttached = dsAttached;

    if (!simPoseInitialized && DriverStation.getAlliance().isPresent()) {
      RobotContainer.drivetrain.resetPose(getStartingPose());
      simPoseInitialized = true;
    }
  }

  // Helper methods for determining bot's initial position on field
  private Pose2d getStartingPose() {
    Pose2d blueStartingPose = new Pose2d(1.5, 5.5, Rotation2d.fromDegrees(0));

    var alliance = DriverStation.getAlliance();
    if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
      return flipPose(blueStartingPose);
    }
    return blueStartingPose;
  }

  private Pose2d flipPose(Pose2d pose) {
    return new Pose2d(
      Constants.Field.FIELD_LENGTH - pose.getX(),
      Constants.Field.FIELD_WIDTH - pose.getY(),
      pose.getRotation().plus(Rotation2d.fromDegrees(180))
    );
  }
}
