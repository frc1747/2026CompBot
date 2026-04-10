// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import monologue.Monologue;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

    private final Timer userCodeTimer = new Timer();
    private final Timer dtTimer = new Timer();

    public Robot() {
        Timer initTimer = new Timer();
        initTimer.start();

        DataLogManager.start();
        DriverStation.startDataLog(DataLogManager.getLog());
        DriverStation.silenceJoystickConnectionWarning(!DriverStation.isFMSAttached());

        m_robotContainer = new RobotContainer();

        Monologue.setupMonologue(m_robotContainer, "Robot", false, false);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
        gitInit();
        initTimer.stop();
        m_robotContainer.log("Timing/Robot init ms", initTimer.get() * 1000);
        dtTimer.start();
    }

    @Override
    public void robotPeriodic() {
        m_robotContainer.log("Timing/Loop dt ms", dtTimer.get() * 1000);
        dtTimer.restart();
        userCodeTimer.restart();

        CommandScheduler.getInstance().run();
        Monologue.updateAll();
        m_robotContainer.log("Timing/User code ms", userCodeTimer.get() * 1000);
    }

    public void gitInit() {
        String warning = BuildConstants.DIRTY == 1 ? "WARNING: The code was built with uncommitted changes!\n" : "";
        String info = String.format("""
        ===============================
        Git information:
        SHA: %s
        Branch: %s
        Commit Date: %s
        Build Date: %s
        %s==============================
        """,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_BRANCH,
        BuildConstants.GIT_DATE,
        BuildConstants.BUILD_DATE,
        warning
        );
        System.out.print(info);
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

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
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
