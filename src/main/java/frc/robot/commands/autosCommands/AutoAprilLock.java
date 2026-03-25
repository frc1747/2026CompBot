// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autosCommands;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Turret;

public class AutoAprilLock extends Command {
    private final Turret turret;
    private final PIDController pid;
    private Timer timer = new Timer();

    // TODO: fix starting pose of robot
    public AutoAprilLock(Turret turret) {
        this.turret = turret;
        this.pid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        double yawOffset = turret.getYawOffset(new Translation2d(Constants.Vision.BLUE_HUB_CENTER_X, Constants.Vision.BLUE_HUB_CENTER_Y));
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            yawOffset = turret.getYawOffset(new Translation2d(Constants.Vision.RED_HUB_CENTER_X, Constants.Vision.RED_HUB_CENTER_Y));
        }

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

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        turret.basicSpin(0.0);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }
}
