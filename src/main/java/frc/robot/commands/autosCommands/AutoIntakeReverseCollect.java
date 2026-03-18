package frc.robot.commands.autosCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeReverseCollect extends Command {
    private Intake intake;
    private double power;
    private Timer timer = new Timer();

    public AutoIntakeReverseCollect(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intake.intakeSpin(power);
    }

    @Override
    public void execute() {
    }

    @Override
    public void end(boolean interrupted) {
        intake.intakeSpin(0.0);
    }
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }
}