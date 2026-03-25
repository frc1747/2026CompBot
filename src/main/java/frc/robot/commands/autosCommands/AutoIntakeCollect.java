package frc.robot.commands.autosCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AutoIntakeCollect extends Command {
    private Intake intake;
    private double power;
    private Timer timer = new Timer();

    public AutoIntakeCollect(Intake intake, double power) {
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
        return timer.hasElapsed(3.0); // run for 1 second
    }
}
