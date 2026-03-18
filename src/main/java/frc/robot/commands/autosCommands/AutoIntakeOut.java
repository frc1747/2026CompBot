package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import edu.wpi.first.wpilibj.Timer;

public class AutoIntakeOut extends Command {
    private IntakePivot intakePivot;
    private int tick;
    private Timer timer = new Timer();

    public AutoIntakeOut(IntakePivot intakePivot, int tick) {
        this.intakePivot = intakePivot;
        this.tick = tick;
        addRequirements(intakePivot);

    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        intakePivot.intakePivot(tick);

    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        intakePivot.intakePivot(0);
    } 
    @Override
    public boolean isFinished() {
        return timer.hasElapsed(1.0); // run for 1 second
    }
}