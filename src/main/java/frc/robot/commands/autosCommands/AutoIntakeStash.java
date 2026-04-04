package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import edu.wpi.first.wpilibj.Timer;


public class AutoIntakeStash extends Command{
    private IntakePivot intakePivot;
    private Timer timer = new Timer();

    public AutoIntakeStash(IntakePivot intakePivot){
        this.intakePivot = intakePivot;
        addRequirements(intakePivot);
    }


    @Override
    public void initialize(){
        timer.reset();
        timer.start();
        intakePivot.moveHomeCommand();
    }

    @Override 
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){

    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(3.0);
    }
}
