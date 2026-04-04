package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakePivot;
import edu.wpi.first.wpilibj.Timer;


public class AutoIntakeLower extends Command{
    private IntakePivot intakePivot;
    private Timer timer = new Timer();

    public AutoIntakeLower(IntakePivot intakePivot){
        this.intakePivot = intakePivot;
        addRequirements(intakePivot);
    }


    @Override
    public void initialize(){
        //Resets and Starts a timer
        timer.reset();
        timer.start();
        intakePivot.moveOutCommand();
    }

    @Override 
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        //IntakePivot does not have a stop Command method. Would like to see if e vould add one
        intakePivot.setPower(0.0);
    }

    @Override
    public boolean isFinished() {
        //This Should 
        return timer.hasElapsed(3.0);
    }
}
