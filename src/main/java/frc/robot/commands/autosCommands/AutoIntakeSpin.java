package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.Timer;


public class AutoIntakeSpin extends Command{
    private Intake intake;
    private Timer timer = new Timer();
    private boolean reversed;

    public AutoIntakeSpin(Intake intake , boolean reversed){
        this.intake = intake;
        this.reversed = reversed;
        addRequirements(intake);

    }


    @Override
    public void initialize(){
        //Resets and Starts a timer
        timer.reset();
        timer.start();
        intake.spin(reversed);
    }

    @Override 
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        intake.StopCommand();
    }

    @Override
    public boolean isFinished() {
        //This Should 
        return timer.hasElapsed(3.0);
    }
}
