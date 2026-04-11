package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;


public class AutoIntakeSpin extends Command{
    private Intake intake;
    private Timer timer = new Timer();

    public AutoIntakeSpin(Intake intake){
        this.intake = intake;
        addRequirements(intake);

    }


    @Override
    public void initialize(){
        //Resets and Starts a timer
        timer.reset();
        timer.start();
        intake.intakeSpin(0.85);
    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        intake.intakeSpin(0.0);
        System.out.println("Intake Spin has Been Stopped");
    }

    @Override
    public boolean isFinished() {
        //This Should
        return timer.hasElapsed(3);
    }
}
