package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Kicker;


public class AutoKickerRun extends Command{
    private Kicker kicker;
    private Timer timer = new Timer();

    public AutoKickerRun(Kicker kicker){
        this.kicker = kicker;
        addRequirements(kicker);

    }


    @Override
    public void initialize(){
        //Resets and Starts a timer
        timer.reset();
        timer.start();
        kicker.run(false);
    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        kicker.setPower(0.0);
        System.out.println("Kicker Has Been Stopped");
    }

    @Override
    public boolean isFinished() {
        //This Should
        return false;
    }
}
