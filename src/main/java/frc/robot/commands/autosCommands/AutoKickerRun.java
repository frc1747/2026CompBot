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
        kicker.setPower(0.35);

    }

    @Override
    public void execute(){
        // if (timer.hasElapsed(5.0)){
        //     kicker.setPower(0.0);
        // }

    }
    @Override
    public void end(boolean interrupted){
        kicker.setPower(0.0);
        System.out.println("Kicker Has Been Stopped");
    }

    @Override
    public boolean isFinished() {
        //This Should
        return timer.hasElapsed(5.0);
    }
}
