package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Hopper;


public class AutoHopperRun extends Command{
    private Hopper hopper;
    private Timer timer = new Timer();

    public AutoHopperRun(Hopper hopper){
        this.hopper = hopper;
        addRequirements(hopper);

    }


    @Override
    public void initialize(){
        //Resets and Starts a timer
        timer.reset();
        timer.start();
        hopper.run(false);
    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        hopper.setPower(0.0);
        System.out.println("The Hopper Has Been Stopped");
    }

    @Override
    public boolean isFinished() {
        //This Should
        return timer.hasElapsed(3.0);
    }
}
