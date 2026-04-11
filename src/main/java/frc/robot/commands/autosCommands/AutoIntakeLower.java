package frc.robot.commands.autosCommands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.IntakePivot;


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
        //    if (intakePivot.getDown()) {
        //      intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_UP);
        //  } else {
        //      intakePivot.setDefaultPosition(Constants.IntakePivot.ENCODER_READY);
        //  }
        // intakePivot.toggleDown();
        intakePivot.setPower(0.35);
        intakePivot.intakePivot(Constants.IntakePivot.OUT);
    }

    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
        //IntakePivot does not have a stop Command method. Would like to see if e vould add one
        intakePivot.setPower(0.0);
        System.out.println("IntakeLower Has Been Ended");
    }

    @Override
    public boolean isFinished() {
        //This Should
        return timer.hasElapsed(2);
    }
}
