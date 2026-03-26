package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class TargetPoses{

    public Pose2d CurrentTarget;

    private final double blueShuttleX = 2.0;
    private final double redShuttleX = 14.0;

    public TargetPoses(){
        setScoring();

    }

    public Pose2d getTargetPose(){
        return CurrentTarget;
    }

    public void setScoring(){
        System.out.println(Constants.TargetPosesConstants.blueHubCenter);
        CurrentTarget = Constants.TargetPosesConstants.blueHubCenter;
         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            CurrentTarget = Constants.TargetPosesConstants.redHubCenter;
        }
    }

    public void setShuttling(){
        CurrentTarget = blueShuttling();
         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            CurrentTarget = redshuttling();
        }
    }

    private Pose2d blueShuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        if (turretPose.getX() > Constants.TargetPosesConstants.BLUE_DEADZONE_MIN && turretPose.getX() < Constants.TargetPosesConstants.BLUE_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPosesConstants.BLUE_DEADZONE_MIN > turretPose.getX() - Constants.TargetPosesConstants.BLUE_DEADZONE_MAX ) {
                turretPose = Constants.TargetPosesConstants.blueLeftShuttlePose2d;
            }
            turretPose = Constants.TargetPosesConstants.blueRightShuttlePose2d;
        }   
        return turretPose;
    }

    private Pose2d redshuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        if (turretPose.getX() > Constants.TargetPosesConstants.RED_DEADZONE_MIN && turretPose.getX() < Constants.TargetPosesConstants.RED_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPosesConstants.RED_DEADZONE_MIN > turretPose.getX() - Constants.TargetPosesConstants.RED_DEADZONE_MAX ) {
                turretPose = Constants.TargetPosesConstants.redLeftShuttlePose2d;
            }
            turretPose = Constants.TargetPosesConstants.redRightShuttlePose2d;
        }   
        return turretPose;
    }

    public void ShootOnTheMove() {
        
    }

    public void fudgeTurretFactor(double radian) { // radians not dergees
        CurrentTarget = CurrentTarget.rotateAround(RobotContainer.drivetrain.getState().Pose.getTranslation(), new Rotation2d(radian));
    }

    public void fudgeShooterFactor(Pose2d botCurrentPose, double distance) { // distance in meters
        double theta = Math.atan2(CurrentTarget.getX() - botCurrentPose.getX() , CurrentTarget.getX()- botCurrentPose.getX() );
        double xPart = distance * Math.cos(theta);
        double yPart = distance * Math.sin(theta);
        CurrentTarget = new Pose2d(CurrentTarget.getX() + xPart, CurrentTarget.getY() + yPart, new Rotation2d());
    }
    
}
