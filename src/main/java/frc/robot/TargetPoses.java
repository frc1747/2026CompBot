package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TargetPoses extends SubsystemBase{

    public Pose2d CurrentTarget;

    private final double blueShuttleX = 0.0;
    private final double redShuttleX = 0.0;

    public TargetPoses(){

    }

    public Pose2d getTargetPose(){
        return CurrentTarget;
    }

    public Command setScoringCommand() {
        return runOnce(() -> setScoring());
    }

    public Command setShuttlingCommand() {
        return runOnce(() -> setShuttling());
    }

    public void setScoring(){
        CurrentTarget = Constants.TargetPoses.blueHubCenter;
         if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            CurrentTarget = Constants.TargetPoses.redHubCenter;
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
        Pose2d shuttlePose = new Pose2d(blueShuttleX, turretPose.getY(), new Rotation2d());
        if (turretPose.getX() > Constants.TargetPoses.BLUE_DEADZONE_MIN && turretPose.getX() < Constants.TargetPoses.BLUE_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPoses.BLUE_DEADZONE_MIN > turretPose.getX() - Constants.TargetPoses.BLUE_DEADZONE_MAX ) {
                turretPose = Constants.TargetPoses.blueLeftShuttlePose2d;
            }
            turretPose = Constants.TargetPoses.blueRightShuttlePose2d;
        }
        return turretPose;
    }

    private Pose2d redshuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        Pose2d shuttlePose = new Pose2d(redShuttleX, turretPose.getY(), new Rotation2d());
        if (turretPose.getX() > Constants.TargetPoses.RED_DEADZONE_MIN && turretPose.getX() < Constants.TargetPoses.RED_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPoses.RED_DEADZONE_MIN > turretPose.getX() - Constants.TargetPoses.RED_DEADZONE_MAX ) {
                turretPose = Constants.TargetPoses.redLeftShuttlePose2d;
            }
            turretPose = Constants.TargetPoses.redRightShuttlePose2d;
        }
        return turretPose;
    }

}
