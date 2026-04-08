package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class TargetPoses{

    public static Pose2d currentTarget;

    public TargetPoses() {
        currentTarget = new Pose2d();
    }

    public static Pose2d getTargetPose() {
        return currentTarget;
    }

    public static double getTargetAutoShootMult() {
        if (currentTarget == Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D || currentTarget == Constants.TargetPosesConstants.BLUE_HUB_CENTER_POSE2D) {
            return Constants.Shooter.AUTO_SHOOTER_MULT_HUB; 
        } else {
            return Constants.Shooter.AUTO_SHOOTER_MULT_SHUTTLE;
        }
    }

    public static void setScoring() {
        RobotContainer.field.getObject("target").setPoses(TargetPoses.getTargetPose());
        currentTarget = Constants.TargetPosesConstants.BLUE_HUB_CENTER_POSE2D;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            currentTarget = Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D;
        }
    }

    public static void setShuttling() {
        RobotContainer.field.getObject("target").setPoses(TargetPoses.getTargetPose());
        currentTarget = blueShuttling();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            currentTarget = redshuttling();
        }
    }

    private static Pose2d blueShuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        Pose2d targetPose = new Pose2d( Constants.Vision.BLUE_SHUTTLE_CENTER_X, turretPose.getY(), new Rotation2d());
        if (turretPose.getY() > Constants.TargetPosesConstants.BLUE_DEADZONE_MIN && turretPose.getY() < Constants.TargetPosesConstants.BLUE_DEADZONE_MAX ){
            if (turretPose.getY() - Constants.TargetPosesConstants.BLUE_DEADZONE_MIN > Math.abs(turretPose.getY() - Constants.TargetPosesConstants.BLUE_DEADZONE_MAX) ) {
                targetPose = Constants.TargetPosesConstants.BLUE_LEFT_SHUTTLE_POSE2D;
            }
            targetPose = Constants.TargetPosesConstants.BLUE_RIGHT_SHUTTLE_POSE2D;
        }
        return targetPose;
    }

    private static Pose2d redshuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        Pose2d targetPose = new Pose2d( Constants.Vision.RED_SHUTTLE_CENTER_X, turretPose.getY(), new Rotation2d());
        if (turretPose.getY() > Constants.TargetPosesConstants.RED_DEADZONE_MIN && turretPose.getY() < Constants.TargetPosesConstants.RED_DEADZONE_MAX ){
            if (turretPose.getY() - Constants.TargetPosesConstants.RED_DEADZONE_MIN > Math.abs(turretPose.getY() - Constants.TargetPosesConstants.RED_DEADZONE_MAX) ) {
                targetPose = Constants.TargetPosesConstants.RED_LEFT_SHUTTLE_POSE2D;
            }
            targetPose = Constants.TargetPosesConstants.RED_RIGHT_SHUTTLE_POSE2D;
        }
        return targetPose;
    }

    public static void ShootOnTheMove() {

    }

    public static void fudgeTurretFactor(double radian) { // radians not dergees
        currentTarget = currentTarget.rotateAround(RobotContainer.drivetrain.getState().Pose.getTranslation(), new Rotation2d(radian));
    }

    public static void fudgeShooterFactor(Pose2d botCurrentPose, double distance) { // distance in meters
        double theta = Math.atan2(currentTarget.getX() - botCurrentPose.getX(), currentTarget.getX()- botCurrentPose.getX() );
        double xPart = distance * Math.cos(theta);
        double yPart = distance * Math.sin(theta);
        currentTarget = new Pose2d(currentTarget.getX() + xPart, currentTarget.getY() + yPart, new Rotation2d());
    }

}
