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

    public static void setScoring() {
        System.out.println(Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D);
        currentTarget = Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D;
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            currentTarget = Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D;
        }
    }

    public static void setShuttling() {
        currentTarget = blueShuttling();
        if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
            currentTarget = redshuttling();
        }
    }

    private static Pose2d blueShuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        if (turretPose.getX() > Constants.TargetPosesConstants.BLUE_DEADZONE_MIN && turretPose.getX() < Constants.TargetPosesConstants.BLUE_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPosesConstants.BLUE_DEADZONE_MIN > turretPose.getX() - Constants.TargetPosesConstants.BLUE_DEADZONE_MAX ) {
                turretPose = Constants.TargetPosesConstants.BLUE_LEFT_SHUTTLE_POSE2D;
            }
            turretPose = Constants.TargetPosesConstants.BLUE_RIGHT_SHUTTLE_POSE2D;
        }
        return turretPose;
    }

    private static Pose2d redshuttling() {
        Pose2d turretPose = RobotContainer.turret.getAbsTurretPose();
        if (turretPose.getX() > Constants.TargetPosesConstants.RED_DEADZONE_MIN && turretPose.getX() < Constants.TargetPosesConstants.RED_DEADZONE_MAX ){
            if (turretPose.getX() - Constants.TargetPosesConstants.RED_DEADZONE_MIN > turretPose.getX() - Constants.TargetPosesConstants.RED_DEADZONE_MAX ) {
                turretPose = Constants.TargetPosesConstants.RED_LEFT_SHUTTLE_POSE2D;
            }
            turretPose = Constants.TargetPosesConstants.RED_RIGHT_SHUTTLE_POSE2D;
        }
        return turretPose;
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
