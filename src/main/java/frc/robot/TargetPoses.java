package frc.robot;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Turret;

// may want to make all static and manually schedule periodic to avoid making subsystem
public class TargetPoses extends SubsystemBase {

    // actual location the fuel should hit
    public Pose2d currentTarget;
    public PIDController lockPid;
    public Translation2d targetLocPrime;
    public Translation2d totalTurretVelocity;
    public Translation2d turretLoc;
    public boolean scoringMode;

    public TargetPoses() {
        // actual location the fuel should hit
        currentTarget = new Pose2d();
        lockPid = new PIDController(Constants.Vision.APRIL_LOCK_P, Constants.Vision.APRIL_LOCK_I, Constants.Vision.APRIL_LOCK_D);
        // location to aim the turret at
        targetLocPrime = currentTarget.getTranslation();
        totalTurretVelocity = new Translation2d();
        turretLoc = new Translation2d();
        scoringMode = true;
        updateTurretVelAndLoc();

        // schedule periodic (may not function, currently unnecessary as this is now a subsystem)
        // CommandScheduler.getInstance().schedule(Commands.run(TargetPoses::periodic));
        // SmartDashboard.putString("debug", "scheduled periodic");
    }

    public void reinitTargetLocPrime() {
        targetLocPrime = currentTarget.getTranslation();
    }

    private Pair<Translation2d, Double> getNextTargetApprox(Translation2d previousApproximation) {
        // distance from  turret to previous approximation
        double dist = turretLoc.getDistance(previousApproximation);

        // gets current hood angle in radians
        double hoodAngleRad = RobotContainer.hood.getCurrentAngle() * Math.PI / 180;

        // predicted amount of time between when the fuel leaves the
        // turret and when it reaches the height of the fuel hub
        // on its way down. should return null if the fuel
        // is predicted not to ever surpass the height of the top of
        // the hub
        // must check for null before use in final code
        Double travelTime = RobotContainer.turret.getFuelTravelTime(hoodAngleRad, dist);
        if (travelTime == null) return null;

        // next iteration adjusted targetting location
        Translation2d nextApprox = totalTurretVelocity.times(-1 * travelTime).plus(currentTarget.getTranslation());
        return new Pair<Translation2d, Double>(nextApprox, travelTime);
    }

    // takes a primary approximation of the point to aim the
    // turret at in order to shoot and hit the actual target
    // and a tolerance for difference in between calculated
    // travel times before the approximation is considered good,
    // then outputs a final approximation that should be reasonable
    // may need more or less iterations.
    private Translation2d getTargetApprox(Translation2d startApprox) {
        // may in choose to iterate the aproximation
        // several times in the future, using a travel
        // time tolerance to determine when to stop.
        // Constants.Vision.LEADING_TRAVEL_TIME_TOLERANCE
        // currently the approximation is always applied 6
        // times per scheduler loop, travelTime ignored
        Translation2d approxA;
        Translation2d approxB = startApprox;
        for (int i = 0; i < 6; i++) {
            approxA = approxB;
            approxB = getNextTargetApprox(approxA).getFirst();
            // return most recent valid approx if calculated approximation is null
            if (approxB == null) return approxA;
        }
        // RobotContainer.field.getObject("approxB").setPoses(new Pose2d(approxB, new Rotation2d()));
        // System.out.println("Getting target approx");
        // return the final approximation
        return approxB;
    }

    // updates the stored velocity and location of the turret
    private void updateTurretVelAndLoc() {
        // may need to find latency and predict future velocity
        // velocity of bot center relative to field
        Translation2d robotVel = RobotContainer.drivetrain.getV;
        // tangential velocity of turret relative to bot center, directed in field space
        Translation2d turretTangentialVel = RobotContainer.turret.getTangentialVelocity();
        // total velocity vector of turret relative to field
        // could instead be made into a turret subsysyem get method
        totalTurretVelocity = robotVel.plus(turretTangentialVel);
        // may need to take latency into account and find where the
        // robot will be in a very short time
        // current location of turret relative to field
        turretLoc = RobotContainer.turret.getAbsTurretPose().getTranslation();
        System.out.println("robotVelocity" + robotVel.toString());
    }

    public void shootOnTheMove() {
        // approximation of point to aim turret at
        targetLocPrime = getTargetApprox(targetLocPrime);
        RobotContainer.field.getObject("target").setPoses(new Pose2d(targetLocPrime, new Rotation2d()));
    }

    // rename to be accurate and refactor
    public Pose2d getTargetPose() {
        return new Pose2d(targetLocPrime, new Rotation2d());
    }

    public void setScoring() {
        scoringMode = true;
    }

    public void setShuttling() {
        scoringMode = false;
    }

    private Pose2d blueShuttling() {
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

    private Pose2d redshuttling() {
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

    public void fudgeTurretFactor(double radian) { // radians not dergees
        currentTarget = currentTarget.rotateAround(RobotContainer.drivetrain.getState().Pose.getTranslation(), new Rotation2d(radian));
    }

    public void fudgeShooterFactor(Pose2d botCurrentPose, double distance) { // distance in meters
        double theta = Math.atan2(currentTarget.getX() - botCurrentPose.getX(), currentTarget.getX()- botCurrentPose.getX() );
        double xPart = distance * Math.cos(theta);
        double yPart = distance * Math.sin(theta);
        currentTarget = new Pose2d(currentTarget.getX() + xPart, currentTarget.getY() + yPart, new Rotation2d());
    }

    // checks current mode and adjusts target accordingly
    public void periodic() {
        if (!scoringMode) {
            currentTarget = blueShuttling();
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                currentTarget = redshuttling();
            }
            reinitTargetLocPrime();
            RobotContainer.field.getObject("target").setPoses(this.getTargetPose());
        } else {
            scoringMode = true;
            currentTarget = Constants.TargetPosesConstants.BLUE_HUB_CENTER_POSE2D;
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red) {
                currentTarget = Constants.TargetPosesConstants.RED_HUB_CENTER_POSE2D;
            }
            reinitTargetLocPrime();
            RobotContainer.field.getObject("target").setPoses(this.getTargetPose());
        }
        updateTurretVelAndLoc();
    }

}
