package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public class TargetTranslation {

    public enum Target {
        SCORING,
        SHUTTLE
    }

    private final Supplier<Translation2d> turretTranslationProvider;

    public TargetTranslation(Supplier<Translation2d> turretTranslationProvider) {
        this.turretTranslationProvider = turretTranslationProvider;
    }

    public Translation2d getTranslation(Target target) {
        return switch (target) {
            case SCORING -> scoringTranslation();
            case SHUTTLE -> shuttleTranslation();
        };
    }

    private Translation2d scoringTranslation() {
        return isRedAlliance()
            ? Constants.TargetTranslation.RED_HUB_CENTER
            : Constants.TargetTranslation.BLUE_HUB_CENTER;
    }

    private Translation2d shuttleTranslation() {
        return isRedAlliance() ? redShuttleTranslation() : blueShuttleTranslation();
    }

    private Translation2d blueShuttleTranslation() {
        double turretY = turretTranslationProvider.get().getY();
        double resolvedY = clampToShuttleLine(
            turretY,
            Constants.TargetTranslation.BLUE_DEADZONE_MIN,
            Constants.TargetTranslation.BLUE_DEADZONE_MAX
        );
        return new Translation2d(Constants.TargetTranslation.BLUE_SHUTTLE_X, resolvedY);
    }

    private Translation2d redShuttleTranslation() {
        double turretY = turretTranslationProvider.get().getY();
        double resolvedY = clampToShuttleLine(
            turretY,
            Constants.TargetTranslation.RED_DEADZONE_MIN,
            Constants.TargetTranslation.RED_DEADZONE_MAX
        );
        return new Translation2d(Constants.TargetTranslation.RED_SHUTTLE_X, resolvedY);
    }

    private double clampToShuttleLine(double value, double deadzoneMin, double deadzoneMax) {
        if (value < deadzoneMin || value > deadzoneMax) {
            return value;
        }
        double distToMin = value - deadzoneMin;
        double distToMax = deadzoneMax - value;
        return distToMin < distToMax ? deadzoneMin : deadzoneMax;
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance()
            .map(a -> a == Alliance.Red)
            .orElse(false);
    }
}
