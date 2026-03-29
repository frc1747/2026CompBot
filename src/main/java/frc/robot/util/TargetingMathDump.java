package frc.robot.util;

public class TargetingMathDump {

    public static void main(String[] args) {
        double[] distances = {1.0, 1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5, 5.0};

        for (double distance : distances) {
            System.out.printf("=== Distance: %.1f m ===%n", distance);
            var states = TargetingMath.getAllValidStatesForDistance(distance);
            if (states.isEmpty()) {
                System.out.println("  No valid states found.");
            } else {
                System.out.printf("  %-12s %-12s%n", "Angle (deg)", "RPM");
                for (double[] state : states) {
                    System.out.printf("  %-12.1f %-12.1f%n", state[0], state[1]);
                }
            }
            System.out.println();
        }
    }
}
