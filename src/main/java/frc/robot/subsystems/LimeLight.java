package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight extends SubsystemBase {

    String name;
    NetworkTable table;
    NetworkTableEntry xOffsetEntry;
    NetworkTableEntry yOffsetEntry;
    NetworkTableEntry areaEntry;
    NetworkTableEntry poseAmbiguityEntry;
    NetworkTableEntry validTargetEntry;
    public LimeLight(String name) {
        this.name = name;
        table = NetworkTableInstance.getDefault().getTable(name);
        xOffsetEntry = table.getEntry("tx");
        yOffsetEntry = table.getEntry("ty");
        areaEntry = table.getEntry("ta");
        poseAmbiguityEntry = table.getEntry("pa");
        validTargetEntry = table.getEntry("tv");
    }

    public String getName() {
        return name;
    }

    public double getXOffset() {
        return xOffsetEntry.getDouble(361.0);
    }

    public double getYOffset() {
        return yOffsetEntry.getDouble(69.0);
    }

    public double getAreaOffset() {
        return areaEntry.getDouble(101.0);
    }

    public double getPoseAmbiguity() {
        return poseAmbiguityEntry.getDouble(-1);
    }

    public boolean hasValidTarget() {
        return validTargetEntry.getDouble(0.0) == 1;
    }

    // what is this and why is it here
    public void robotInit() {
      for (int port = 5800; port <= 5809; port ++) {
        PortForwarder.add(port+10, "limelight.local", port);
      }
    }

    public LimelightHelpers.PoseEstimate getMt2Pose() {
        var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
        return mt2;
    }
    
    @Override
    public void periodic() {
        boolean targetValidity = hasValidTarget();
        SmartDashboard.putBoolean("Valid Target", targetValidity);
    }    
}