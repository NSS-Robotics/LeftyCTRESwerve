package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {

    private NetworkTable table;
    private String name;

    public double ta = 0;
    public double tx = 0;
    public double ty = 0;
    public double tv = 0;

    public Pose2d botPose;

    public Limelight(String name) {
        this.name = "limelight_" + name;
        table = NetworkTableInstance.getDefault().getTable(this.name);
    }

    public void turnLimelightLED(boolean on) {
        table.getEntry("ledMode").setNumber(on ? 3 : 1);
    }
    public double getTX(){
         return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    }
    public double getTY(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
   }
   public double getRZ(){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("rz").getDouble(0);
   }

    public void updateLimelightTracking() {
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        SmartDashboard.putNumber("tx", tx);
        SmartDashboard.putNumber("ty", ty);

        double[] pos = table
            .getEntry("botpose_wpiblue")
            .getDoubleArray(new double[6]);

        if (pos.length < 6) {
            return;
        }

        double rz = pos[5];

        botPose = new Pose2d(
            pos[0],
            pos[1],
            new Rotation2d(Math.toRadians(rz))
        );

        String[] names = {
            "llpos x",
            "llpos y",
            "llpos z",
            "llrot x",
            "llrot y",
            "llrot z",
        };

        for (int i = 0; i < names.length; i++) {
            SmartDashboard.putNumber(name + ' ' + names[i], pos[i]);
        }
    }

    @Override
    public void periodic() {
        updateLimelightTracking();
    }
}
