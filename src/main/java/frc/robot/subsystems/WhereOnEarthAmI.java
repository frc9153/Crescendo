package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Vector2;

public class WhereOnEarthAmI extends SubsystemBase {
    // private final AHRS gyro = new AHRS(SPI.Port.kMXP);
    private final NetworkTableEntry limelightPose = NetworkTableInstance.getDefault().getTable("limelight")
            .getEntry("botpose");
    private long lastPoseReset = 0;
    private Vector2 pose = Vector2.Zero();
    private Vector2 gyroHomedPose = Vector2.Zero();

    public WhereOnEarthAmI() {
    }

    private void tryUpdatePose() {
        long poseLastChange = limelightPose.getLastChange();
        if (poseLastChange == lastPoseReset)
            return;
        lastPoseReset = poseLastChange;
        Double poseData[] = limelightPose.getDoubleArray(new Double[] { 0.0, 0.0 });

        // TODO: Validate axes
        pose.x = poseData[0];
        pose.y = poseData[1];

        // TODO: Actually home value
        gyroHomedPose = pose.graft();
    }

    private void updateWithKinetic() {
        Vector2 gyroPose = new Vector2(
                0, 0
        // gyro.getDisplacementX(),
        // gyro.getDisplacementY()
        );
        pose = pose.plus(gyroPose.minus(gyroHomedPose));
    }

    @Override
    public void periodic() {
        tryUpdatePose();
        updateWithKinetic();
    }
}
