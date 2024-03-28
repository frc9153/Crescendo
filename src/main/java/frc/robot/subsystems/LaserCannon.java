package frc.robot.subsystems;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;

import frc.robot.Constants;

public class LaserCannon {

    private LaserCan m_laserCan;

    public LaserCannon() {
        m_laserCan = new LaserCan(Constants.LaserCannon.laserCannonId);

        // Optionally initialise the settings of the LaserCAN, if you haven't already done so in GrappleHook
        try {
            m_laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            m_laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(8, 8, 16, 16));
            m_laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("Configuration failed! " + e);
        }
    }
    
    public int getDistance() {
        LaserCan.Measurement measurement = m_laserCan.getMeasurement();
        if (measurement == null || measurement.status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("Sensor Data Invalid");
        }
        return measurement.distance_mm;
    }

    public boolean sensorTriggered() {
        if (this.getDistance() < Constants.LaserCannon.noteDistThreshold) {
            return true;
        }
        return false;
    }
}
