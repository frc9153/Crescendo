package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class HID {
        public static final int driverJoystickPort = 0;
        public static final int operatorJoystickPort = 1;
        public static final int childJoystickPort = 2; // Child-safe joystick for Peach Festival
        public static final double driverJoystickDeadband = 0.1;

        public class Binds {
            public class Driver {
                public static final int robotOrientedDriveButton = 2;
                public static final int resetSwerveHeadingButton = 8;
                public static final int magicAimButton = 12;
            }

            public class Operator {
                public static final int armIntakeButton = 1;
                public static final int armShootButton = 2;
                public static final int armStartButton = 3;
                public static final int armAmpButton = 4;
                public static final int intakeReverseButton = 5;
                public static final int shooterSlowButton = 6;
                
                public static final int manualArmTriggerButton = 8; // Should be Start, double check
            }

            public class Child {
                public static final int shootButton = 1;
                public static final int intakeButton = 2;
            }

        }
    }

    public class Climbing {
        public static final int leftMotorId = 12;
        public static final int rightMotorId = 13;

        public static final double climbingMaxSpeeed = 0.75;
    }
    
    public class UpDownForever {
        public static final int upDownMotorIdOne = 11;
        public static final int upDownMotorIdTwo = 15;
        public static final double upDownP = 8.0; // 4.0
        public static final double upDownI = 0.0; // 0.0
        public static final double upDownD = 4.0; // 5.0
        public static final double upDownIZone = 0.0;
        public static final double upDownFF = 0.0;
        public static final double upDownMaxSpeed = 0.50;// 0.35 tested up to 0.50
        public static final double upDownPIDEpsilon = 0.05;

        public static final double manualSpeed = 0.13;

        // Increase -> Down
        public static final double setpointOffset = 0.0;// 
        public enum Setpoint {
            INTAKE(0.472+setpointOffset),
            SHOOT(0.445+setpointOffset),
            START(0.280+setpointOffset),
            AMP(0.232+setpointOffset);

            private final double targetPosition;
            private Setpoint(double targetPosition) {
                this.targetPosition = targetPosition;
            }

            public final double targetPosition() {
                return targetPosition;
            }
        };
        public static final double manualLowerLimit = 0.472+setpointOffset;
        public static final double manualUpperLimit = 0.232+setpointOffset;
    }

    public class Esophagus {
        public static final int esophagusId = 9;
        public static final double esophagusSpeed = 0.37;// 0.55
        public static final double reverseSpeed = -0.18;
    }

    public class Archerfish {
        public static final int archerfishId = 10;
        // "0.4 is the one that hit the ceiling light" -Cedric, hours ago
        // he did not say this..............Liar
        
        public static final double archerfishP = 0.00002; // 0.6
        public static final double archerfishI = 0.00000000015; // 0.0
        public static final double archerfishD = 0.0;
        public static final double archerfishIZone = 0.0;
        public static final double archerfishFF = 0.00000481; // 0.15
        public static final double archerfishMaxSpeed = 0.80;
        public static final double archerfishPIDEpsilon = 50; // rpm, so very high

        // Percent Motor Power (old)
        public static final double archerfishSpeed = 0.7;//0.7
        public static final double archerfishSpeedChild = 0.5;
        public static final double archerfishSpeedSlow = 0.3;//0.3
        
        // Velocity in rpm (new)
        public static final double archerfishVelocity = 2000; // Dummy values, probably something more like 2000
        public static final double archerfishVelocityChild = 100;
        public static final double archerfishVelocitySlow = 50;
    }

    public class LaserCannon {
        public static final int laserCannonId = 14;

        public static final double noteDistThreshold = 200.0;
    }

    public class Autonomous {
        public static final double autoSpeed = 0.3363;
        
        public static final double inchToSpeedless = 0.0042;

        public class Speaker_Front {
            public static final double SpeakerToSpikeMark = 80*inchToSpeedless; // Speedless measure
            public static final double SpikeMarkToSpikeMark = 55*inchToSpeedless;
        }
    }

    public class Drive {//4.8
        public static final double maxSpeedMetersPerSec = 4.46; // Please do not kill anyone with this
        public static final double maxRotSpeedRadsPerSec = Math.PI;

        public class Chassis {
            // Distance between centers of right and left wheels
            public static final double trackWidth = Units.inchesToMeters(21.5);
            // Distance between front and back wheels
            public static final double wheelBase = Units.inchesToMeters(26.5);

            public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
                    new Translation2d(wheelBase / 2, trackWidth / 2),
                    new Translation2d(wheelBase / 2, -trackWidth / 2),
                    new Translation2d(-wheelBase / 2, trackWidth / 2),
                    new Translation2d(-wheelBase / 2, -trackWidth / 2));

            // Angular offsets of the modules relative to the chassis in radians
            public static final double frontLeftChassisAngularOffset = -Math.PI / 2;
            public static final double frontRightChassisAngularOffset = 0;
            public static final double backLeftChassisAngularOffset = Math.PI;
            public static final double backRightChassisAngularOffset = Math.PI / 2;
        }

        public class CAN {
            public static final int frontLeftThrustId = 7;
            public static final int frontRightThrustId = 1;
            public static final int backLeftThrustId = 5;
            public static final int backRightThrustId = 3;

            public static final int frontLeftSteerId = 8;
            public static final int frontRightSteerId = 2;
            public static final int backLeftSteerId = 6;
            public static final int backRightSteerId = 4;
        }

        public class SwerveModule {
            // The MAXSwerve module can be configured with one of three pinion gears: 12T,
            // 13T, or 14T.
            // This changes the drive speed of the module (a pinion gear with more teeth
            // will result in a
            // robot that drives faster).
            public static final int drivingMotorPinionTeeth = 14;

            // Invert the turning encoder, since the output shaft rotates in the opposite
            // direction of
            // the steering motor in the MAXSwerve Module.
            public static final boolean turningEncoderInverted = true;

            // Calculations required for driving motor conversion factors and feed forward
            public static final double freeSpeedRpm = 5676;
            public static final double drivingMotorFreeSpeedRps = freeSpeedRpm / 60;
            public static final double wheelDiameterMeters = Units.inchesToMeters(3.0); // Should equal 0.0762. If it doesn't, scream
            public static final double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
            // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
            // teeth on the bevel pinion
            public static final double drivingMotorReduction = (45.0 * 22) / (drivingMotorPinionTeeth * 15);
            public static final double driveWheelFreeSpeedRps = (drivingMotorFreeSpeedRps * wheelCircumferenceMeters)
                    / drivingMotorReduction;

            public static final double drivingEncoderPositionFactor = (wheelDiameterMeters * Math.PI)
                    / drivingMotorReduction; // meters
            public static final double drivingEncoderVelocityFactor = ((wheelDiameterMeters * Math.PI)
                    / drivingMotorReduction) / 60.0; // meters per second

            public static final double turningEncoderPositionFactor = (2 * Math.PI); // radians
            public static final double turningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

            public static final double turningEncoderPositionPIDMinInput = 0; // radians
            public static final double turningEncoderPositionPIDMaxInput = turningEncoderPositionFactor; // radians

            public static final double drivingP = 0.04;
            public static final double drivingI = 0;
            public static final double drivingD = 0;
            public static final double drivingFF = 1 / driveWheelFreeSpeedRps;
            public static final double drivingMinOutput = -1;
            public static final double drivingMaxOutput = 1;

            public static final double turningP = 1;
            public static final double turningI = 0;
            public static final double turningD = 0;
            public static final double turningFF = 0;
            public static final double turningMinOutput = -1;
            public static final double turningMaxOutput = 1;

            public static final IdleMode drivingMotorIdleMode = IdleMode.kCoast;
            public static final IdleMode turningMotorIdleMode = IdleMode.kCoast;

            
            public static final int drivingMotorCurrentLimit = 50; // amps
            public static final int turningMotorCurrentLimit = 20; // amps
        }
    }
}
