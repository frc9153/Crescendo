package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class Constants {
    public class HID {
        public static final int driverJoystickPort = 0;
        public static final int operatorJoystickPort = 1;
        public static final double driverJoystickDeadband = 0.1;

        public class Binds {
            public class Driver {
                public static final int robotOrientedDriveButton = 3;
                public static final int resetSwerveHeadingButton = 8;
                public static final int magicAimButton = 12;
            }

            public class Operator {
                public static final int armIntakeButton = 1;
                public static final int armShootButton = 2;
                public static final int armStartButton = 3;
                public static final int armAmpButton = 4;
            }

        }
    }

    public class Climbing {
        public static final int leftMotorId = 12;
        public static final int rightMotorId = 13;

        public static final double climbingMaxSpeeed = 0.3;
    }
    
    public class UpDownForever {
        public static final int upDownMotorId = 11;
        public static final double upDownP = 5.0;
        public static final double upDownI = 0.0;
        public static final double upDownD = 4.0;
        public static final double upDownIZone = 0.0;
        public static final double upDownFF = 0.0;
        public static final double upDownMaxSpeed = 0.15;
        public static final double upDownPIDEpsilon = 0.05;

        public enum Setpoint {
            INTAKE(0.490),
            SHOOT(0.460),
            START(0.360),
            AMP(0.254);

            private final double targetPosition;
            private Setpoint(double targetPosition) {
                this.targetPosition = targetPosition;
            }

            public final double targetPosition() {
                return targetPosition;
            }
        };
    }

    public class Esophagus {
        public static final int esophagusId = 9;
        public static final double esophagusSpeed = 0.55;
    }

    public class Archerfish {
        public static final int archerfishId = 10;
        // "0.4 is the one that hit the ceiling light" -Cedric, hours ago
        // he did not say this..............Liar
        public static final double archerfishSpeed = 0.55;
    }

    public class Drive {
        public static final double maxSpeedMetersPerSec = 1.5; // Please do not kill anyone with this
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
            // TODO: FILLIN
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
