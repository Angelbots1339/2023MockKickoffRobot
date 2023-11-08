package frc.robot;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.lib.util.swerve.SwerveModuleConstants;


public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double angularStickDeadband = 0.1;

    public static final String CANIVORE = "canivore";
    public static final int MAX_VOLTAGE = 12;
    public static final int CAN_TIMEOUT = 100; // ms

    public static final PneumaticsModuleType PNEUMATIC_TYPE = PneumaticsModuleType.CTREPCM;


    public static final class SwerveConstants {

        public static final double LOOPER_DT = 0.02; // used for 254's solution to swerve skew it is loop time in sec
        public static final double FUDGE_FACTOR_KP = 0.1; // used for the CD fudge factor solution to swerve skew
        public static final double FUDGE_FACTOR_SIMPLE_KP = 0.1; // used for the CD fudge factor solution to swerve skew

        public static final int PIGEON_ID = 21;
        public static final boolean INVERT_GYRO = false; // Always ensure Gyro is CCW+ CW-
        public static final int GYRO_BUFFER_PERIOD = 10; // 5ms
        public static final int GYRO_BUFFER_SIZE = 9; // amount of values to store in buffer

        /* Drivetrain Constants */
        public static final double TRACK_WIDTH = Units.inchesToMeters(19.75);
        public static final double WHEEL_BASE = Units.inchesToMeters(19.75);
        public static final double WHEEL_CIRCUMFERENCE = 0.0980808154 * Math.PI;
        public static final double ALIGN_OFFSET= Units.inchesToMeters(15.75); //distance from center of robot to edge of bumper track width + bumper width



        /* Module Gear Ratios */
        /** SDS MK4i l2 - 6.75 : 1 */
        public static final double DRIVE_GEAR_RATIO = (6.75 / 1.0);
        /** SDS MK4i l1 - (150 / 7) : 1 */
        public static final double ANGLE_GEAR_RATIO = ((150.0 / 7.0) / 1.0);

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(-WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, TRACK_WIDTH / 2.0),
                new Translation2d(-WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0),
                new Translation2d(WHEEL_BASE / 2.0, -TRACK_WIDTH / 2.0));

        /*
         * Drive Motor Characterization Values
         * Divide SYSID values by 12 to convert from volts to percent output for CTRE
         * max speed:
         * 6380/8.14 * 0.1 * pi / 60 = 4.103882295
         * 12 / 4.103882295 = 2.920
         */
        public static final double DRIVE_KS = (0.23153 / 12);
        public static final double DRIVE_KV = (2.3061 / 12);
        public static final double DRIVE_KA = (0.27485 / 12);



        /* Heading deadBand */
        public static final double HEADING_DEADBAND = 0.3;

        /* Swerve Profiling Values */
        /** Meters per Second */
        private static final double TRUE_MAX_SPEED = 6380 / 60 / DRIVE_GEAR_RATIO * WHEEL_CIRCUMFERENCE; // 1.5;
        private static final double KV_MAXSPEED = 1 / DRIVE_KV;
        public static final double MAX_SPEED = KV_MAXSPEED;// 4;
        /** Radians per Second */
        private static final double TRUE_MAX_ANGULAR = MAX_SPEED / 0.7094402336;
        public static final double MAX_ANGULAR_VELOCITY = TRUE_MAX_ANGULAR; // 5.0;
        /** Meters per Second */
        public static final double MIN_CLOSE_LOOP_SPEED = 0.2;

        public static final class FalconConfigConstants {

            /* Neutral Modes */
            public static final NeutralModeValue ANGLE_NEUTRAL_MODE = NeutralModeValue.Brake;
            public static final NeutralModeValue DRIVE_NEUTRAL_MODE = NeutralModeValue.Brake;

            /* Sensor Initialization strategy */
            public static final SensorInitializationStrategy ANGLE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;
            public static final SensorInitializationStrategy DRIVE_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToZero;

            /* drive motor velocity sensor period*/
            public static final SensorVelocityMeasPeriod DRIVE_SENSOR_VELOCITY_MEAS_PERIOD = SensorVelocityMeasPeriod.Period_5Ms;
            public static final int DRIVE_SENSOR_VELOCITY_MEAS_WINDOW = 32;

            /* Angle Motor PID Values */
            public static final double ANGLE_KP = 0.3;
            public static final double ANGLE_KI = 0.0;
            public static final double ANGLE_KD = 0.0;
            public static final double ANGLE_KF = 0.0;

            /* Drive Motor PID Values */
            public static final double DRIVE_KP = 0.1; 
            public static final double DRIVE_KD = 0.0;
            public static final double DRIVE_KF = 0.0;
            /*
             * These values are used by the drive falcon to ramp in open loop and closed
             * loop driving.
             * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
             */
            public static final double OPEN_LOOP_RAMP = 0.25;
            public static final double CLOSED_LOOP_RAMP = 0.0;

            /* Motor Inverts */
            public static final InvertedValue ANGLE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
            public static final InvertedValue DRIVE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;

            /* Swerve Current Limiting */
            public static final int ANGLE_CONTINUOUS_CURRENT_LIMIT = 25;
            public static final int ANGLE_PEAK_CURRENT_LIMIT = 40;
            public static final double ANGLE_PEAK_CURRENT_DURATION = 0.1;
            public static final boolean ANGLE_ENABLE_CURRENT_LIMIT = true;
            public static final SupplyCurrentLimitConfiguration ANGLE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                    ANGLE_ENABLE_CURRENT_LIMIT, ANGLE_CONTINUOUS_CURRENT_LIMIT, ANGLE_PEAK_CURRENT_LIMIT,
                    ANGLE_PEAK_CURRENT_DURATION);

            public static final int DRIVE_CONTINUOUS_CURRENT_LIMIT = 35;
            public static final int DRIVE_PEAK_CURRENT_LIMIT = 60;
            public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
            public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;
            public static final SupplyCurrentLimitConfiguration DRIVE_CURRENT_LIMIT = new SupplyCurrentLimitConfiguration(
                    DRIVE_ENABLE_CURRENT_LIMIT, DRIVE_CONTINUOUS_CURRENT_LIMIT, DRIVE_PEAK_CURRENT_LIMIT,
                    DRIVE_PEAK_CURRENT_DURATION);

            /*------- CANcoder Config ------- */
            /* Angle Encoder Invert */
            public static final SensorDirectionValue CANCODER_INVERT = SensorDirectionValue.CounterClockwise_Positive;

            public static final AbsoluteSensorRangeValue CANCODER_ABSOLUTE_SENSOR_RANGE = AbsoluteSensorRangeValue.Unsigned_0To1;
            public static final SensorInitializationStrategy CANCODER_SENSOR_INIT_STRATEGY = SensorInitializationStrategy.BootToAbsolutePosition;
            public static final SensorTimeBase CANCODER_SENSOR_TIME_BASE = SensorTimeBase.PerSecond;
        }

        /* Module Specific Constants */
        /** Back Left Module - Module 0 */
        public static final class Mod0 {
            public static final int DRIVE_MOTOR_ID = 4;
            public static final int CANCODER_ID = 5;
            public static final int ANGLE_MOTOR_ID = 6;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(295.751953125);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        /** Front Left Module - Module 1 */
        public static final class Mod1 {
            public static final int DRIVE_MOTOR_ID = 1;
            public static final int CANCODER_ID = 2;
            public static final int ANGLE_MOTOR_ID = 3;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(222.97851562500003);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }

        
        /** Back Right Module - Module 2 */
        public static final class Mod2 {
            public static final int DRIVE_MOTOR_ID = 9;
            public static final int CANCODER_ID = 10;
            public static final int ANGLE_MOTOR_ID = 11;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(228.2519);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
            ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        
        /** front Right Module - Module 3 */
        public static final class Mod3 {
            public static final int DRIVE_MOTOR_ID = 12;
            public static final int CANCODER_ID = 13;
            public static final int ANGLE_MOTOR_ID = 14;
            public static final Rotation2d ANGLE_OFFSET = Rotation2d.fromDegrees(195.820);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(DRIVE_MOTOR_ID,
                    ANGLE_MOTOR_ID, CANCODER_ID, ANGLE_OFFSET);
        }
        public static final class DrivePidConstants {

            public static final double ANGLE_KP = 0.025;  //0.05// radians per sec per degrees
            public static final double ANGLE_KD = 0.0;
            public static final double ANGLE_KV = 0.0;

            public static final double TRANSLATION_KP = 0.5; // meters per sec per meter
            public static final double TRANSLATION_KD = 0;
            public static final double TRANSLATION_KS = 0.21;

            // Feedfowards
            public static final double ANGLE_KS = 0.7; // radians per sec
            public static final double ANGLE_TOLERANCE = 2; // Degrees
        
            public static final double TRANSLATION_PID_TOLERANCE = 0.02;

        }
    }


    public static final class IntakeConstants {

        public static final int INTAKE_MOTOR_ID = 15;
        public static final int INTAKE_SOLENOID_ID = 0;
        

        public static final InvertedValue INTAKE_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue INTAKE_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;


        public static final double INTAKE_DUTY_CYCLE_SPEED = 0.5; // Percentage (0 to 1)
        public static final double INTAKE_TORQUE_SPEED = 5; // Amps

        public static final double INTAKE_MAX_TORQUE_OUTPUT = 0.5; // Percentage (0 to 1)
        public static final double INTAKE_TORQUE_DEADBAND = 1; // Amps
        public static final boolean INTAKE_COAST_WHEN_STOPPED = true; // Amps



    }

    public static final class ShooterConstants {


        // Shooter Motor

        public static final int SHOOTER_MOTOR_ID = 16;

        public static final double SHOOTER_MOTOR_KP = 0;
        public static final double SHOOTER_MOTOR_KI = 0;
        public static final double SHOOTER_MOTOR_KD = 0;

        public static final double SHOOTER_MOTOR_KV = 0;

        public static final InvertedValue SHOOTER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue SHOOTER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        public static final boolean SHOOTER_MOTOR_ENABLE_FOC = true;
        public static final int SHOOTER_MOTOR_CONTROL_SLOT = 0;

        public static final double SHOOTER_MOTOR_GEAR_RATIO = 1/1;



        // Hood Motor

        public static final int HOOD_MOTOR_ID = 17;


        public static final double HOOD_MOTOR_KP = 0;
        public static final double HOOD_MOTOR_KI = 0;
        public static final double HOOD_MOTOR_KD = 0;

        public static final double HOOD_MOTOR_KG = 0;
        public static final GravityTypeValue HOOD_MOTOR_GRAVITY_TYPE = GravityTypeValue.Arm_Cosine;
        
        public static final InvertedValue HOOD_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue HOOD_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        public static final boolean HOOD_MOTOR_ENABLE_FOC = true;
        public static final int HOOD_MOTOR_CONTROL_SLOT = 0;


        public static final double HOOD_MOTOR_GEAR_RATIO = 1/1;

    }

    public static final class IndexerConstants {

        public static final int INDEXER_MOTOR_ID = 17;

        public static final InvertedValue INDEXER_MOTOR_INVERT = InvertedValue.Clockwise_Positive;
        public static final NeutralModeValue INDEXER_MOTOR_NEUTRAL_MODE = NeutralModeValue.Coast;

        public static final double INDEXER_TORQUE_SPEED = 5; // Amps

        public static final double INDEXER_MAX_TORQUE_OUTPUT = 0.5; // Percentage (0 to 1)
        public static final double INDEXER_TORQUE_DEADBAND = 1; // Amps
        public static final boolean INDEXER_COAST_WHEN_STOPPED = true; // Amps

    }

    public static final class VisionConstants {

        public static final String LIMELIGHT_NAME = "limelight1";

        public static final Transform3d APRILTAG_CAM_POS = new Transform3d(new Translation3d(0.24345, -0.25397, 0.56859),
                new Rotation3d(0, -0, 0)); 

                        /*
         * Standard deviations of model states. Increase these numbers to trust your
         * model's state estimates less. This matrix is in the form [x, y, theta]ᵀ, with
         * units in
         * meters and radians.
         */
        public static final Matrix<N3, N1> STATE_STD_DEVS = new MatBuilder<>(Nat.N3(), Nat.N1()).fill(
            0.10,
            0.10,
            0.10);

        public static final Transform3d LIMELIGHT_CAM_POS = new Transform3d(new Translation3d(-0.04, -0.25, 0.85),
                new Rotation3d(0, Math.toRadians(0), 0));

        public static final double LIMELIGHT_ALIGN_MAX_SPEED = 2; //m/s

    }

    public static final class AutoConstants {
        public static final double X_KP = 1.5;
        public static final double Y_KP = 1.5;
        public static final double THETA_KP = 3.2;
    }


    public final static class FieldConstants {
        public static final Translation2d RED_ORIGIN = new Translation2d(16.540988, 8.02);
    }

}
