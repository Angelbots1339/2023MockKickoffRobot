package frc.lib.util;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import frc.lib.team254.util.LazyTalonFX;

/**
 * Creates CANTalon objects and configures all the parameters we care about to factory defaults. Closed-loop and sensor
 * parameters are not set, as these are expected to be set by the application.
 */
public class TalonFXFactory {

    private final static double kTimeoutSeconds = 0.1;
    public static int kTimeoutMs = 0;


    public static class Configuration {
        public NeutralModeValue NEUTRAL_MODE = NeutralModeValue.Coast;
        // factory defaults
        public double NEUTRAL_DEADBAND = 0.04;

        public SensorInitializationStrategy SENSOR_INITIALIZATION_STRATEGY = SensorInitializationStrategy.BootToZero;
        public double SENSOR_OFFSET_DEGREES = 0;

        public boolean ENABLE_SUPPLY_CURRENT_LIMIT = true;
        public boolean ENABLE_STATOR_CURRENT_LIMIT = true;
        public int SUPPLY_CURRENT_LIMIT = 40;
        public int STATOR_CURRENT_LIMIT = 80;
       

        /*
         * Status 1 (Default Period 10ms):
         - Applied Motor Output
         - Fault Information
         - Limit Switch Information
         */
        public int GENERAL_STATUS_FRAME_RATE_MS = 10;

        /*
         * Status 2 (Default Period 20ms):
         - Selected Sensor Position (PID 0)
         - Selected Sensor Velocity (PID 0)
         - Brushed Supply Current Measurement
         - Sticky Fault Information
         */
        public int SENSOR_FEEDBACK_RATE = 1000;
        
        /*
         * Status 3 (Default Period >100ms):
         - Quadrature Information
         */
        // public int QUAD_ENCODER_STATUS_FRAME_RATE_MS = 1000;
        
        /*
         * Status 4 (Default Period >100ms):
         - Analog Input
         - Supply Battery Voltage
         - Controller Temperature
         */
        public int ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;

        /*Status 8 (Default Period >100ms):
        - Pulse Width Information 
        */
        public int PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
        public int CONTROL_FRAME_PERIOD_MS = 10;
        

        public int MOTION_CONTROL_FRAME_PERIOD_MS = 1000;

        
        

        public SensorVelocityMeasPeriod VELOCITY_MEASUREMENT_PERIOD = SensorVelocityMeasPeriod.Period_100Ms;
        public int VELOCITY_MEASUREMENT_ROLLING_AVERAGE_WINDOW = 64;

        public double OPEN_LOOP_RAMP_RATE = 0.0;
        public double CLOSED_LOOP_RAMP_RATE = 0.0;
    }

    private static final Configuration kDefaultConfiguration = new Configuration();
    private static final Configuration kFollowerConfiguration = new Configuration();

    static {
        // This control frame value seems to need to be something reasonable to avoid the Talon's
        // LEDs behaving erratically. Potentially try to increase as much as possible.
        kFollowerConfiguration.CONTROL_FRAME_PERIOD_MS = 100;
        kFollowerConfiguration.MOTION_CONTROL_FRAME_PERIOD_MS = 1000;
        kFollowerConfiguration.GENERAL_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.ANALOG_TEMP_VBAT_STATUS_FRAME_RATE_MS = 1000;
        kFollowerConfiguration.PULSE_WIDTH_STATUS_FRAME_RATE_MS = 1000;
    }

    // create a CANTalon with the default (out of the box) configuration
    public static TalonFX createDefaultTalon(int id, String canBus) {
        return createTalon(id, canBus, kDefaultConfiguration);
    }

    public static WPI_TalonFX createDefaultSimulationTalon(int id, String canBus) {
        return createSimulationTalon(id, canBus, kDefaultConfiguration);
    }

    /**
     * 
     * @param follower_id
     * @param leader_id
     * @param canBus
     * @param invertedFromMaster Set to true for the motor to spin the opposite way of the master
     * @return TalonFX
     */
    public static TalonFX createPermanentFollowerTalon(int follower_id, int leader_id, String canBus, Boolean invertedFromMaster) {
        final TalonFX talon = createTalon(follower_id, canBus, kFollowerConfiguration);
        talon.setControl(new Follower(leader_id, invertedFromMaster));
        return talon;
    }
    public static WPI_TalonFX createPermanentSimulationFollowerTalon(int follower_id, int leader_id, String canBus) {
        final WPI_TalonFX talon = createSimulationTalon(follower_id, canBus, kFollowerConfiguration);
        talon.set(ControlMode.Follower, leader_id);
        return talon;
    }

    public static TalonFX createTalon(int id, String canBus, Configuration config) {
        TalonFX talon = new TalonFX(id, canBus); // TODO Use LazyTalonFX here once it's fixed

        TalonFXConfiguration motorConfig = new TalonFXConfiguration();


        talon.setControl(new DutyCycleOut(0)); 
        talon.clearStickyFaults(kTimeoutSeconds);


        talon.getPosition().setUpdateFrequency(id, kTimeoutSeconds);
        talon.getVelocity().setUpdateFrequency(id, kTimeoutSeconds);

        
        motorConfig.MotorOutput.NeutralMode = config.NEUTRAL_MODE;


        // Current limits
        motorConfig.CurrentLimits.StatorCurrentLimitEnable = config.ENABLE_STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.StatorCurrentLimit = config.STATOR_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimitEnable = config.ENABLE_SUPPLY_CURRENT_LIMIT;
        motorConfig.CurrentLimits.SupplyCurrentLimit = config.SUPPLY_CURRENT_LIMIT;
        

        

        talon.optimizeBusUtilization();

        return talon;
    }
    public static WPI_TalonFX createSimulationTalon(int id, String canBus, Configuration config) {
        WPI_TalonFX talon = new WPI_TalonFX(id, canBus);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.MOTION_CONTROL_FRAME_PERIOD_MS);
        talon.clearMotionProfileHasUnderrun(kTimeoutMs);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(kTimeoutMs);

        return talon;
    }
}