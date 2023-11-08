package frc.lib.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.reduxrobotics.sensors.canandcoder.Canandcoder;
import com.reduxrobotics.sensors.canandcoder.CanandcoderSettings;

import frc.lib.util.ErrorCheckUtil;

import static frc.robot.Constants.SwerveConstants.FalconConfigConstants.*;

public class SwerveEncoder {

    private CANcoder canCoder;
    private Canandcoder canandcoder;
    private EncoderType encoderType;

    /**
     * Used for easy switching of code between CTRE CANcoder and Redux
     * Robotics Canandcoder so the swerve code can be reused.
     * 
     * @param canID      Can ID
     * @param canBus     Either "rio" or "canivore"
     * @param isCANcoder Which type of encoder to use
     */
    public SwerveEncoder(int canID, String canBus, EncoderType encoderType) {

        this.encoderType = encoderType;

        switch (encoderType) {
            case CANCODER:
                canCoder = new CANcoder(canID, canBus);
                break;

            case CANANDCODER:
                canandcoder = new Canandcoder(canID);
                break;

            default:
                throw new RuntimeException("Invalid encoder type provided");
        }

    }

    /**
     * Used for easy switching of code between CTRE CANcoder and Redux
     * Robotics Canandcoder so the swerve code can be reused.
     * 
     * @param canID      Can ID
     * @param isCANcoder Which type of encoder to use
     */
    public SwerveEncoder(int canID, EncoderType encoderType) {

        this.encoderType = encoderType;

        switch (encoderType) {
            case CANCODER:
                canCoder = new CANcoder(canID, "rio");
                break;

            case CANANDCODER:
                canandcoder = new Canandcoder(canID);
                break;
        }

    }

    public double getAbsolutePosition() {
        switch (encoderType) {
            case CANCODER:
                return canCoder.getAbsolutePosition().getValue();

            case CANANDCODER:
                return canandcoder.getAbsPosition();
        }

        return 0;
    }

    /**
     * 
     * @param position
     */
    public void setPosition(double position) {
        switch (encoderType) {
            case CANCODER:
                canCoder.setPosition(position);

            case CANANDCODER:
                canandcoder.setAbsPosition(position);
        }
    }

    public void configure(int moduleNumber) {
        switch (encoderType) {
            case CANCODER:
                configureCANcoder(moduleNumber);
                break;
            case CANANDCODER:
                configureCanandcoder(moduleNumber);
                break;
        }
    }

    public void configureCANcoder(int moduleNumber) {
        CANcoderConfiguration config = new CANcoderConfiguration();

        config.MagnetSensor.AbsoluteSensorRange = CANCODER_ABSOLUTE_SENSOR_RANGE;
        config.MagnetSensor.SensorDirection = CANCODER_INVERT;

        ErrorCheckUtil.checkError(canCoder.getConfigurator().apply(config),
                "Failed to configure CANCoder on module " + moduleNumber);
    }

    public void configureCanandcoder(int moduleNumber) {

        CanandcoderSettings canandcoderSettings = new CanandcoderSettings();

        ErrorCheckUtil.checkBoolean(canandcoder.magnetInRange(),
                "Problem with Canandcoder magnet on module " + moduleNumber);
        // canandcoderSettings.setVelocityFilterWidth(0);

        canandcoderSettings.setInvertDirection(CANCODER_INVERT != SensorDirectionValue.CounterClockwise_Positive);

        ErrorCheckUtil.checkBoolean(canandcoder.setSettings(canandcoderSettings),
                "Failed to configure Canandcoder on module " + moduleNumber);
    }

    public enum EncoderType {
        CANCODER,
        CANANDCODER
    }
}
