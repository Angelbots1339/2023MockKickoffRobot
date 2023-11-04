package frc.lib.team254.util;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFX;

/**
 * This class is a thin wrapper around the CANTalon that reduces CAN bus / CPU overhead by skipping duplicate set
 * commands. (By default the Talon flushes the Tx buffer on every set call).
 */
public class LazyTalonFX extends TalonFX {
    protected double mLastSet = Double.NaN;
    protected ControlRequest mLastControlMode = null;

    public LazyTalonFX(int deviceNumber) {
        super(deviceNumber);
    }
    public LazyTalonFX(int deviceNumber, String canbus) {
        super(deviceNumber, canbus);
    }

    public double getLastSet() {
        return mLastSet;
    }

    //TODO Fix LazyTalonFX to use Phoenix6
    // @Override
    // public void set(ControlRequest mode, double value) {
    //     if (value != mLastSet || mode != mLastControlMode) {
    //         mLastSet = value;
    //         mLastControlMode = mode;
    //         super.set(mode, value);
    //     }
    // }
}