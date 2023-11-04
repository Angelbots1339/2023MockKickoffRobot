package frc.lib.util;

import com.ctre.phoenix6.hardware.TalonFX;

public class TalonFXUtil {
    

    public static void updateAllTalonFXStatusSignals(TalonFX talon) {

        talon.getPosition().refresh();

    }
}
