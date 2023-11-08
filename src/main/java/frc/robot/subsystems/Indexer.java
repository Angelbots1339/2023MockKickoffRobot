// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;

import static frc.robot.Constants.IndexerConstants.*;

public class Indexer extends SubsystemBase {

  private TalonFX indexerMotor;
  private DutyCycleOut dutyCycleControl;
  private TorqueCurrentFOC torqueControl;

  /** Creates a new Indexer. */
  public Indexer() {
    indexerMotor = TalonFXFactory.createDefaultTalon(INDEXER_MOTOR_ID, Constants.CANIVORE);

    dutyCycleControl = new DutyCycleOut(0, true, false);
    torqueControl = new TorqueCurrentFOC(0, INDEXER_MAX_TORQUE_OUTPUT, INDEXER_TORQUE_DEADBAND, INDEXER_COAST_WHEN_STOPPED);


    configureIndexerMotor();
  }


    /**
   * Run intake at speed
   * 
   * @param speed range -1 to 1
   */
  public void runIndexerDutyCycle(double speed) {
    indexerMotor.setControl(dutyCycleControl.withOutput(speed));

  }

  /**
   * Run intake at torque
   * 
   * @param amps
   */
  public void runIndexerTorqueControl(double amps) {
    indexerMotor.setControl(torqueControl.withOutput(amps));

  }

  private void configureIndexerMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    indexerMotor.getConfigurator().refresh(config);

    config.TorqueCurrent.TorqueNeutralDeadband = INDEXER_TORQUE_DEADBAND;
    config.MotorOutput.Inverted = INDEXER_MOTOR_INVERT;
    config.MotorOutput.NeutralMode = INDEXER_MOTOR_NEUTRAL_MODE;

    indexerMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
