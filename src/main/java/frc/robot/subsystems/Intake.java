// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TalonFXFactory;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

public class Intake extends SubsystemBase {

  private Solenoid intakeSolenoid;
  private TalonFX intakeMotor;
  private DutyCycleOut dutyCycleControl;
  private TorqueCurrentFOC torqueControl;
  /** Creates a new Intake. */
  public Intake() {
    intakeSolenoid = new Solenoid(Constants.PNEUMATIC_TYPE, INTAKE_SOLENOID_ID);
    intakeMotor = TalonFXFactory.createDefaultTalon(INTAKE_MOTOR_ID, Constants.RIO);

    dutyCycleControl = new DutyCycleOut(0, true, false);
    torqueControl = new TorqueCurrentFOC(0, INTAKE_MAX_TORQUE_OUTPUT, INTAKE_TORQUE_DEADBAND, INTAKE_COAST_WHEN_STOPPED);

    configureIntakeMotor();

  }


  public void deployIntake() {
    intakeSolenoid.set(true);
  }

  public void retractIntake() {
    intakeSolenoid.set(false);
  }

  /**
   * Run intake at speed
   * 
   * @param speed range -1 to 1
   */
  public void runIntakeDutyCycle(double speed) {
    intakeMotor.setControl(dutyCycleControl.withOutput(speed));

  }


/**
 * Run intake at torque
 * @param amps
 */
  public void runIntakeTorqueControl(double amps) {
    intakeMotor.setControl(torqueControl.withOutput(amps));

  }

  /**
   * @return true if intake is deployed
   */
  public boolean isIntakeDeployed() {
    return intakeSolenoid.get(); 
  }

  public void stop() {
    intakeMotor.set(0);
  }

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    intakeMotor.getConfigurator().refresh(config);

    config.TorqueCurrent.TorqueNeutralDeadband = INTAKE_TORQUE_DEADBAND; 
    config.MotorOutput.Inverted = INTAKE_MOTOR_INVERT;
    config.MotorOutput.NeutralMode = INTAKE_MOTOR_NEUTRAL_MODE;


    intakeMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
