// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Conversions;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor;
  private TalonFX hoodMotor;
  private VelocityVoltage shooterVelocityControl;
  private PositionVoltage hoodPositionControl;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);
    hoodMotor = new TalonFX(HOOD_MOTOR_ID);

    shooterVelocityControl = new VelocityVoltage(0, 0, SHOOTER_MOTOR_ENABLE_FOC, 0, SHOOTER_MOTOR_CONTROL_SLOT, false);
    hoodPositionControl = new PositionVoltage(0, 0, HOOD_MOTOR_ENABLE_FOC, 0, HOOD_MOTOR_CONTROL_SLOT, false);

    configureShooterMotor();
    configureHoodMotor();
  }

  /********* Setters *********/

  /**
   * Run the shooter at a target RPM
   * 
   * @param rpm Target rotations per minute (RPM)
   */
  public void runShooter(double rpm) {
    double rps = rpm / 60;
    shooterMotor.setControl(shooterVelocityControl.withVelocity(rps));
  }

  public void setHoodPosition(double position) {
    hoodMotor.setControl(hoodPositionControl);
  }

  /********* Getters *********/

  /**
   * 
   * @return Velocity in Rotations Per Second (RPS)
   */
  public double getShooterVelocity() {
    return shooterMotor.getVelocity().getValue();
  }

  /**
   * 
   * @return Velocity in Rotations Per Minute (RPM)
   */
  public double getShooterVelocityRPM() {
    return shooterMotor.getVelocity().getValue() * 60;
  }


  /**
   * 
   * @return Hood position in rotations
   */
  public double getHoodPosition() {
    return hoodMotor.getPosition().getValue();
  }

    /**
   * 
   * @return Hood position in rotations
   */
  public double getHoodPositionDegrees() {
    return Conversions.rotationsToDegrees(hoodMotor.getPosition().getValue(), 1);
  }




  private void configureShooterMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    shooterMotor.getConfigurator().refresh(config);

    config.Slot0.kP = SHOOTER_MOTOR_KP;
    config.Slot0.kI = SHOOTER_MOTOR_KI;
    config.Slot0.kD = SHOOTER_MOTOR_KD;

    config.Slot0.kV = SHOOTER_MOTOR_KV;

    config.MotorOutput.Inverted = SHOOTER_MOTOR_INVERT;
    config.MotorOutput.NeutralMode = SHOOTER_MOTOR_NEUTRAL_MODE;

    config.Feedback.RotorToSensorRatio = SHOOTER_MOTOR_GEAR_RATIO;


    shooterMotor.getConfigurator().apply(config);
  }

  private void configureHoodMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    hoodMotor.getConfigurator().refresh(config);

    config.Slot0.kP = HOOD_MOTOR_KP;
    config.Slot0.kI = HOOD_MOTOR_KI;
    config.Slot0.kD = HOOD_MOTOR_KD;

    config.Slot0.GravityType = HOOD_MOTOR_GRAVITY_TYPE;
    config.Slot0.kG = HOOD_MOTOR_KG;

    config.MotorOutput.Inverted = HOOD_MOTOR_INVERT;
    config.MotorOutput.NeutralMode = HOOD_MOTOR_NEUTRAL_MODE;

    config.Feedback.RotorToSensorRatio = HOOD_MOTOR_GEAR_RATIO;
    config.ClosedLoopGeneral.ContinuousWrap = false;


    hoodMotor.getConfigurator().apply(config);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
