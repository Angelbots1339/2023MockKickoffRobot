// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.Conversions;

import static frc.robot.Constants.ShooterConstants.*;

public class Shooter extends SubsystemBase {

  private TalonFX shooterMotor;
  private VelocityVoltage shooterVelocityControl;

  /** Creates a new Shooter. */
  public Shooter() {
    shooterMotor = new TalonFX(SHOOTER_MOTOR_ID);

    shooterVelocityControl = new VelocityVoltage(0, 0, SHOOTER_MOTOR_ENABLE_FOC, 0, SHOOTER_MOTOR_CONTROL_SLOT, false);

    configureShooterMotor();
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


  public void stop() {
    shooterMotor.set(0);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
