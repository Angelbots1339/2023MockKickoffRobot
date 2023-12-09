// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.util.Conversions;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.commands.AlignAndShoot;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Swerve;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  private final XboxController driver = new XboxController(0);
  // private final XboxController operator = new XboxController(1);

  private final Swerve swerve = new Swerve();
  private final Shooter shooter = new Shooter();
  private final Intake intake = new Intake();
  private final Indexer indexer = new Indexer();

  /****** Driver Controls ******/

  // Maps to rect than applys deadband
  private Supplier<Double> translationX = () -> MathUtil.applyDeadband(Conversions.mapJoystick(driver.getLeftY(),
      driver.getLeftX()), Constants.stickDeadband);
  private Supplier<Double> strafeY = () -> MathUtil.applyDeadband(Conversions.mapJoystick(driver.getLeftX(),
      driver.getLeftY()), Constants.stickDeadband);
  private Supplier<Double> rotation = () -> MathUtil.applyDeadband(-driver.getRightX(), Constants.stickDeadband);

  private Rotation2d previousAngle = swerve.getYaw();
  private Supplier<Rotation2d> angle = () -> {
    if (Math.abs(driver.getRightX()) > Constants.angularStickDeadband ||
        Math.abs(driver.getRightY()) > Constants.angularStickDeadband) {
      previousAngle = Conversions.ConvertJoystickToAngle(driver.getRightX(),
          driver.getRightY());
    }
    return previousAngle;
  };

  private final Trigger zeroGyro = new JoystickButton(driver,
      XboxController.Button.kStart.value);

  private final Trigger runIntake = new JoystickButton(driver,
      XboxController.Button.kLeftBumper.value);



  /****** Operator Controls ******/
  // private final Trigger toggleIntakeDeploy = new JoystickButton(operator,
  //     XboxController.Button.kA.value);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    setDefaultCommands();

    // Configure the trigger bindings
    configureBindings();



    // Configure PathPlanner's AutoBuilder
    AutoFactory.config(swerve);
    autoChooser = AutoBuilder.buildAutoChooser();
    // System.out.println("Auto Chooser: " + autoChooser.toString());

    
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void setDefaultCommands() {
    swerve.setDefaultCommand(
        new TeleopSwerve(swerve, translationX, strafeY, rotation, angle, () -> false,
            true // Is field relative
        ));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    configureOperatorBindings();
    configureDriverBindings();
  }

  private void configureDriverBindings() {
    zeroGyro.onTrue(new InstantCommand(swerve::resetGyroTowardsDriverStation));

    runIntake.whileTrue(
        new StartEndCommand(() -> intake.runIntakeTorqueControl(Constants.IntakeConstants.INTAKE_TORQUE_SPEED),
            () -> intake.stop(), intake));

  }

  private void configureOperatorBindings() {

  }


  public void resetToAbsloute() {
    swerve.resetToAbsolute();
}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

}
