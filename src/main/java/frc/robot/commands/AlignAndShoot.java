// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class AlignAndShoot extends Command {

  private Shooter shooter;
  private Intake intake;
  private Indexer indexer;

  /** Creates a new AlignAndShoot. */
  public AlignAndShoot(Shooter shooter, Intake intake, Indexer indexer) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.shooter = shooter;
    this.intake = intake;
    this.indexer = indexer;

    addRequirements(shooter, intake, indexer);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
    indexer.stop();
    shooter.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
