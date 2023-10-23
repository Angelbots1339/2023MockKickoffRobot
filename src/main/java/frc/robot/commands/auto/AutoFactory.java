// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.lang.reflect.Field;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.subsystems.Swerve;


/** Add your docs here. */
public class AutoFactory {

        public static Command Score2BalancePos6(/*Put subsystem dependencies here*/) {
                return null;
        }

        public static Command resetGyroAndPos(Swerve swerve, PathPlannerTrajectory trajectory) {
                return new InstantCommand(swerve::resetGyroTowardsDriverStation)
                                .andThen(() -> SwerveFollowTrajectory.resetPos(trajectory, swerve));
        }

}
