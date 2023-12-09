// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.lang.reflect.Field;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.AlignAndShoot;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AutoFactory {

        // public static Command Test2m(Swerve swerve) {
        //         return new PathPlannerAuto("2mTest");
        // }

        // public static Command resetGyroAndPos(Swerve swerve, PathPlannerTrajectory
        // trajectory) {
        // return new InstantCommand(swerve::resetGyroTowardsDriverStation)
        // .andThen(() -> SwerveFollowTrajectory.resetPos(trajectory, swerve));
        // }



        

        public static void config(Swerve swerve) {

                NamedCommands.registerCommand("test", new InstantCommand(() -> {}));

                AutoBuilder.configureHolonomic(
                                swerve::getPose, // Robot pose supplier
                                swerve::resetOdometry, // Method to reset odometry (will be called if your auto has a
                                                       // starting pose)
                                swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                                swerve::setChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE
                                                          // ChassisSpeeds
                                AutoConstants.HOLONOMIC_CONFIG,
                                swerve // Reference to this subsystem to set requirements
                );
        }

}
