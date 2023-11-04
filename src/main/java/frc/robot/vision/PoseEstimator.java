// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.logging.loggedObjects.LoggedField;
import frc.lib.util.poseEstimation.LimelightHelpers;

import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Constants.SwerveConstants;
import frc.robot.regressions.KalmanVisionRegression;
import frc.robot.subsystems.Swerve;;

/** Add your docs here. */
public class PoseEstimator {

    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator poseEstimatorNonVision;
    private AprilTagFieldLayout layout;

    public PoseEstimator(LoggedField logger, Rotation2d gyroAngle, SwerveModulePosition[] positions) {

        // Pose Estimator
        // try {
        //     layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        // } catch (IOException e) {
        //     e.printStackTrace();
        // }
        
        poseEstimatorNonVision = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions,
                new Pose2d(), STATE_STD_DEVS, VecBuilder.fill(0, 0, 0));

        poseEstimator = new SwerveDrivePoseEstimator(SwerveConstants.KINEMATICS, gyroAngle, positions, new Pose2d(),
                STATE_STD_DEVS, VecBuilder.fill(0, 0, 0));



        logger.addPose2d("Robot", () -> poseEstimator.getEstimatedPosition(), true);
        logger.addPose2d("NonVisionPoseEstimation", () -> poseEstimatorNonVision.getEstimatedPosition(), false);
    }

    public void updateOdometry(Swerve swerve) {
        poseEstimator.update(swerve.getYaw(), swerve.getPositions());
        poseEstimatorNonVision.update(swerve.getYaw(), swerve.getPositions());
    }

    /**
     * 
     * @param swerve Swerve subsystem instance
     * @param referencePose Pose2d of the robot at the time of the vision measurement
     */
    public void updateVisionMeasurement(Swerve swerve, Pose2d referencePose) {
    

            // if (robotToTarget == null)
            //     return;

            
            double tagDistance = LimelightHelpers.getTargetPose3d_CameraSpace(LIMELIGHT_NAME).getTranslation().getNorm(); // Find direct distance to target for std dev calculation
            double xyStdDev2 = MathUtil.clamp(0.002 * Math.pow(2.2, tagDistance), 0, 1);


            Pose2d poseFromVision =  LimelightHelpers.getBotPose2d(LIMELIGHT_NAME);
            double poseFromVisionTimestamp = Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Capture(LIMELIGHT_NAME) - LimelightHelpers.getLatency_Pipeline("limelight1");

            poseEstimator.addVisionMeasurement(poseFromVision, poseFromVisionTimestamp, VecBuilder.fill(xyStdDev2, xyStdDev2, 0));

        }
    

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Pose2d getPoseNonVision() {
        return poseEstimatorNonVision.getEstimatedPosition();
    }

    // public Optional<Pair<EstimatedRobotPose, Translation2d>> getPoseOnlyVision(Pose2d referencePose) {
    //     PhotonPipelineResult result = APRILTAG_CAM.getLatestResult();
    //     if (result.hasTargets()) {
    //         EstimatedRobotPose lowestDeltaPose = null;
    //         double smallestPoseDelta = 10e9;
    //         Translation2d robotToTarget = null;

    //         for (PhotonTrackedTarget target : result.getTargets()) {
    //             int id = target.getFiducialId();
    //             if (id > 8 || id < 1)
    //                 continue;

    //             Pose3d targetPostiton = layout.getTagPose(target.getFiducialId()).get();

    //             Transform3d transformToTarget = target.getBestCameraToTarget();

    //             Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(transformToTarget, targetPostiton,
    //                     APRILTAG_CAM_POS);

    //             double poseDelta = referencePose.getTranslation()
    //                     .getDistance(estimatedPose.getTranslation().toTranslation2d());
    //             if (poseDelta < smallestPoseDelta) {
    //                 smallestPoseDelta = poseDelta;
    //                 lowestDeltaPose = new EstimatedRobotPose(estimatedPose, result.getTimestampSeconds(),
    //                         List.of(target));
    //                 robotToTarget = transformToTarget.getTranslation().toTranslation2d();
    //             }
    //         }
    //         return Optional.of(Pair.of(lowestDeltaPose, robotToTarget));
    //     }
    //     return Optional.empty();
    // }

    public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] swerveModulePositions, Pose2d pose) {
        poseEstimator.resetPosition(gyroAngle, swerveModulePositions, pose);
        poseEstimatorNonVision.resetPosition(gyroAngle, swerveModulePositions, pose);
    }

    public AprilTagFieldLayout getField() {
        return layout;
    }

}
