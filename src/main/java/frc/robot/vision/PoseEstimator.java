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
import static frc.robot.Constants.VisionConstants.*;
import frc.robot.Constants.SwerveConstants;
import frc.robot.regressions.KalmanVisionRegression;
import frc.robot.subsystems.Swerve;;

/** Add your docs here. */
public class PoseEstimator {

    private SwerveDrivePoseEstimator poseEstimator;
    public SwerveDrivePoseEstimator poseEstimatorNonVision;
    private AprilTagFieldLayout layout;

    private final Pose2d[] cubeNodePoses = new Pose2d[6];

    public PoseEstimationState state = PoseEstimationState.ALL;

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

    public void justUpdate(Swerve swerve) {
        poseEstimator.update(swerve.getYaw(), swerve.getPositions());
        poseEstimatorNonVision.update(swerve.getYaw(), swerve.getPositions());
    }

    private static GenericEntry poseFinderOuttakePercent = Shuffleboard.getTab("vis").add("std", 0)
            .withWidget(BuiltInWidgets.kNumberSlider)
            .withProperties(Map.of("min", 0, "max", 2, "Block increment", 0.01)).getEntry();

    // public void updateOdometry(Swerve swerve, Pose2d referencePose) {
    //     PhotonPipelineResult result = APRILTAG_CAM.getLatestResult();
    //     SmartDashboard.putString("Vision State", state.toString());
    //     if (result.hasTargets()) {
    //         double smallestPoseDelta = 10e9;
    //         EstimatedRobotPose lowestDeltaPose = null;
    //         Translation2d robotToTarget = null;

    //         for (PhotonTrackedTarget target : result.getTargets()) {
    //             int id = target.getFiducialId();
    //             if (id > 8 || id < 1)
    //                 continue;


    //             switch (state) {
    //                 case CABLE:
    //                     if (id != 8 && id != 1)
    //                         continue;
    //                         break;
    //                 case NON:
    //                     if (id != 6 && id != 3)
    //                         continue;
    //                         break;
    //             }

    //             Pose3d targetPosition = layout.getTagPose(id).get();

    //             // TODO check sign of pitch and maybe add pitch from gyro
    //             Rotation3d gyroCalculatedAngle;
    //             double yaw = swerve.getYawLatency(result.getLatencyMillis() + 15).getRadians();

    //             if (id > 4)
    //                 gyroCalculatedAngle = new Rotation3d(0,
    //                         -APRILTAG_CAM_POS.getRotation().getY()
    //                                 + swerve.getGyroAtTime(result.getTimestampSeconds()).getY(),
    //                         -yaw);

    //             else
    //                 gyroCalculatedAngle = new Rotation3d(0,
    //                         -APRILTAG_CAM_POS.getRotation().getY()
    //                                 + swerve.getGyroAtTime(result.getLatencyMillis()).getY(),
    //                         (yaw > 0 ? 1 : -1) * Math.PI - yaw);

    //             Translation3d transformToTarget = target.getBestCameraToTarget().getTranslation();

    //             Transform2d lastTransformToTarget = referencePose.minus(targetPosition.toPose2d());

    //             SmartDashboard.putNumber("GyroCalcangle", Math.toDegrees(gyroCalculatedAngle.getZ()));
    //             SmartDashboard.putNumber("target angle",
    //                     Math.toDegrees(target.getBestCameraToTarget().getRotation().getZ()));

    //             // Pose3d estimatedPose = PhotonUtils.estimateFieldToRobotAprilTag(
    //             // new Transform3d(transformToTarget, gyroCalculatedAngle), targetPosition,
    //             // APRILTAG_CAM_POS);
    //             Pose3d estimatedPose = targetPosition
    //                     .transformBy(new Transform3d(transformToTarget, gyroCalculatedAngle).inverse())
    //                     .transformBy(APRILTAG_CAM_POS.inverse());

    //             // SmartDashboard.putNumber("estimatedPose", estimatedPose.getX());

    //             double poseDelta = referencePose.getTranslation()
    //                     .getDistance(estimatedPose.getTranslation().toTranslation2d());
    //             if (poseDelta < smallestPoseDelta) {
    //                 smallestPoseDelta = poseDelta;
    //                 lowestDeltaPose = new EstimatedRobotPose(estimatedPose, result.getTimestampSeconds(),
    //                         List.of(target));
    //                 robotToTarget = lastTransformToTarget.getTranslation();
    //             }
    //             // SmartDashboard.putNumber("xVision", transformToTarget.getX() +
    //             // APRILTAG_CAM_POS.getX());
    //             // SmartDashboard.putNumber("yVision", transformToTarget.getY());

    //         }

    //         if (robotToTarget == null)
    //             return;

    //         double tagDistance = Math.abs(robotToTarget.getNorm());
    //         double xyStdDev2 = MathUtil.clamp(0.002 * Math.pow(2.2, tagDistance), 0, 1);
    //         double xyStdDev = poseFinderOuttakePercent.getDouble(0);

    //         // SmartDashboard.putNumber("xyStdDev", xyStdDev2);
    //         // SmartDashboard.putNumber("Calculated", xyStdDev2);
    //         // SmartDashboard.putNumber("Dist", tagDistance);

    //         poseEstimator.addVisionMeasurement(lowestDeltaPose.estimatedPose.toPose2d(),
    //                 lowestDeltaPose.timestampSeconds, VecBuilder.fill(xyStdDev2, xyStdDev2, xyStdDev));

    //     }
    // }

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

    public void alignPoseNonVisionEstimator(SwerveModulePosition[] swerveModulePositions) {
        poseEstimatorNonVision.resetPosition(getPose().getRotation(), swerveModulePositions, getPose());
    }

    public AprilTagFieldLayout getField() {
        return layout;
    }

    public enum PoseEstimationState {
        ALL,
        CABLE,
        NON;
    }
}
