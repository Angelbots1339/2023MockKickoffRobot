package frc.lib.util.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import static frc.robot.Constants.SwerveConstants.*;

public class SwerveMath {

    /**
     * Minimize the change in heading the desired swerve module state would require
     * by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to
     * include placing
     * in appropriate scope for CTRE onboard control.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle = placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle       Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) {
            newAngle += 360;
        }
        while (newAngle > upperBound) {
            newAngle -= 360;
        }
        if (newAngle - scopeReference > 180) {
            newAngle -= 360;
        } else if (newAngle - scopeReference < -180) {
            newAngle += 360;
        }
        return newAngle;
    }


    /*-----Possible Solutions for Swerve Drive Skew------ */

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1}
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeeds254(ChassisSpeeds chassisSpeeds) {
        frc.lib.team254.geometry.Pose2d robot_pose_vel = new frc.lib.team254.geometry.Pose2d(
                chassisSpeeds.vxMetersPerSecond * LOOPER_DT,
                chassisSpeeds.vyMetersPerSecond * LOOPER_DT,
                frc.lib.team254.geometry.Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * LOOPER_DT));
        frc.lib.team254.geometry.Twist2d twist_vel = frc.lib.team254.geometry.Pose2d.log(robot_pose_vel);
        return new ChassisSpeeds(
                twist_vel.dx / LOOPER_DT, twist_vel.dy / LOOPER_DT, twist_vel.dtheta / LOOPER_DT);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */

    private static double previousDriveTime;

    public static ChassisSpeeds reduceSkewFromLogTwist2d(ChassisSpeeds chassisSpeeds) {
        double timerDt = (Timer.getFPGATimestamp() - previousDriveTime);
        Pose2d robot_pose_vel = new Pose2d(chassisSpeeds.vxMetersPerSecond * timerDt,
                chassisSpeeds.vyMetersPerSecond * timerDt,
                Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * timerDt));
        Twist2d twist_vel = new Pose2d(0, 0, new Rotation2d(0)).log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / timerDt, twist_vel.dy / timerDt, twist_vel.dtheta / timerDt);
        previousDriveTime = Timer.getFPGATimestamp();
        return updated_chassis_speeds;
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         See
     *         {@link https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1}
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeedsFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);
        double fudgeFactor = chassisSpeeds.omegaRadiansPerSecond / linearVelocity * FUDGE_FACTOR_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /**
     * @param chassisSpeeds
     * @return adjusted chassisSpeeds
     *         Takes the unit orthogonal vector and scales it by a constant times
     *         the angular velocity
     */
    public static ChassisSpeeds reduceSkewFromChassisSpeedsSimpleFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);

        double fudgeFactor = Math.signum(chassisSpeeds.omegaRadiansPerSecond) * FUDGE_FACTOR_SIMPLE_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthY = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthY;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }
}
