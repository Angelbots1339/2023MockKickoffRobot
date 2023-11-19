package frc.robot.subsystems;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.Conversions;
import frc.lib.util.TalonFXFactory;
import frc.lib.util.ErrorCheckUtil;
import frc.lib.util.swerve.SwerveEncoder;
import frc.lib.util.swerve.SwerveMath;
import frc.lib.util.swerve.SwerveModuleConstants;
import frc.lib.util.swerve.SwerveEncoder.EncoderType;
import frc.robot.Constants;

import static frc.robot.Constants.SwerveConstants.FalconConfigConstants.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.VelocityVoltage;
// import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.CANcoder;

public class SwerveModule {
        public int moduleNumber;
        public Rotation2d angleOffset;
        private Rotation2d lastAngle;

        private TalonFX angleMotor;
        private TalonFX driveMotor;
        private SwerveEncoder angleEncoder;

        private SwerveModuleState desiredState = new SwerveModuleState();

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.SwerveConstants.DRIVE_KS,
                        Constants.SwerveConstants.DRIVE_KV, Constants.SwerveConstants.DRIVE_KA);

        public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
                this.moduleNumber = moduleNumber;
                this.angleOffset = moduleConstants.angleOffset;

                /* Angle Encoder Config */
                angleEncoder = new SwerveEncoder(moduleConstants.cancoderID, Constants.RIO, EncoderType.CANCODER);
                configAngleEncoder();

                /* Angle Motor Config */
                angleMotor = TalonFXFactory.createDefaultTalon(moduleConstants.angleMotorID, Constants.RIO);
                // angleMotor = new WPI_TalonFX(moduleConstants.angleMotorID,
                // Constants.CANIVORE);
                configAngleMotor();

                /* Drive Motor Config */
                driveMotor = TalonFXFactory.createDefaultTalon(moduleConstants.driveMotorID, Constants.RIO);
                // driveMotor = new WPI_TalonFX(moduleConstants.driveMotorID,
                // Constants.CANIVORE);
                configDriveMotor();

                lastAngle = getState().angle;
        }

        /**
         * 
         * @param desiredState
         * @param isOpenLoop
         */
        public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
                /*
                 * This is a custom optimize function, since default WPILib optimize assumes
                 * continuous controller which CTRE and Rev onboard is not
                 */
                desiredState = SwerveMath.optimize(desiredState, getState().angle);
                setAngle(desiredState);
                setSpeed(desiredState, isOpenLoop);
                this.desiredState = desiredState;
        }

        /**
         *
         * @param desiredState
         * @param isOpenLoop
         */
        public void setDesiredStateStrict(SwerveModuleState desiredState, boolean isOpenLoop) {

                desiredState = SwerveMath.optimize(desiredState, getState().angle);
                setAngleStrict(desiredState);
                setSpeed(desiredState, isOpenLoop);
                this.desiredState = desiredState;
        }

        /**
         * 
         * @param desiredState
         * @param isOpenLoop
         */
        private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
                if (isOpenLoop) {
                        double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.MAX_SPEED;
                        driveMotor.setControl(new DutyCycleOut(percentOutput));
                } else {
                        double velocity = Conversions.MPSToRPS(desiredState.speedMetersPerSecond,
                                        Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                                        Constants.SwerveConstants.DRIVE_GEAR_RATIO);
                        // if(Math.abs(desiredState.speedMetersPerSecond) <= MIN_CLOSE_LOOP_SPEED){
                        // driveMotor.set(ControlMode.PercentOutput,
                        // feedforward.calculate(desiredState.speedMetersPerSecond));
                        // return;
                        // }

                        VelocityVoltage velocityVoltage = new VelocityVoltage(velocity).withSlot(0);
                        driveMotor.setControl(velocityVoltage.withVelocity(velocity)
                                        .withFeedForward(feedforward.calculate(desiredState.speedMetersPerSecond)));

                        // driveMotor.setControl(new VelocityVoltage(velocity)) (ControlMode.Velocity,
                        // velocity, DemandType.ArbitraryFeedForward,
                        // feedforward.calculate(desiredState.speedMetersPerSecond));
                }
        }

        private void setAngle(SwerveModuleState desiredState) {
                Rotation2d angle = (Math
                                .abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.MAX_SPEED * 0.01))
                                                ? lastAngle
                                                : desiredState.angle; // Prevent rotating module if speed is less then
                // 1%. Prevents Jittering.

                // angleMotor.setControl(new PositionVoltage(ANGLE_KF,
                // DRIVE_SENSOR_VELOCITY_MEAS_WINDOW, CANCODER_INVERT, ANGLE_KD,
                // DRIVE_SENSOR_VELOCITY_MEAS_WINDOW, CANCODER_INVERT)) (ControlMode.Position,
                // Conversions.degreesToFalcon(angle.getDegrees(),
                // Constants.SwerveConstants.ANGLE_GEAR_RATIO));

                PositionVoltage positionVoltage = new PositionVoltage(Conversions.gearRatioConvert(angle.getRotations(),
                                Constants.SwerveConstants.ANGLE_GEAR_RATIO)).withSlot(0);

                angleMotor.setControl(positionVoltage);

                lastAngle = angle;
        }

        private void setAngleStrict(SwerveModuleState desiredState) {

                PositionVoltage positionVoltage = new PositionVoltage(
                                Conversions.gearRatioConvert(desiredState.angle.getRotations(),
                                                Constants.SwerveConstants.ANGLE_GEAR_RATIO))
                                .withSlot(0);

                angleMotor.setControl(positionVoltage);

                // angleMotor.set(ControlMode.Position,
                // Conversions.degreesToFalcon(desiredState.angle.getDegrees(),
                // Constants.SwerveConstants.ANGLE_GEAR_RATIO));
                lastAngle = desiredState.angle;
        }

        private Rotation2d getAngle() {
                return Rotation2d.fromRotations(Conversions.gearRatioConvert(angleMotor.getPosition().getValue(),
                                Constants.SwerveConstants.ANGLE_GEAR_RATIO));
        }

        public Rotation2d getCanCoder() {
                return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
        }

        public TalonFX getAngleMotor() {
                return angleMotor;
        }

        public TalonFX getDriveMotor() {
                return angleMotor;
        }

        public void resetDriveEncoder() {
                driveMotor.setPosition(0);
        }

        public double getEncoderInMeters() {
                return Conversions.rotationsToMeters(driveMotor.getPosition().getValue(),
                                Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                                Constants.SwerveConstants.DRIVE_GEAR_RATIO);
        }

        public void resetToAbsolute() {
                double absolutePosition = Conversions.degreesToRotations(
                                angleEncoder.getAbsolutePosition() - angleOffset.getDegrees(),
                                Constants.SwerveConstants.ANGLE_GEAR_RATIO);
                angleMotor.setPosition(absolutePosition);

        }

        private void configAngleEncoder() {
                angleEncoder.configure(moduleNumber);
        }

        private void configAngleMotor() {

                TalonFXConfiguration angleMotorConfig = new TalonFXConfiguration();
                angleMotor.getConfigurator().refresh(angleMotorConfig, 0.1);

                angleMotorConfig.Slot0.kP = ANGLE_KP;
                angleMotorConfig.Slot0.kI = ANGLE_KI;
                angleMotorConfig.Slot0.kD = ANGLE_KD;

                angleMotorConfig.CurrentLimits.SupplyCurrentLimit = ANGLE_CONTINUOUS_CURRENT_LIMIT;
                angleMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = ANGLE_ENABLE_CURRENT_LIMIT;
                angleMotorConfig.CurrentLimits.SupplyTimeThreshold = ANGLE_PEAK_CURRENT_DURATION;

                angleMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
                angleMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;

                angleMotorConfig.Feedback.FeedbackRotorOffset = 0;

                angleMotorConfig.MotorOutput.Inverted = ANGLE_MOTOR_INVERT;
                angleMotorConfig.MotorOutput.NeutralMode = ANGLE_NEUTRAL_MODE;

                resetToAbsolute();
        }

        private void configDriveMotor() {

                TalonFXConfiguration driveMotorConfig = new TalonFXConfiguration();
                angleMotor.getConfigurator().refresh(driveMotorConfig, 0.1);

                driveMotorConfig.Slot0.kP = DRIVE_KP;
                driveMotorConfig.Slot0.kI = 0;
                driveMotorConfig.Slot0.kD = DRIVE_KD;

                driveMotorConfig.CurrentLimits.SupplyCurrentLimit = DRIVE_CONTINUOUS_CURRENT_LIMIT;
                driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = DRIVE_ENABLE_CURRENT_LIMIT;
                driveMotorConfig.CurrentLimits.SupplyTimeThreshold = DRIVE_PEAK_CURRENT_DURATION;

                driveMotorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = CLOSED_LOOP_RAMP;
                driveMotorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = OPEN_LOOP_RAMP;

                driveMotorConfig.MotorOutput.Inverted = DRIVE_MOTOR_INVERT;
                driveMotorConfig.MotorOutput.NeutralMode = DRIVE_NEUTRAL_MODE;

                driveMotor.setPosition(0);

        }

        public SwerveModuleState getState() {
                return new SwerveModuleState(
                                Conversions.RPSToMPS(driveMotor.getVelocity().getValue(),
                                                Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                                                Constants.SwerveConstants.DRIVE_GEAR_RATIO),
                                getAngle());
        }

        public SwerveModuleState getDesiredState() {
                return desiredState;
        }

        public SwerveModulePosition getPosition() {
                return new SwerveModulePosition(Conversions.rotationsToMeters(driveMotor.getPosition().getValue(),
                                Constants.SwerveConstants.WHEEL_CIRCUMFERENCE,
                                Constants.SwerveConstants.DRIVE_GEAR_RATIO), getAngle());
        }

        public double getRotations() {
                return Conversions.gearRatioConvert(driveMotor.getPosition().getValue(),
                                Constants.SwerveConstants.DRIVE_GEAR_RATIO);
        }

}