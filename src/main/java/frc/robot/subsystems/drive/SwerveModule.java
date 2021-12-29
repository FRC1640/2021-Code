// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drive;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;

public class SwerveModule {
	private static final double kWheelRadius = 0.0508; 
	private static final int kEncoderResolution = 4096;

	private static final double kModuleMaxAngularVelocity = DriveSubsystem.kMaxAngularSpeed;
	private static final double kModuleMaxAngularAcceleration = 2 * Math.PI; // radians per second squared

	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private CANEncoder driveEncoder;
	public Resolver turningEncoder;

	private final PIDController drivePIDController = new PIDController(0.05, 0, 0.0);

	private final PIDController turningPIDController = new PIDController(0.725, 0.0, 0.005);

	// Gains are for example purposes only - must be determined for your own robot!
	private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(4, 3);

	private PivotConfig cfg;

	/**
	 * Constructs a SwerveModule.
	 *
	 * @param driveMotorChannel   ID for the drive motor.
	 * @param turningMotorChannel ID for the turning motor.
	 */
	public SwerveModule(PivotConfig cfg) {
		this.cfg = cfg;
		driveMotor = new CANSparkMax(cfg.getDriveChannel(), MotorType.kBrushless);
		turningMotor = new CANSparkMax(cfg.getSteerChannel(), MotorType.kBrushless);

		driveEncoder = driveMotor.getEncoder();
		turningEncoder = new Resolver(cfg.getResolverChannel(), cfg.getMinVoltage(), cfg.getMaxVoltage(),
				cfg.getOffset(), cfg.isReverseAngle());

		// turningEncoder = new Resolver(cfg.getResolverChannel(), 2.5, 2.5,
		// 		cfg.getOffset(), cfg.isReverseAngle());

		driveMotor.setInverted(cfg.isReverseDrive());
		turningMotor.setInverted(cfg.isReverseSteer());

		// Set the distance per pulse for the drive encoder. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// resolution.
		// driveEncoder.setPositionConversionFactor(2 * Math.PI * kWheelRadius *
		// 0.10472); 
		// driveEncoder.setVelocityConversionFactor(2 * Math.PI * kWheelRadius *
		// 0.10472); //9 ft/s

		// Set the distance (in this case, angle) per pulse for the turning encoder.
		// This is the the angle through an entire rotation (2 * wpi::math::pi)
		// divided by the encoder resolution.
		// turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
		// turningPIDController.enableContinuousInput(0, 2 * Math.PI);
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), new Rotation2d(turningEncoder.get()));
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param state Desired state with speed and angle.
	 */
	// public void setDesiredState(SwerveModuleState state) {
	// // Calculate the drive output from the drive PID controller.
	// final double driveOutput =
	// drivePIDController.calculate(driveEncoder.getVelocity(),
	// state.speedMetersPerSecond);

	// final double driveFeedforward =
	// driveFeedforward.calculate(state.speedMetersPerSecond);

	// // Calculate the turning motor output from the turning PID controller.
	// final double turnOutput =
	// turningPIDController.calculate(turningEncoder.get(),
	// state.angle.getRadians());

	// final double turnFeedforward =
	// turnFeedforward.calculate(turningPIDController.getSetpoint().velocity);

	// driveMotor.setVoltage(driveOutput + driveFeedforward);
	// turningMotor.setVoltage(turnOutput + turnFeedforward);
	// }

	public void resetEncoder() {
		driveEncoder.setPosition(0);
	}

	public double getPositionX() {
		return driveEncoder.getPosition() * kWheelRadius * 0.10472 * 0.12 * Math.sin(turningEncoder.get());
	}

	public double getPositionY() {
		return driveEncoder.getPosition() * kWheelRadius * 0.10472 * 0.12 * Math.cos(turningEncoder.get());
	}

	public double getVelocity() {
		return driveEncoder.getVelocity() * kWheelRadius * 0.10472 * 0.12;
	}

	public double echo(double val) {
		return val;
	}

	public SwerveModuleState desiredState = null;

	public void setDesiredState(SwerveModuleState state) {
		desiredState = state;

		// Calculate the drive output from the drive PID controller
		// final double driveFeedforward =
		// driveFeedforward.calculate(state.speedMetersPerSecond);

		// System.out.println(x);

		double dAngle = state.angle.getDegrees() - turningEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		final double targetSpeed = flipDrive ? state.speedMetersPerSecond : -state.speedMetersPerSecond;

		if (Math.abs(targetSpeed) < 0.1) {
			turnOutput = 0;
		}

		// Logger.log((driveEncoder.getVelocity() * kWheelRadius * 0.10472) * 3.281 + "");
		// Logger.log(cfg.getName().getName() + " " + turningEncoder.toString());
		// drivePIDController.calculate(driveEncoder.getVelocity(), flipDrive ?
		// -state.speedMetersPerSecond : state.speedMetersPerSecond);
		// Logger.log("Current " + driveEncoder.getVelocity() + " Target " +
		// state.speedMetersPerSecond + " Output " + driveOutput);

		driveMotor.set(targetSpeed);
		turningMotor.set(turnOutput);
	}

	public void setDesiredStateAuto(SwerveModuleState state) {
		desiredState = state;

		// Calculate the drive output from the drive PID controller
		// final double driveFeedforward =
		// driveFeedforward.calculate(state.speedMetersPerSecond);

		// System.out.println(x);

		double dAngle = state.angle.getDegrees() - turningEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		final double targetSpeed = flipDrive ? state.speedMetersPerSecond : -state.speedMetersPerSecond;

		if (Math.abs(targetSpeed) < 0.1) {
			turnOutput = 0;
		}

		double vSetpoint = driveFeedforward.calculate(targetSpeed);
		double driveOutput = drivePIDController.calculate(getVelocity(), vSetpoint);

		// Logger.log((driveEncoder.getVelocity() * kWheelRadius * 0.10472) * 3.281 + "");
		// Logger.log(cfg.getName().getName() + " " + turningEncoder.toString());
		// drivePIDController.calculate(driveEncoder.getVelocity(), flipDrive ?
		// -state.speedMetersPerSecond : state.speedMetersPerSecond);
		// Logger.log("Current " + driveEncoder.getVelocity() + " Target " +
		// state.speedMetersPerSecond + " Output " + driveOutput);

		driveMotor.set(driveOutput);
		turningMotor.set(turnOutput);
	}

	public void setAngleD(double angle) {
		double dAngle = angle - turningEncoder.getD();
		double dAngleAbs = Math.abs(dAngle) % 360;
		boolean flipDrive = (90.0 <= dAngleAbs) && (dAngleAbs <= 270.0);
		double sin = Math.sin(dAngle * Math.PI / 180.0);
		sin = (flipDrive) ? -sin : sin;
		double turnOutput = turningPIDController.calculate(sin, 0);

		turningMotor.set(turnOutput);
	}
}
