package frc.robot.autonomous.paths;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.commands.AlignAuton;
import frc.robot.autonomous.commands.IntakeAndIndex;
import frc.robot.autonomous.commands.IntakeTime;
import frc.robot.autonomous.commands.PointWheels;
import frc.robot.autonomous.commands.Shoot5;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Chaos implements PathInterface {

        public static final double x = 0.257175; // 10.125"
        public static final double y = 0.32146875; // 12.65625"

        private final DriveSubsystem swerve = RobotContainer.swerve;
        private final IntakeSubsystem intake = RobotContainer.intake;
        private final IndexerSubsystem indexer = RobotContainer.indexer;
        private final ShooterSubsystem shooter = RobotContainer.shooter;

        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                        Math.PI, Math.PI);

        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),
                        new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

        @Override
        public Command getAutoCommand() {
                // Create config for trajectory
                TrajectoryConfig config = new TrajectoryConfig(2.0, 1.0)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(kDriveKinematics);
                config.setReversed(true);

                TrajectoryConfig config2 = new TrajectoryConfig(3.0, 1.0)
                                // Add kinematics to ensure max speed is actually obeyed
                                .setKinematics(kDriveKinematics);
                config2.setReversed(false);

                // An example trajectory to follow. All units in meters.
                Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(-1, 0.05)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(-1.4, 0.1, new Rotation2d(0)), config);

                Trajectory exampleTrajectory2 = TrajectoryGenerator.generateTrajectory(
                                // Start at the origin facing the +X direction
                                new Pose2d(0, 0, new Rotation2d(0)),
                                // Pass through these two interior waypoints, making an 's' curve path
                                List.of(new Translation2d(1, -1)),
                                // End 3 meters straight ahead of where we started, facing forward
                                new Pose2d(1.5, -2.5, new Rotation2d(-Math.PI/4)), config2);

                var thetaController = new ProfiledPIDController(-5, 0.0, 0.0, kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(exampleTrajectory,
                                swerve::getPose, // Functional interface to feed supplier
                                kDriveKinematics, new PIDController(0.15, 0.0, 0.0), new PIDController(-3, 0.0, -0.2),
                                thetaController, swerve::setModuleStates, swerve);

                SwerveControllerCommand swerveControllerCommand2 = new SwerveControllerCommand(exampleTrajectory2,
                                swerve::getPose, // Functional interface to feed supplier
                                kDriveKinematics, new PIDController(0.15, 0.0, 0.0), new PIDController(0.15, 0.0, 0.0),
                                thetaController, swerve::setModuleStates, swerve);

                Command pointWheels1 = new PointWheels(swerve, 0);
                ParallelCommandGroup intakeAndDrive = new ParallelCommandGroup(
                                new IntakeAndIndex(intake, indexer, 2250), swerveControllerCommand);
                Command intake1 = new IntakeTime(intake, 500);
                ParallelCommandGroup intakeAndDrive2 = new ParallelCommandGroup(
                                new IntakeAndIndex(intake, indexer, 1500), swerveControllerCommand2);
                Command align = new AlignAuton(swerve, shooter);

                Command shoot = new Shoot5(shooter, indexer, intake, 4000);

                SequentialCommandGroup chaos = new SequentialCommandGroup(pointWheels1, intakeAndDrive, intake1,
                                intakeAndDrive2, align, shoot);

                // Reset odometry to the starting pose of the trajectory.
                swerve.resetOdometry(exampleTrajectory.getInitialPose());

                return chaos;
        }

}