package frc.robot;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.autonomous.paths.Chaos;
import frc.robot.autonomous.paths.PathInterface;
import frc.robot.autonomous.paths.Shutout;
import frc.robot.autonomous.paths.Simple;
import frc.robot.autonomous.paths.Trench;
import frc.robot.subsystems.climber.ClimberSubsystem;
import frc.robot.subsystems.climber.commands.DriveClimber;
import frc.robot.subsystems.climber.commands.LowerClimber;
import frc.robot.subsystems.climber.commands.RaiseClimber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.commands.AutoAlign;
import frc.robot.subsystems.drive.commands.Drive;
import frc.robot.subsystems.drive.commands.ResetGyro;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.indexer.commands.Index;
import frc.robot.subsystems.indexer.commands.IndexFast;
import frc.robot.subsystems.indexer.commands.IndexShooter;
import frc.robot.subsystems.indexer.commands.Outdex;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.intake.commands.Intake;
import frc.robot.subsystems.intake.commands.IntakeWhileShooting;
import frc.robot.subsystems.intake.commands.Outtake;
import frc.robot.subsystems.leds.LedSubsystem;
import frc.robot.subsystems.leds.commands.SabotageText;
import frc.robot.subsystems.leds.commands.SetLeds;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.shooter.commands.ShootFountain;
import frc.robot.subsystems.shooter.commands.SpinClose;
import frc.robot.subsystems.shooter.commands.SpinDistance;
import frc.robot.subsystems.spinner.SpinnerSubsystem;
import frc.robot.subsystems.spinner.commands.SpinnerStage2;
import frc.robot.subsystems.spinner.commands.SpinnerStage3;

public class RobotContainer {
    // The robot's subsystems
    public static final DriveSubsystem swerve = new DriveSubsystem();
    public static final IntakeSubsystem intake = new IntakeSubsystem();
    public static final IndexerSubsystem indexer = new IndexerSubsystem();
    public static final ShooterSubsystem shooter = new ShooterSubsystem();
    public static final SpinnerSubsystem spinner = new SpinnerSubsystem();
    public static final ClimberSubsystem climber = new ClimberSubsystem();
    public static final LedSubsystem leds = new LedSubsystem();

    private SendableChooser<PathInterface> sChooser;

    // The driver's controller
    XboxController driverController = new XboxController(0);
    XboxController operatorController = new XboxController(1);
    Joystick joystick = new Joystick(0);
    Joystick opJoystick = new Joystick(1);

    private boolean fieldRelative = true;

    // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private double xSpeed;
    private double ySpeed;
    private double rot;

    public static final double x = 0.257175; // 10.125"
    public static final double y = 0.32146875; // 12.65625"

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(new Translation2d(y, x),
            new Translation2d(y, -x), new Translation2d(-y, x), new Translation2d(-y, -x));

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        sChooser = new SendableChooser<PathInterface>();

        sChooser.setDefaultOption("Simple", new Simple());
        sChooser.addOption("Trench", new Trench());
        sChooser.addOption("Simple", new Simple());
        sChooser.addOption("Chaos", new Chaos());
        sChooser.addOption("Shutout", new Shutout());
    
        Shuffleboard.getTab("Auton").add(sChooser);
        swerve.initDefaultCommand();
        leds.initDefaultCommand();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * calling passing it to a {@link JoystickButton}.
     */
    public void configureButtonBindings() {

        //Driver Buttons
        JoystickButton startButton = new JoystickButton(joystick, 8);
        startButton.whenPressed(new ResetGyro(swerve));
        JoystickButton selectButton = new JoystickButton(joystick, 7);
        selectButton.whenPressed(() -> {
            System.out.println("Toggled field centric!");
            fieldRelative = !fieldRelative;
        });
        JoystickButton leftBumper = new JoystickButton(joystick, 5);
        JoystickButton yButton = new JoystickButton(joystick, 4);

        yButton.whenActive(new AutoAlign(swerve));
        yButton.whenInactive(new Drive(swerve, fieldRelative));

        JoystickButton oLeftBumper = new JoystickButton(opJoystick, 5);

        Button shooterTrigger = new Button() {

            @Override
            public boolean get() {
                return shooter.atTargetRpm();
            }

        };

        ConditionalCommand indexerSpeed = new ConditionalCommand(new IndexFast(indexer), new IndexShooter(indexer), shooter::isFastIndexer);
        Command spin = new SpinDistance(shooter);
        Command shoot = new IndexShooter(indexer);
        Command intakeDistance = new IntakeWhileShooting(intake);
        ParallelCommandGroup shootGroup = new ParallelCommandGroup(shoot, intakeDistance);
        shooterTrigger.whenPressed(shootGroup);
        leftBumper.whileHeld(spin);
        leftBumper.whenReleased(() -> {
            shootGroup.cancel();
        });

        Button down = new Button() {
            public boolean get() {
                return driverController.getPOV() == 180;
            }
        };

        ParallelCommandGroup shootFountainCommandGroup = new ParallelCommandGroup(new IndexFast(indexer), new ShootFountain(shooter));
        down.whileHeld(shootFountainCommandGroup);

        Button rightTrigger = new Button() {
            
            public boolean get() {
                return driverController.getTriggerAxis(Hand.kRight) > 0.1;
            }

        };

        
        // rightTrigger.whenHeld(new Intake(intake, indexer));

        Button indexerSensorTriggered = new Button() {
            public boolean get() {
                if(!rightTrigger.get() && (indexer.getProx1() == false && indexer.getProx2() == false)) {
                   return false;
                }
                else if(leftBumper.get() || oLeftBumper.get()) {
                    return false;
                } 
                else if(rightTrigger.get() || indexer.getProx1() == false) {
                    return true;
                } else {
                    return false;
                }
            }
        };

        Button leftTrigger = new Button() {
            
            public boolean get() {
                return driverController.getTriggerAxis(Hand.kLeft) > 0.1;
            }

        };

        Command intakeCommand = new Intake(intake, indexer);

        indexerSensorTriggered.whenPressed(intakeCommand);
        indexerSensorTriggered.whenReleased(() -> {
            intakeCommand.cancel();
        });

        ParallelCommandGroup outtakeGroup = new ParallelCommandGroup(new Outtake(intake), new Outdex(indexer));
        leftTrigger.whenHeld(outtakeGroup);

        //Operator Buttons
        // SpinnerStage2 stage2 = new SpinnerStage2(spinner);
        // SpinnerStage2 stage3 = new SpinnerStage3(spinner);

        Command intakeDistance2 = new IntakeWhileShooting(intake);
        Command spinClose = new SpinClose(shooter);
        // ParallelCommandGroup spinClose = new ParallelCommandGroup(new SpinClose(shooter), intakeDistance2);
        oLeftBumper.whileHeld(spinClose);
        oLeftBumper.whenReleased(() -> {
            shootGroup.cancel();
        });

        JoystickButton oAButton = new JoystickButton(opJoystick, 1);
        oAButton.whenPressed(new SpinnerStage2(spinner));
        JoystickButton oBButton = new JoystickButton(opJoystick, 2);
        oBButton.whenPressed(new SpinnerStage3(spinner));
        JoystickButton oRBButton = new JoystickButton(opJoystick, 6);
        JoystickButton oXButton = new JoystickButton(opJoystick, 3);
        JoystickButton oYButton = new JoystickButton(opJoystick, 4);

        Button raiseClimber = new Button(() -> oRBButton.get() && oXButton.get());

        ParallelCommandGroup climbLeds = new ParallelCommandGroup(new RaiseClimber(climber), new SabotageText(leds));

        raiseClimber.whenPressed(climbLeds);

        Button lowerClimber = new Button(() -> oRBButton.get() && oYButton.get());

        lowerClimber.whenPressed(new LowerClimber(climber));

        Button leftY = new Button() {
            
            public boolean get() {
                return Math.abs(operatorController.getY(GenericHID.Hand.kLeft)) > 0.15;
            }

        };

        leftY.whenActive(new DriveClimber(climber));

        //TODO add spinner cancel option
        // Button oRight = new Button() {
        //     public boolean get() {
        //         return driverController.getPOV() == 90;
        //     }
        // };

        

        // BooleanSupplier spinnerAction;
        // if(stage2.isScheduled()) {
        //     spinnerAction = () -> true;
        // } else {
        //     spinnerAction = () -> false;
        // }

        // ConditionalCommand spinnerCancelCommand = new ConditionalCommand(stage2.cancel(), null, spinnerAction);

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() { 

        CommandScheduler.getInstance().clearButtons();

        swerve.updateOdometry();

        Command autoCommand = sChooser.getSelected().getAutoCommand();

        // Run path following command, then stop at the end.
        return autoCommand.andThen(() -> swerve.drive(0, 0, 0, false));
    }
}
