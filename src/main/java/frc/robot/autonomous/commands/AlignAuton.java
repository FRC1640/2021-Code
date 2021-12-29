package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LedEnum;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import edu.wpi.first.wpilibj.GenericHID;

public class AlignAuton extends CommandBase {

    private DriveSubsystem driveSubsystem;
    ShooterSubsystem shooterSubsystem;
    XboxController driverController = new XboxController(0);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);

    private double xSpeed;
    private double ySpeed;

    private Limelight limelight = new Limelight();
    private PIDController limelightController = new PIDController(0.05, 0.0, 0.0);

    private long initialTime;
    private long elapsedTime;

    private long startTime;

    public AlignAuton(DriveSubsystem driveSubsystem, ShooterSubsystem shooterSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(driveSubsystem, shooterSubsystem);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {

        limelight.setProcessing(true);
        limelight.setLEDOn(LedEnum.FORCE_ON);

        shooterSubsystem.setHoodDistance();
        shooterSubsystem.shootDistance();

        xSpeed = -m_xspeedLimiter.calculate(driverController.getY(GenericHID.Hand.kLeft));
        ySpeed = -m_yspeedLimiter.calculate(driverController.getX(GenericHID.Hand.kLeft));

        if (Math.hypot(xSpeed, ySpeed) < 0.15) {
            xSpeed = 0;
            ySpeed = 0;
        }

        double rot = limelightController.calculate(limelight.getTargetX(0.0), 2);

        driveSubsystem.drive(xSpeed, ySpeed, -rot * 2, true);
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setProcessing(false);
        limelight.setLEDOn(LedEnum.FORCE_OFF);
     }

    @Override
    public boolean isFinished() {
        if(limelight.getTargetX(0.0) <= 2 && limelight.getTargetX(0.0) >= -0.5 && limelight.getTargetX(0.0) != 0.0) {
            elapsedTime = System.currentTimeMillis();
        } else {
            initialTime = System.currentTimeMillis();
        }

        if(elapsedTime - initialTime >= 500) {
            System.out.println("first condition");
            return true;
        }

        if(elapsedTime - startTime >= 1000) {
            System.out.println("second condition");
            return true;
        }
        return false; //TODO add time override
    }
    
}
