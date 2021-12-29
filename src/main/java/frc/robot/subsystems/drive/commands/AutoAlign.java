package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LedEnum;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;

public class AutoAlign extends CommandBase {

    private DriveSubsystem driveSubsystem;
    XboxController driverController = new XboxController(0);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);

    private double xSpeed;
    private double ySpeed;

    private Limelight limelight = new Limelight();
    private PIDController limelightController = new PIDController(0.05, 0.0, 0.0);

    public AutoAlign(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {

        limelight.setProcessing(true);
        limelight.setLEDOn(LedEnum.FORCE_ON);

        xSpeed = -m_xspeedLimiter.calculate(driverController.getY(GenericHID.Hand.kLeft));
        ySpeed = -m_yspeedLimiter.calculate(driverController.getX(GenericHID.Hand.kLeft));

        if (Math.hypot(xSpeed, ySpeed) < 0.15) {
            xSpeed = 0;
            ySpeed = 0;
        }

        double rot = limelightController.calculate(limelight.getTargetX(0), 1.5);

        driveSubsystem.drive(xSpeed, ySpeed, -rot, true);

        // System.out.println(limelight.getTargetX(0.0));
        // System.out.println(limelight.getTargetX(0.0) <= 0.5 && limelight.getTargetX(0.0) >= -0.5);
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setProcessing(false);
        limelight.setLEDOn(LedEnum.FORCE_OFF);
     }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
