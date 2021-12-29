package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.drive.DriveSubsystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;

public class Drive extends CommandBase {

    DriveSubsystem driveSubsystem;
    XboxController driverController = new XboxController(0);
    Joystick joystick = new Joystick(0);

    private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    private double xSpeed;
    private double ySpeed;
    private double rot;

    private boolean fieldRelative;

    public Drive(DriveSubsystem driveSubsystem, boolean fieldRelative) {
        this.fieldRelative = fieldRelative;
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        fieldRelative = true;

        boolean slowModeState = false;

        JoystickButton aButton = new JoystickButton(joystick, 1);

        if(aButton.get()) {
            slowModeState = true;
        }

        if(slowModeState) {
            xSpeed = -driverController.getY(GenericHID.Hand.kLeft) * 0.25;
            ySpeed = -driverController.getX(GenericHID.Hand.kLeft) * 0.25;
            rot = driverController.getX(GenericHID.Hand.kRight) * 0.25; 
        } else {
            xSpeed = -m_xspeedLimiter.calculate(driverController.getY(GenericHID.Hand.kLeft));
            ySpeed = -m_yspeedLimiter.calculate(driverController.getX(GenericHID.Hand.kLeft));
            rot = m_rotLimiter.calculate(driverController.getX(GenericHID.Hand.kRight));
        }

        if (Math.hypot(xSpeed, ySpeed) < 0.15) {
            xSpeed = 0;
            ySpeed = 0;
        }

        if (Math.abs(rot) < 0.15) {
            rot = 0;
        }

        driveSubsystem.drive(xSpeed, ySpeed, rot * 2, fieldRelative);
    }

    @Override
    public void end(boolean interrupted) { }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
