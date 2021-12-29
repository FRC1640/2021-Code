package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpinDistance extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public SpinDistance (ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.ledOn();
        shooterSubsystem.setHoodDistance();
        shooterSubsystem.shootDistance();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.ledOff();
        shooterSubsystem.setHoodRest();
        shooterSubsystem.stopShooter();
        shooterSubsystem.resetTargetRpm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
