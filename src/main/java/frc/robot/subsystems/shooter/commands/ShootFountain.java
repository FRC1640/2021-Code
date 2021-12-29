package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootFountain extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public ShootFountain (ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.setHoodFountain();
        shooterSubsystem.shootFountain();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodRest();
        shooterSubsystem.stopShooter();
        shooterSubsystem.resetTargetRpm();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
