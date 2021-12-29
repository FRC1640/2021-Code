package frc.robot.subsystems.shooter.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class SpinClose extends CommandBase {

    ShooterSubsystem shooterSubsystem;

    public SpinClose(ShooterSubsystem shooterSubsystem) {
        this.shooterSubsystem = shooterSubsystem;
        addRequirements(shooterSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        shooterSubsystem.shootClose();
        shooterSubsystem.setHoodClose();
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.setHoodRest();
        shooterSubsystem.stopShooter();
        shooterSubsystem.resetTargetRpm();
        System.out.println("done spin close");
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
