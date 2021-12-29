package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeWhileShooting extends CommandBase {

    IntakeSubsystem intakeSubsystem;

    public IntakeWhileShooting(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSubsystem.deployIntake();
        intakeSubsystem.runFunnel();
        intakeSubsystem.intake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopFunnel();
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
