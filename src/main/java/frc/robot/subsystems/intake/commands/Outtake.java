package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class Outtake extends CommandBase {

    IntakeSubsystem intakeSubsystem;

    public Outtake (IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSubsystem.deployIntake();
        intakeSubsystem.outtake();
        intakeSubsystem.reverseFunnel();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopFunnel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
