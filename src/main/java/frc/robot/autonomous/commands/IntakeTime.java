package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeTime extends CommandBase {
    
    IntakeSubsystem intakeSubsystem;
    long duration;
    long initialTime;
    long elapsedTime;

    public IntakeTime (IntakeSubsystem intakeSubsystem, long duration) {
        this.intakeSubsystem = intakeSubsystem;
        this.duration = duration;
        addRequirements(intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        initialTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        intakeSubsystem.deployIntake();
        intakeSubsystem.intake(); //TODO figure out stopping when the indexer is full
        intakeSubsystem.runFunnel();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopFunnel();
    }

    @Override
    public boolean isFinished() {
        elapsedTime = System.currentTimeMillis();
        // System.out.println(elapsedTime - initialTime);
        return elapsedTime - initialTime >= duration;
    }

}
