package frc.robot.subsystems.intake.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class Intake extends CommandBase {
    
    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    public Intake (IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(intakeSubsystem, indexerSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intakeSubsystem.deployIntake();
        intakeSubsystem.intake(); 
        if(indexerSubsystem.getProx1() == false ) {
            if(indexerSubsystem.getProx2() == true) {
                indexerSubsystem.runIndexer();
            } else {
                indexerSubsystem.stopIndexer();
            }
        } else {
            indexerSubsystem.stopIndexer();
        }

        if(indexerSubsystem.getProx2() == false) {
            intakeSubsystem.stopFunnel();
        } else {
            intakeSubsystem.runFunnel();
        }
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopFunnel();
        indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
