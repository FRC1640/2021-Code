package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class IndexShooter extends CommandBase {

    IndexerSubsystem indexerSubsystem;

    public IndexShooter (IndexerSubsystem indexerSubsystem) {
        this.indexerSubsystem = indexerSubsystem;
        addRequirements(indexerSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        indexerSubsystem.runIndexer();
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
