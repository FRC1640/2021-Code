package frc.robot.subsystems.indexer.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Proximity;
import frc.robot.subsystems.indexer.IndexerSubsystem;

public class Index extends CommandBase {

    IndexerSubsystem indexerSubsystem;
    long initialTime;
    long time;
    boolean stop;

    public Index (IndexerSubsystem indexerSubsystem) {
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
        if(indexerSubsystem.getProx1() == true) {
            return true;
        }

        if(indexerSubsystem.getProx2() == false && stop == false) {
            initialTime = System.currentTimeMillis();
            stop = true;
        } 
        else if(indexerSubsystem.getProx2() == true) {
            initialTime = System.currentTimeMillis();
        }
        return System.currentTimeMillis() - initialTime >= 50;
    }
    
}
