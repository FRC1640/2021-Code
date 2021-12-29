package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;

public class IntakeAndIndex extends CommandBase {

    IntakeSubsystem intakeSubsystem;
    IndexerSubsystem indexerSubsystem;

    boolean prevProxState;
    boolean curProxState;
    long duration;
    long initialTime;
    long elapsedTime;

    public IntakeAndIndex(IntakeSubsystem intakeSubsystem, IndexerSubsystem indexerSubsystem, long duration) {
        this.indexerSubsystem = indexerSubsystem;
        this.intakeSubsystem = intakeSubsystem;
        this.duration = duration;
        addRequirements(indexerSubsystem, intakeSubsystem);
    }

    @Override
    public void initialize() {
        initialTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {
        if(indexerSubsystem.getProx1() == false && indexerSubsystem.getProx2() == true) {
            indexerSubsystem.runIndexer();
        } else {
            indexerSubsystem.stopIndexer();
        }

        intakeSubsystem.deployIntake();
        intakeSubsystem.intake(); 
        intakeSubsystem.runFunnel();

        curProxState = indexerSubsystem.getProx2();
        if (curProxState != prevProxState) {
            indexerSubsystem.setBallCount(indexerSubsystem.getBallCount() + 1);
        }
        prevProxState = curProxState;
        
    }

    @Override
    public void end(boolean interrupted) {
        indexerSubsystem.stopIndexer();
        intakeSubsystem.retractIntake();
        intakeSubsystem.stopIntake();
        intakeSubsystem.stopFunnel();
    }

    @Override
    public boolean isFinished() {
        elapsedTime = System.currentTimeMillis();
        return elapsedTime - initialTime >= duration;
    }
    
}
