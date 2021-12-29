package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootCount extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    IndexerSubsystem indexerSubsytem;
    IntakeSubsystem intakeSubsystem;
    int initialBallCount = 0;
    boolean prevProxState;
    boolean curProxState;
    boolean endCondition;
    int count = 0;
    double targetRpm;
    long initialTime;
    long time = 0;
    int n = 0;

    public ShootCount (ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsytem, IntakeSubsystem intakeSubsystem, double targetRpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsytem = indexerSubsytem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetRpm = targetRpm;
        addRequirements(shooterSubsystem, indexerSubsytem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        prevProxState = indexerSubsytem.getProx2();
        if (!indexerSubsytem.getProx2()) { 
            n = 1;
        }
    }

    @Override
    public void execute() {
        shooterSubsystem.ledOn();
        shooterSubsystem.setHoodDistance();
        shooterSubsystem.shootDistance();

        if(Math.abs(shooterSubsystem.getActualRpm()) > Math.abs(targetRpm)) {
            indexerSubsytem.runIndexerFast();
            intakeSubsystem.runFunnel();
            intakeSubsystem.intake();
        }

        curProxState = indexerSubsytem.getProx2();
        if (curProxState != prevProxState) {
            count++;
        }
        prevProxState = curProxState;

        // System.out.println(count);
        // System.out.println("execute");
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.ledOff();
        shooterSubsystem.setHoodRest();
        shooterSubsystem.stopShooter();
        shooterSubsystem.resetTargetRpm();
        indexerSubsytem.stopIndexer();
        intakeSubsystem.stopFunnel();
        intakeSubsystem.stopIntake();
    }

    @Override
    public boolean isFinished() {
        System.out.println(count);
        if(count == indexerSubsytem.getBallCount() - n + 2 - 1) {
            initialTime = System.currentTimeMillis();
        }
        if(count >= indexerSubsytem.getBallCount() - n + 2) {
            time = System.currentTimeMillis();
        }
        return time - initialTime > 500;
    }
    
}
