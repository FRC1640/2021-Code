package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class ShootOnStart extends CommandBase {

    ShooterSubsystem shooterSubsystem;
    IndexerSubsystem indexerSubsytem;
    IntakeSubsystem intakeSubsystem;
    int initialBallCount = 3;
    boolean prevProxState;
    boolean curProxState;
    boolean endCondition;
    int count = 0;
    double targetRpm;
    long initialTime;
    long startTime;
    long time = 0;

    public ShootOnStart (ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsytem, IntakeSubsystem intakeSubsystem, double targetRpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsytem = indexerSubsytem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetRpm = targetRpm;
        addRequirements(shooterSubsystem, indexerSubsytem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        prevProxState = indexerSubsytem.getProx2();
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
        // System.out.println("end");
    }

    @Override
    public boolean isFinished() {
        if(count == 5) {
            initialTime = System.currentTimeMillis();
        }
        if(count >= 6) {
            time = System.currentTimeMillis();
        }

        long time2 = System.currentTimeMillis();

        if(time - initialTime > 500) {
            return true;
        }

        if(time2 - startTime >= 5000) {
            return true;
        }

        // System.out.println(time - initialTime);
        return false;
    }
    
}
