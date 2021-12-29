package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.indexer.IndexerSubsystem;
import frc.robot.subsystems.intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;

public class Shoot5Fast extends CommandBase {

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
    long time = 0;

    public Shoot5Fast (ShooterSubsystem shooterSubsystem, IndexerSubsystem indexerSubsytem, IntakeSubsystem intakeSubsystem, double targetRpm) {
        this.shooterSubsystem = shooterSubsystem;
        this.indexerSubsytem = indexerSubsytem;
        this.intakeSubsystem = intakeSubsystem;
        this.targetRpm = targetRpm;
        addRequirements(shooterSubsystem, indexerSubsytem, intakeSubsystem);
    }
    
    @Override
    public void initialize() {
        prevProxState = indexerSubsytem.getProx2();
    }

    @Override
    public void execute() {
        shooterSubsystem.ledOn();
        shooterSubsystem.setHoodDistance();
        shooterSubsystem.shootDistance();

        if(Math.abs(shooterSubsystem.getActualRpm()) > Math.abs(targetRpm)) {
            indexerSubsytem.runIndexerExtraFast();
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
        if(count == 8) {
            initialTime = System.currentTimeMillis();
        }
        if(count >= 9) {
            time = System.currentTimeMillis();
        }

        // System.out.println(time - initialTime);
        return time - initialTime > 500;
    }
    
}
