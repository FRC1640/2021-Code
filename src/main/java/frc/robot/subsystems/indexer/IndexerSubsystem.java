package frc.robot.subsystems.indexer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IndexerSubsystem extends SubsystemBase {
    
    private CANSparkMax indexerMotor = new CANSparkMax(5, MotorType.kBrushless);
    private DigitalInput prox1 = new DigitalInput(9);
    private DigitalInput prox2 = new DigitalInput(8);

    private int ballCount = 0;

    public void runIndexer() {
        indexerMotor.set(-0.4);
    }

    public void runIndexerFast() {
        indexerMotor.set(-0.7);
    }

    public void runIndexerExtraFast() {
        indexerMotor.set(-1.0);
    }

    public void reverseIndexer() {
        indexerMotor.set(1.0);
    }

    public void stopIndexer() {
        indexerMotor.set(0.0);
    }

    public boolean getProx1() {
        return prox1.get();
    }

    public boolean getProx2() {
        return prox2.get();
    }

    public void setBallCount(int ballCount) {
        this.ballCount = ballCount;
    }

    public int getBallCount() {
        return ballCount;
    }
    
}


