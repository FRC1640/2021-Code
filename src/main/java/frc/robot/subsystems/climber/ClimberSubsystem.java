package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax liftMotor = new CANSparkMax(13, MotorType.kBrushless);
    private Solenoid dogSolenoid = new Solenoid(4);
    private DoubleSolenoid deploySolenoid = new DoubleSolenoid(2, 3);

    public void setPistons(boolean state) {
        Value vState = (state) ? Value.kForward : Value.kReverse;
		deploySolenoid.set(vState);    
    }

    public void setDog(boolean state) {
        dogSolenoid.set(state);
    }

    public void setSpeed(double speed) {
        liftMotor.set(speed);
    }
    
}
