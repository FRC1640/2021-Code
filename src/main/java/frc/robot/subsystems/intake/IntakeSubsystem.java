package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {

    private CANSparkMax intakeMotor = new CANSparkMax(10, MotorType.kBrushless); 
	private CANSparkMax funnelMotor = new CANSparkMax(9, MotorType.kBrushless); //Brushless on prime, brushed on deux

	private Solenoid intakeSolenoid = new Solenoid(1);

    public void deployIntake() {
        intakeSolenoid.set(true);
    }

    public void retractIntake() {
        intakeSolenoid.set(false);
    }

    public void intake() {
        intakeMotor.set(-1.0);
    }

    public void outtake() {
        intakeMotor.set(1.0);
    }

    public void runFunnel() {
        funnelMotor.set(-1.0);
    }

    public void reverseFunnel() {
        funnelMotor.set(1.0);
    }
    
    public void stopIntake() {
        intakeMotor.set(0.0);
    }

    public void stopFunnel() {
        funnelMotor.set(0.0);
    }
    
}
