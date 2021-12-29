package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Limelight;
import frc.robot.Limelight.LedEnum;

public class ShooterSubsystem extends SubsystemBase {

    CANSparkMax motor1 = new CANSparkMax(14, MotorType.kBrushless);
    CANSparkMax motor2 = new CANSparkMax(3, MotorType.kBrushless);

    Servo servo = new Servo(4);

    Limelight limelight = new Limelight();

    double targetRpm = 0;

    public ShooterSubsystem() {
        motor2.follow(motor1, true);
        motor2.burnFlash();
    }

    public void setHoodDistance() {
        servo.set(calculateHoodAngle(limelight.getTargetY()));
    }

    public void setHoodFountain() {
        servo.set(0.75);
    }  

    public void setHoodRest() {
        servo.set(1.0);
    }

    public void setHoodClose() {
        servo.set(0.95);
    }

    public void shootDistance() {
        if (calculateDistanceFromTarget(limelight.getTargetY() + 21, 25.5, 98.6) < 250) {
            motor1.set(-0.8);
            targetRpm = -4000.0;
        } else if (calculateDistanceFromTarget(limelight.getTargetY() + 21, 25.5, 98.6) > 250
                && calculateDistanceFromTarget(limelight.getTargetY() + 21, 25.5, 98.6) < 400) {
            motor1.set(-0.95);
            targetRpm = -4400.0;
        } else if (calculateDistanceFromTarget(limelight.getTargetY() + 21, 25.5, 98.6) >= 400) {
            motor1.set(-1.0);
            targetRpm = -4800;
        }
    }

    public void shootClose() {
        targetRpm = -2000;
        motor1.set(-0.425);
    }

    public void shootFountain() {
        motor1.set(-0.25);
    }

    public void stopShooter() {
        motor1.set(0.0);
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public double getActualRpm() {
        return motor1.getEncoder().getVelocity();
    }

    public void resetTargetRpm() {
        targetRpm = 0.0;
    }

    public boolean atTargetRpm() {
        return Math.abs(getTargetRpm()) <= Math.abs(getActualRpm()) && getTargetRpm() != 0.0;
    }

    public boolean isFastIndexer() {
        limelight.setLEDOn(LedEnum.FORCE_ON);
        if (calculateDistanceFromTarget(limelight.getTargetY() + 21, 25.5, 98.6) < 250) {
            return true;
        } else {
            return false;
        }
    }

    public void ledOn() {
        limelight.setLEDOn(LedEnum.FORCE_ON);
        limelight.setProcessing(true);
    }

    public void ledOff() {
        limelight.setLEDOn(LedEnum.FORCE_OFF);
        limelight.setProcessing(false);
    }

    public static float calculateDistanceFromTarget(double angleDegrees, double distanceToFloorInches,
            double heightOfTargetInches) {
        return (float) ((heightOfTargetInches - distanceToFloorInches) / (Math.tan(Math.toRadians(angleDegrees))));
    }

    public static double calculateHoodAngle(double ty) {
        double distance = calculateDistanceFromTarget(ty + 21, 25.5, 98.6);
        // System.out.println(distance);
        return -0.0002*distance + 0.61;
    }

}
