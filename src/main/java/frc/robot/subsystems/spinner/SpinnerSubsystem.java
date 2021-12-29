package frc.robot.subsystems.spinner;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SpinnerSubsystem extends SubsystemBase {

    private CANSparkMax spinnerMotor = new CANSparkMax(7, MotorType.kBrushless);
	private Solenoid spinnerPiston = new Solenoid(0);

	private final I2C.Port i2cPort = I2C.Port.kOnboard;
	private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
	private Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
	private Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
	private Color kRedTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
	private Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);
	private ColorMatch colorMatcher = new ColorMatch();
	private String gameData;

	public SpinnerSubsystem() {
		colorMatcher.addColorMatch(kBlueTarget);
		colorMatcher.addColorMatch(kGreenTarget);
		colorMatcher.addColorMatch(kRedTarget);
		colorMatcher.addColorMatch(kYellowTarget);
	}

	public void setSpinnerSpeed(double speed) {
		spinnerMotor.set(speed);
	}

	public void deploySpinner() {
		spinnerPiston.set(true);
	}

	public void retractSpinner() {
		spinnerPiston.set(false);
	}

	public char getFMSColor() {
		gameData = DriverStation.getInstance().getGameSpecificMessage();
		if (gameData.length() > 0) {
			return gameData.charAt(0);
		} else {
			return 'F';
		}
	}

	public char getColor() {
		Color detectedColor = colorSensor.getColor();
		String colorString;
		ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

		colorMatcher.addColorMatch(kBlueTarget);
		colorMatcher.addColorMatch(kGreenTarget);
		colorMatcher.addColorMatch(kRedTarget);
		colorMatcher.addColorMatch(kYellowTarget);

		if (match.color == kBlueTarget) {
			colorString = "Blue";
			return 'B';
		} else if (match.color == kRedTarget) {
			colorString = "Red";
			return 'R';
		} else if (match.color == kGreenTarget) {
			colorString = "Green";
			return 'G';
		} else if (match.color == kYellowTarget) {
			colorString = "Yellow";
			return 'Y';
		} else {
			colorString = "Unknown";
			return 'F';
		}
	}
    
}
