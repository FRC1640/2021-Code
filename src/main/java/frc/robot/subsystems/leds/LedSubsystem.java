package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.leds.commands.SabotageText;
import frc.robot.subsystems.leds.commands.SetLeds;

public class LedSubsystem  extends SubsystemBase {

    AddressableLED ledStrip = new AddressableLED(5);
    AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(300);

    public LedSubsystem() {
        ledStrip.setLength(ledBuffer.getLength());
        ledStrip.setData(ledBuffer);
        ledStrip.start();
    }

    public void initDefaultCommand() {
        setDefaultCommand(new SetLeds(this));
      }

    public void setLed(int led, int r, int g, int b) {
        ledBuffer.setRGB(led, r, g, b);
        ledStrip.setData(ledBuffer);
    }
    
}
