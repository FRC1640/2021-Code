package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;

public class Proximity {

    private DigitalInput proximitySensor;

    public Proximity(int port) {
        proximitySensor = new DigitalInput(port);
    }

    public boolean get() {
        return !proximitySensor.get();
    }
    
}
