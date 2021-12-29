package frc.robot.utilities;

import edu.wpi.first.wpilibj.Timer;

public class TimeUtil {

    private static Timer timer;

    static {
        timer = new Timer();
        timer.start();
    }

    public static double getTime () {
        return timer.get();
    }
    
}
