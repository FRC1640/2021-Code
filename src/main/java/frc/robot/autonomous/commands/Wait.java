package frc.robot.autonomous.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class Wait extends CommandBase {

    long duration;
    long initTime;
    long currentTime;

    public Wait(long duration) {
        this.duration = duration;
    }

    @Override
    public void initialize() {
        initTime = System.currentTimeMillis();
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        currentTime = System.currentTimeMillis();
        return currentTime - initTime >= duration;
    }
    
}
