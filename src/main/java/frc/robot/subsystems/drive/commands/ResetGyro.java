package frc.robot.subsystems.drive.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utilities.Logger;

public class ResetGyro extends CommandBase {

    private DriveSubsystem driveSubsystem;

    public ResetGyro (DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        addRequirements(driveSubsystem);
    }
    
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        driveSubsystem.resetGyro();
        Logger.log("Gyro reset!");
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }

}
