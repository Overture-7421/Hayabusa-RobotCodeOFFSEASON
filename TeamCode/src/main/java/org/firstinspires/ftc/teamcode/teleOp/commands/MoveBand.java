package org.firstinspires.ftc.teamcode.teleOp.commands;


import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.teleOp.subsystems.Band;



public class MoveBand extends CommandBase {
    private Band band;
    private double voltage;

    public MoveBand(Band subsystem, double voltage) {
        this.voltage =voltage;
        band = subsystem;
        addRequirements(band);
    }
    @Override
    public void initialize(){
        band.Voltage(voltage);
    }
    @Override
    public void end(boolean interrupted){
        band.Voltage(0);
    }
}
