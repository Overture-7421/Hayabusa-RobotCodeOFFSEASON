package org.firstinspires.ftc.teamcode.teleOp.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleOp.subsystems.Chassis;

public class MoveChassis extends CommandBase {
    private final Chassis chassis;
    private final Gamepad driverGamepad;

    public MoveChassis(Chassis chassis, Gamepad driverGamepad) {

    }

}
