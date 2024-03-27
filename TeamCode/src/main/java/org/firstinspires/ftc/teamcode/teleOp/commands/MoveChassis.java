package org.firstinspires.ftc.teamcode.teleOp.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.teleOp.subsystems.Chassis;
import org.firstinspires.ftc.teamcode.utils.JoystickHandler;

public class MoveChassis extends CommandBase {
    private final Chassis chassis;
    private final Gamepad driverGamepad;

    public MoveChassis(Chassis subsystem, Gamepad driverGamepad) {
        this.driverGamepad = driverGamepad;
        chassis = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        double right = -driverGamepad.right_stick_x;
        double left = -driverGamepad.left_stick_y;

        right = JoystickHandler.handleJosytickInput(right);
        left = JoystickHandler.handleJosytickInput(left);

        chassis.setSpeed(left, right);
    }

}
