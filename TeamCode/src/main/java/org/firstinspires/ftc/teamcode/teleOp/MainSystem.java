
/*
    __  __                  __                     ______          __
   / / / /___ ___  ______ _/ /_  __  ___________ _/ ____/___  ____/ /__
  / /_/ / __ `/ / / / __ `/ __ \/ / / / ___/ __ `/ /   / __ \/ __  / _ \
 / __  / /_/ / /_/ / /_/ / /_/ / /_/ (__  ) /_/ / /___/ /_/ / /_/ /  __/
/_/ /_/\__,_/\__, /\__,_/_.___/\__,_/____/\__,_/\____/\____/\__,_/\___/
            /____/

This is the code to control team Overture 23619's robot "Hayabusa".
Future iterations may change the overall functionality, though it is all used for the 2024 CENTERSTAGE FTC competition by FIRST.
All rights reserved. Copyright Overture 23619. Overture holds the right to modify and distribute this code.
*/

package org.firstinspires.ftc.teamcode.teleOp;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


// Commands Import
import org.firstinspires.ftc.teamcode.teleOp.commands.MoveChassis;

// Subsystems Import
import org.firstinspires.ftc.teamcode.teleOp.subsystems.Chassis;

@TeleOp
public class MainSystem extends LinearOpMode {

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().reset();

        Chassis chassis = new Chassis(hardwareMap);     //Create an instance of Chassis


        // -- CHASSIS MOVEMENT -- //
        chassis.setDefaultCommand(new MoveChassis(chassis, gamepad1));

        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();
            Pose2d pose = chassis.getPose();

            // -- ODOMETRY TELEMETRY -- //
            telemetry.addData("X", pose.getX());
            telemetry.addData("Y", pose.getY());
            telemetry.addData("Heading", pose.getRotation().getDegrees());

            telemetry.addData("RightDistance", chassis.rightDistance());
            telemetry.addData("LeftDistance", chassis.leftDistance());



            telemetry.update();
        }

    }



}
