package org.firstinspires.ftc.teamcode;


import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;


import java.util.Calendar;
import java.util.List;
@TeleOp
public class MainSystem extends LinearOpMode {
    public static class NestedClass{
        public int a, b;
    }

    public static int[] array = new int[3];
    public static NestedClass[] innerArray =
            new NestedClass[] {new NestedClass(), new NestedClass(), new NestedClass()};

    @Override
    public void runOpMode() {
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        //      telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("array0", array[0]);
            telemetry.addData("array1", array[1]);
            telemetry.addData("innerArray0A", innerArray[0].a);
            telemetry.addData("innerArray1B", innerArray[0].b);
            telemetry.addData("innerArray2A", innerArray[0].a);
            telemetry.update();
        }


    }
}
