package org.firstinspires.ftc.teamcode.teleOp.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Band extends SubsystemBase {
    //Motor declaration
    private DcMotorEx belt_Spin;

    public Band(HardwareMap hardwareMap) {
        belt_Spin = hardwareMap.get(DcMotorEx.class, "belt_Spin");
        belt_Spin.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void Voltage(double BandMotorVoltage) {
        belt_Spin.setPower(BandMotorVoltage);
    }
}
