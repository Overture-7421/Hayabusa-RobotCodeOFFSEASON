package org.firstinspires.ftc.teamcode.utils;

import static java.lang.Math.abs;
import static java.lang.Math.pow;

public class JoystickHandler {
    public static final double DEADZONE = 0.15; //Set gamepad joysticks' deadzone treshold
    public static final double EXPONENTIAL_GAIN = 0.15; //Set exponential gain

    public static double handleJosytickInput(double axisValue) {
        double axisMag = abs(axisValue);
        if (axisMag < DEADZONE) return 0.0;

        double res = EXPONENTIAL_GAIN * pow
                ((axisMag - DEADZONE) / (1-DEADZONE), 3) +
                (1 - EXPONENTIAL_GAIN) * (axisMag - DEADZONE) / (1 - DEADZONE);
        return Math.copySign(res, axisValue);
    }
}
