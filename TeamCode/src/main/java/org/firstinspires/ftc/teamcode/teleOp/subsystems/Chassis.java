package org.firstinspires.ftc.teamcode.teleOp.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;
import com.arcrobotics.ftclib.kinematics.wpilibkinematics.DifferentialDriveOdometry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class Chassis extends SubsystemBase {
    //Motor declaration
    private DcMotorEx left_Drive;
    private DcMotorEx right_Drive;

    private final double M_PER_TICK = 1.0 / 54000.0 * 9.0 * Math.PI;
    static final double TRACKWIDTH = 1; //space that wheels take

    private DifferentialDriveOdometry diffOdom;

    private IMU imu;

    private int leftOffset = 0, rightOffset = 0;

    public Chassis(HardwareMap hardwareMap) {
        //Motor ID
        left_Drive = (DcMotorEx) hardwareMap.get(DcMotor.class, "left_Drive");
        right_Drive = (DcMotorEx) hardwareMap.get(DcMotor.class, "right_Drive");

        //Invert left motor
        left_Drive.setDirection(DcMotor.Direction.REVERSE);
        right_Drive.setDirection(DcMotor.Direction.FORWARD);

        //Initialize odometry and IMU
        diffOdom = new DifferentialDriveOdometry(new Rotation2d());
        imu = hardwareMap.get(IMU.class, "imu");

        //Where the IMU is facing and where the USB port is located
        IMU.Parameters imuParameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP)
        );

        //Initialize and reset imu
        imu.initialize(imuParameters);
        imu.resetYaw();

    }

    //Set speed function. Let MoveChassis set the values itself
    public void setSpeed(double linearSpeed, double angularSpeed) {
        left_Drive.setPower(linearSpeed - angularSpeed);
        right_Drive.setPower(linearSpeed + angularSpeed);
    }

    public double leftDistance() {
        return (left_Drive.getCurrentPosition() - leftOffset) * M_PER_TICK;
    }

    public double rightDistance() {
        return (left_Drive.getCurrentPosition() - leftOffset) * M_PER_TICK;
    }




}


