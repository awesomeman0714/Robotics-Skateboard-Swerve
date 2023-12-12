package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;

public class Devices {
    public static Joystick driveStick;
    public static Joystick rotateStick;

    public static AHRS rawNavX;

    public static void init() {
        driveStick = new Joystick(0);
        rotateStick = new Joystick(1);

        rawNavX = new AHRS();

        rawNavX.setAngleAdjustment(180);
    }

    public static double swerveXRate() {
        return qDeadband(-1*driveStick.getRawAxis(1), 0.1);
    }

    public static double swerveYRate() {
        return qDeadband(-1*driveStick.getRawAxis(0),0.1);
    }

    public static double swerveRotationRate() {
    return -1*deadband(rotateStick.getRawAxis(0),0.1);
    }

    public static double getYawRadians(){
        return rawNavX.getAngle() * (Math.PI / 180);
    }

    public static double deadband(double inputSpeed, double dband) {
        
        if (Math.abs(inputSpeed)<=dband)
        {
            return 0;
        }
        else 
        {
            return Math.signum(inputSpeed)*(Math.abs(inputSpeed)-dband)/(1-dband);
        }
    }

    public static double qDeadband(double inputSpeed, double dband)
    {
        
    if (Math.abs(inputSpeed)<=dband)
    {
        return 0;
    }
    else 
    {
        return Math.signum(inputSpeed)*(Math.abs(inputSpeed)-dband)*(Math.abs(inputSpeed)-dband)/((1-dband)*(1-dband));
    }
    }


}