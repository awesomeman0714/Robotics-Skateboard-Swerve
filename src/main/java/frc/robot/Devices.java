package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Joystick;

public class Devices {
    public static Joystick driveStick;
    public static Joystick rotateStick;

    public static AHRS rawNavX;

    public void init() {
        driveStick = new Joystick(0);
        rotateStick = new Joystick(1);
    }

    public static double getStickX() {
        return driveStick.getX();
    }

    public static double getStickY() {
        return driveStick.getY();
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


}
