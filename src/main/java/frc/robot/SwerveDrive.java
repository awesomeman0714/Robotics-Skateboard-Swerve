package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveDrive {
    private static final int driveFRcanID = 18;
    private static final int driveFLcanID = 12;
    private static final int driveBRcanID = 16;
    private static final int driveBLcanID = 14;

    private static final int steerFRcanID = 11;
    private static final int steerFLcanID = 13;
    private static final int steerBRcanID = 17;
    private static final int steerBLcanID = 15;

    public static final double fullScaleXSpeed=4.5;  //The speed that corresponds to joystick 1;
    public static final double fullScaleYSpeed=4.5;  //The ground speeds are in meters/second;
    public static final double fullScaleRotationRate=2*3.14; //Max rotation rate in radians/second

    public static SwerveModule frontRight;
    public static SwerveModule frontLeft;
    public static SwerveModule backRight;
    public static SwerveModule backLeft;

    public static void init() {
        frontRight = new SwerveModule(driveFRcanID, steerFRcanID, false, true);
        frontLeft = new SwerveModule(driveFLcanID, steerFLcanID, false, false);
        backRight = new SwerveModule(driveBRcanID, steerBRcanID, false, true);
        backLeft = new SwerveModule(driveBLcanID, steerBLcanID, false, false);
    }

    public static void drive(){

           //The stick values will give the fraction of full scale
        double requestedXStick = Devices.swerveXRate();
        double requestedYStick = Devices.swerveYRate();
        double requestedRotationStick = Devices.swerveRotationRate();
           
        SmartDashboard.putNumber("Drive Stick X", requestedXStick);
        SmartDashboard.putNumber("Drive Stick Y", requestedYStick);
   
     /*    double requestedXVelocity=requestedXStick*fullScaleXSpeed;
        double requestedYVelocity = requestedYStick*fullScaleYSpeed;*/

        double requestedXVelocity = pedalToMedal(requestedXStick);
        double requestedYVelocity = pedalToMedal(requestedYStick);

        SmartDashboard.putNumber("X Velocity", requestedXVelocity);
        SmartDashboard.putNumber("Y Velocity", requestedYVelocity);

        double requestedRotationRate = requestedRotationStick * fullScaleRotationRate;
        double yawRadians = Devices.getYawRadians();

        SmartDashboard.putNumber("NavX Degree", yawRadians * (180/Math.PI));

        driveByNumbersFieldCentric(requestedXVelocity, requestedYVelocity, requestedRotationRate,yawRadians);
           
    }

    public static double pedalToMedal(double stickval)
    {
        if (Math.abs(stickval)<0.95)
        {
           return  stickval*fullScaleXSpeed;
        }
        else
        {
            return 2*fullScaleXSpeed*Math.signum(stickval);
        }
    }

    public static void driveByNumbersFieldCentric(double x, double y, double rot, double yaw)
    {

        
        SwerveModuleState[] requiredStates = computeModuleStatesFieldCentric(x,y,rot,yaw);
        //0, 1, 2, 3 = FRONTLEFT, FRONTRIGHT, BACKLEFT, BACKRIGHT

        //The SwerveModuleState returned from computeModuleStates is going to be in meters per second, but we could translate units
        //from there.
        
        frontLeft.setModuleState(requiredStates[0].speedMetersPerSecond, requiredStates[0].angle.getRadians());
        frontRight.setModuleState(requiredStates[1].speedMetersPerSecond, requiredStates[1].angle.getRadians());
        backLeft.setModuleState(requiredStates[2].speedMetersPerSecond, requiredStates[2].angle.getRadians());
        backRight.setModuleState(requiredStates[3].speedMetersPerSecond, requiredStates[3].angle.getRadians());
    }

    public static SwerveModuleState[] computeModuleStatesFieldCentric(double xSpeed, double ySpeed, double rotationRate, double yaw)
    {
        SwerveModuleState[] ret; //Students:  Note that "new" has not been called.  This is a declaration.
  
        //Using this now would result in a null pointer.
        Rotation2d rotationObject = Rotation2d.fromRadians(yaw);

        ChassisSpeeds desiredMotion =  ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed,ySpeed,rotationRate,rotationObject);
        ret = Kinematics.getToModuleStates(desiredMotion); //Still no "new" for ret....but it was called inside the function, so it's ok

        return ret;   
    }
}
