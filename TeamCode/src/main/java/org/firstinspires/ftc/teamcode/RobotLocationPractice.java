package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class RobotLocationPractice extends OpMode {
    double angle;

    //we need a getter so another class can get angle
    //constructor, called whenever the class is initialized. usually has the same name as the class
    public RobotLocationPractice(double angle) {
        this.angle = angle;
    }

    public double getHeading(){ //normalize angle, ie between -180 and 180
        double angle = this.angle; //copy angle, usually off imu
        while (angle>180){
            angle-=360;
        }
        while (angle <= -180){
            angle+=360;
        }
        return angle; //return normalized value

    }

    public void setAngle(double angle) {
        this.angle = angle;
    }
}
