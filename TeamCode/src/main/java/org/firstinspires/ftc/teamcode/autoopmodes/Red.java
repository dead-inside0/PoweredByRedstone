package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class Red extends AutonomousOpMode {
    public double[][] path = {
            {}
    };
    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}

    @Override
    public double[][] getPath() {
        return path;
    }


    @Override
    public double[] getPlacementPosition(int elementLocation) {
        if(elementLocation == 0){
            return new double[]{50,tile*1.25,Math.PI *0.9,1,1,0,5};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile * 1.25,Math.PI * 1.5 * 0.9,1,1,0,5};
        }
        else if(elementLocation == 2){
            return new double[]{50,tile*1.25,0,1,1,0,5};
        }
        return new double[]{};
    }
}
