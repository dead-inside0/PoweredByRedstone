package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class RedRight extends AutonomousOpMode {

    final double tile = 610;
    double[][] path = {
            {},
            /*{tile,tile*1.25,0,0,1,0,0},
            {tile*1.25, tile, 0,0,0,-1,1},
            {tile*1.25, 0, 0,0,1,1,0},
            {tile*2, 0, 0,0,1,1,0},*/
    };

    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}


    @Override
    public double[] getPlacementPosition(int elementLocation) {
        if(elementLocation == 0){
            return new double[]{0,tile*1.25,Math.PI,1,1,0,1};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile*1.25,Math.PI * 1.5,1,1,0,1};
        }
        else if(elementLocation == 2){
            return new double[]{100,100,Math.PI * 1.5,1,1,0,1};
        }
        return new double[]{};
    }
}
