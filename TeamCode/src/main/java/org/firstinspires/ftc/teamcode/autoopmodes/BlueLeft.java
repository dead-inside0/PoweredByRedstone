package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class BlueLeft extends AutonomousOpMode {

    final double tileLength = 610;
    double[][] path = {
            {},
            {0, tile*2, 0,0,1,0,0},
            {-tile, tile*2, 0,0,1,0,0},
            {-tile*2, tile*2, 0,0,1,0,0},
            {-tile*3, tile*2, 0,0,1,0,0},
            {-tile*3.25, tile,0,0,0,-1,1},
            {-tile*3.25, tile*2,0,1,1,0},
            {-tile*4,tile*2,0,1,0,0}
    };

    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('b');}


    @Override
    public double[] getPlacementPosition(int elementLocation) {
        if(elementLocation == 0){
            return new double[]{0,tile*1.25,Math.PI,-1,1,0,1};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile*2,Math.PI * 0.5,-1,1,0,1};
        }
        else if(elementLocation == 2){
            return new double[]{0,tile*1.25,0,-1,1,0,1};
        }
        return new double[]{};
    }
}
