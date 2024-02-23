package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class AutonomousTest extends AutonomousOpMode {
    final double tileLength = 610;
    double[][] path = {
            {}
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
            return new double[]{0,tile/2,Math.PI,-1,1,0,1};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile/2,Math.PI * 1.5,-1,1,0,1};
        }
        else if(elementLocation == 2){
            return new double[]{0,tile/2,0,-1,1,0,1};
        }
        return new double[]{};
    }
}
