package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class BlueRight extends AutonomousOpMode {
    public double[][] path = {
            {}
    };
    @Override
    public Scalar[] getColorBounds() {return colorByIndex('b');}
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public double[] getPlacementPosition(int elementLocation) {
        if(elementLocation == 0){
            return new double[]{0,tile*0.99,Math.PI *1.17,1,1,0,1};
        }
        else if(elementLocation == 1){
            return new double[]{0,tile * 1.1,Math.PI * 1.5,1,1,0,1};
        }
        else if(elementLocation == 2){
            return new double[]{70,tile * 0.9,Math.PI * 1.85,1,1,0,1};
        }
        return new double[]{};
    }
}
