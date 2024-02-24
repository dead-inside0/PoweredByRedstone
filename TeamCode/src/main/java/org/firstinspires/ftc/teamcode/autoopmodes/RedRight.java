package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous
public class RedRight extends BlueRight {
    public double[][] path = {
            {},
            {0,100,Math.PI,0,1,1,0},
            {tile*2,0,0,0,1,1,0},
    };
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}
}
