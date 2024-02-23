package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class RedRight extends AutonomousOpMode {
    double[][] path = {
            {0, 85, 0},
            {tile, tile, Math.PI/2},
            {tile, 0, Math.PI/2}
    };

    int linearExtensionIndex = 1;
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}
}