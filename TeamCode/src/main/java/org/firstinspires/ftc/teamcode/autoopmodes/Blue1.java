package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class Blue1 extends AutonomousOpMode {

    final double tileLength = 600;
    double[][] path = {
            {0, 85, 0},
            {-tileLength*2, 85, 0},
            {-tileLength*3, tileLength, Math.PI},
            {-tileLength*3, 85,Math.PI}
    };

    int linearExtensionIndex = 2;
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('b');}

    @Override
    public int linearExtensionIndex() {return linearExtensionIndex;}
}
