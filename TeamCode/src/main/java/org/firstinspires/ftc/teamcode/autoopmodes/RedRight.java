package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous
public class RedRight extends AutonomousOpMode {

    final double tileLength = 600;
    double[][] path = {
            {0, 85, 0},
            {tileLength, tileLength, Math.PI/2},
            {tileLength, 0, Math.PI/2}
    };

    int linearExtensionIndex = 1;
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}

    @Override
    public int linearExtensionIndex() {return linearExtensionIndex;}
}