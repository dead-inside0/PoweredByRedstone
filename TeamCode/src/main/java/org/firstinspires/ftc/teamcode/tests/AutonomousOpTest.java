package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutonomousOpMode;
import org.opencv.core.Scalar;

@Autonomous(name="AutonomousOpTest", group="Tests")
public class AutonomousOpTest extends AutonomousOpMode {

    double[][] path = {
            {0, 400, 0},
            {400, 400, 0},
            {400, 0, 0},
            {0, 0, 0}
    };

    int linearExtensionIndex = 5;
    @Override
    public double[][] getPath() {
        return path;
    }

    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}

    @Override
    public int linearExtensionIndex() {return linearExtensionIndex;}

    @Override
    public double[] getPlacePosition(int elementLocation) {
        switch (elementLocation) {
            case 0:
                return new double[]{};
            case 2:
                return new double[]{};
            default:
                return new double[]{};
        }
    }
}
