package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Scalar;

@Autonomous
public class RedLeft extends BlueRight {
    @Override
    public Scalar[] getColorBounds() {return colorByIndex('r');}
}
