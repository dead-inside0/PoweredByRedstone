package org.firstinspires.ftc.teamcode.autoopmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueLeft extends BlueRight {
    public double[][] path = {
            {},
            {0,0,0,0,1,1,0},
            {-tile*2,0,0,0,1,1,0},
    };
    @Override
    public double[][] getPath() {
        return path;
    }
}
