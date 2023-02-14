package com.castle.castle_build_path.block;

import javafx.scene.paint.Color;
import javafx.util.Pair;


public class X1_Y4_Z1 extends Block {
    public X1_Y4_Z1(int Cgrid,int Rgrid) {
        super(Cgrid,Rgrid);
        this.x = 1;
        this.y = 4;
        this.z = 1;
        this.color = Color.rgb(118, 64, 215);
        this.underColor = Color.rgb(118, 64, 215,0.4);
    }

    @Override
    public Pair<Integer, Integer> getRealPosition() {
        if(rotationState == RotationState.UP || rotationState == RotationState.DOWN)
        {
            return new Pair<Integer,Integer>(unitLength*(Cgrid-px-1)+unitLength/2,unitLength*(py-1));
        }
        else
        {
            return new Pair<Integer,Integer>(unitLength*(Cgrid-px-2),unitLength*py+unitLength/2);
        }
    }


}
