package com.castle.castle_build_path.block;

import javafx.scene.paint.Color;
import javafx.util.Pair;

public class X1_Y1_Z2 extends Block {
    public X1_Y1_Z2(int Cgrid,int Rgrid) {
        super(Cgrid, Rgrid);
        this.x = 1;
        this.y = 1;
        this.z = 2;
        this.color = Color.rgb(7, 210, 190);
        this.underColor = Color.rgb(7, 210, 190,0.4);
    }

    @Override
    public Pair<Integer, Integer> getRealPosition() {
        return new Pair<Integer,Integer>(unitLength*(Cgrid-px-1)+unitLength/2,unitLength*py+unitLength/2);
    }

}
