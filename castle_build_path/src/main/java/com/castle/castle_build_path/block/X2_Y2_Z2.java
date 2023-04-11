package com.castle.castle_build_path.block;

import javafx.scene.paint.Color;
import javafx.util.Pair;

import java.util.LinkedList;
import java.util.List;

public class X2_Y2_Z2 extends Block {
    public X2_Y2_Z2(int Cgrid, int Rgrid) {
        super(Cgrid, Rgrid);
        this.x = 2;
        this.y = 2;
        this.z = 2;
        this.color = Color.rgb(255, 162, 9);
        this.underColor = Color.rgb(255, 162, 9, 0.4);
    }

    @Override
    public Pair<Integer, Integer> getRealPosition() {
        return new Pair<Integer, Integer>(unitLength * (Cgrid - px - 1), unitLength * py + unitLength);
    }

}
