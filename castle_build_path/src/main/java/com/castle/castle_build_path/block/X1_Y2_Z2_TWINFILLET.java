package com.castle.castle_build_path.block;

import com.castle.castle_build_path.view.Square;
import com.castle.castle_build_path.view.SquarePos;
import javafx.scene.paint.Color;

import java.util.LinkedList;
import java.util.List;


public class X1_Y2_Z2_TWINFILLET extends Block {
    public X1_Y2_Z2_TWINFILLET(int Cgrid, int Rgrid) {
        super(Cgrid,Rgrid);
        this.x = 1;
        this.y = 2;
        this.z = 2;
        this.color = Color.rgb(14, 180, 90);
        this.underColor = Color.rgb(14, 180, 90,0.4);
    }

    @Override
    public List<SquarePos> getPosition() {
        List<SquarePos> list = new LinkedList<>();

        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                list.add(new SquarePos(px + i, py + j, new Square(color, false)));
            }
        }

        return list;
    }

}
