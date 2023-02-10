package com.castle.castle_build_path.block;

import com.castle.castle_build_path.view.Square;
import com.castle.castle_build_path.view.SquarePos;
import javafx.scene.paint.Color;

import java.util.LinkedList;
import java.util.List;


public class X2_Y2_Z2_FILLET extends Block {
    public X2_Y2_Z2_FILLET(int Cgrid,int Rgrid) {
        super(Cgrid,Rgrid);
        this.x = 2;
        this.y = 2;
        this.z = 2;
        this.color = Color.rgb(255, 207, 2);
        this.underColor = Color.rgb(255, 207, 2,0.4);
    }

    @Override
    public List<SquarePos> getPosition() {
        List<SquarePos> list = new LinkedList<>();

        if (rotationState == RotationState.UP) {
            list.add(new SquarePos(px, py, new Square(color, true)));
            list.add(new SquarePos(px+1, py, new Square(color, true)));
            list.add(new SquarePos(px, py+1, new Square(color, false)));
            list.add(new SquarePos(px+1, py+1, new Square(color, false)));
        }
        else if (rotationState == RotationState.RIGHT){
            list.add(new SquarePos(px, py, new Square(color, false)));
            list.add(new SquarePos(px+1, py, new Square(color, true)));
            list.add(new SquarePos(px, py+1, new Square(color, false)));
            list.add(new SquarePos(px+1, py+1, new Square(color, true)));
        }
        else if (rotationState == RotationState.DOWN){
            list.add(new SquarePos(px, py, new Square(color, false)));
            list.add(new SquarePos(px+1, py, new Square(color, false)));
            list.add(new SquarePos(px, py+1, new Square(color, true)));
            list.add(new SquarePos(px+1, py+1, new Square(color, true)));
        }
        else if (rotationState == RotationState.LEFT){
            list.add(new SquarePos(px, py, new Square(color, true)));
            list.add(new SquarePos(px+1, py, new Square(color, false)));
            list.add(new SquarePos(px, py+1, new Square(color, true)));
            list.add(new SquarePos(px+1, py+1, new Square(color, false)));
        }

        return list;
    }

}
