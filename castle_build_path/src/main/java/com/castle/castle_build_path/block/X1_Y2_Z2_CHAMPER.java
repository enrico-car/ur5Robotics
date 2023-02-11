package com.castle.castle_build_path.block;

import com.castle.castle_build_path.view.Square;
import com.castle.castle_build_path.view.SquarePos;
import javafx.scene.paint.Color;

import java.util.LinkedList;
import java.util.List;


public class X1_Y2_Z2_CHAMPER extends Block {
    public X1_Y2_Z2_CHAMPER(int Cgrid,int Rgrid) {
        super(Cgrid,Rgrid);
        this.x = 1;
        this.y = 2;
        this.z = 2;
        this.color = Color.rgb(255, 90, 90);
        this.underColor = Color.rgb(255, 90, 90,0.4);
    }

    @Override
    public List<SquarePos> getPosition() {
        List<SquarePos> list = new LinkedList<>();

        if (rotationState == RotationState.UP) {
            list.add(new SquarePos(px, py, new Square(color, true)));
            list.add(new SquarePos(px, py+1, new Square(color, false)));
        }
        else if (rotationState == RotationState.RIGHT){
            list.add(new SquarePos(px, py, new Square(color, false)));
            list.add(new SquarePos(px+1, py, new Square(color, true)));
        }
        else if (rotationState == RotationState.DOWN){
            list.add(new SquarePos(px, py, new Square(color, false)));
            list.add(new SquarePos(px, py+1, new Square(color, true)));
        }
        else if (rotationState == RotationState.LEFT){
            list.add(new SquarePos(px, py, new Square(color, true)));
            list.add(new SquarePos(px+1, py, new Square(color, false)));
        }

        return list;
    }

    @Override
    public List<SquarePos> getPositionBack() {
        List<SquarePos> list = new LinkedList<>();

        if (rotationState == RotationState.UP) {
            list.add(new SquarePos(px, py, new Square(underColor, true)));
            list.add(new SquarePos(px, py+1, new Square(underColor, false)));
        }
        else if (rotationState == RotationState.RIGHT){
            list.add(new SquarePos(px, py, new Square(underColor, false)));
            list.add(new SquarePos(px+1, py, new Square(underColor, true)));
        }
        else if (rotationState == RotationState.DOWN){
            list.add(new SquarePos(px, py, new Square(underColor, false)));
            list.add(new SquarePos(px, py+1, new Square(underColor, true)));
        }
        else if (rotationState == RotationState.LEFT){
            list.add(new SquarePos(px, py, new Square(underColor, true)));
            list.add(new SquarePos(px+1, py, new Square(underColor, false)));
        }

        return list;
    }

}
