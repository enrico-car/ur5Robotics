package com.castle.castle_build_path.block;

import com.castle.castle_build_path.view.Square;
import com.castle.castle_build_path.view.SquarePos;
import javafx.scene.paint.Color;
import javafx.util.Pair;

import java.util.LinkedList;
import java.util.List;

enum RotationState {
    UP,
    LEFT,
    DOWN,
    RIGHT
}

public abstract class Block {
    final static int unitLength = 3175 + 100; // +100 to leave space between block
    final static int unitHeight = 1875 + 100; // +100 to leave space between block

    int x, y, z;
    int px, py, pz;
    int Cgrid, Rgrid;

    RotationState rotationState;

    Color color;

    Color underColor;

    public Block(int Cgrid, int Rgrid) {
        px = 0;
        py = 0;
        pz = 0;
        this.Cgrid = Cgrid;
        this.Rgrid = Rgrid;
        rotationState = RotationState.UP;
    }

    public void rotate() {
        if (px + y <= Cgrid && py + x <= Rgrid)  // check boundary of grid
        {
            int temp = x;
            x = y;
            y = temp;
            if (rotationState == RotationState.UP) {
                rotationState = RotationState.LEFT;
            } else if (rotationState == RotationState.LEFT) {
                rotationState = RotationState.DOWN;
            } else if (rotationState == RotationState.DOWN) {
                rotationState = RotationState.RIGHT;
            } else {
                rotationState = RotationState.UP;
            }
        }
    }

    public List<SquarePos> getPosition() {
        List<SquarePos> list = new LinkedList<>();

        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                list.add(new SquarePos(px + i, py + j, new Square(color, true)));
            }
        }

        return list;
    }

    public List<SquarePos> getPositionBack() {
        List<SquarePos> list = new LinkedList<>();

        for (int i = 0; i < x; i++) {
            for (int j = 0; j < y; j++) {
                list.add(new SquarePos(px + i, py + j, new Square(underColor, true)));
            }
        }

        return list;
    }

    public abstract Pair<Integer, Integer> getRealPosition();

    public int getRealHeight() {
        return unitHeight * pz;
    }

    public int getX() {
        return x;
    }

    public int getY() {
        return y;
    }

    public int getZ() {
        return z;
    }

    public int getPx() {
        return px;
    }

    public int getPy() {
        return py;
    }

    public int getPz() {
        return pz;
    }

    public void setX(int x) {
        this.x = x;
    }

    public void setY(int y) {
        this.y = y;
    }

    public void setZ(int z) {
        this.z = z;
    }

    public void setPx(int px) {
        this.px = px;
    }

    public void setPy(int py) {
        this.py = py;
    }

    public void setPz(int pz) {
        this.pz = pz;
    }

    public Color getColor() {
        return color;
    }

    public void right() {
        px++;
    }

    public void left() {
        px--;
    }

    public void up() {
        py--;
    }

    public void down() {
        py++;
    }

    public int getRotation() {
        return rotationState.ordinal();
    }
}
