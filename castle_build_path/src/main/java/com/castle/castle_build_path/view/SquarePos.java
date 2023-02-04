package com.castle.castle_build_path.view;

public class SquarePos {
    private int px, py;
    private Square square;

    public SquarePos(int px, int py, Square square) {
        this.px = px;
        this.py = py;
        this.square = square;
    }

    public int getPx() {
        return px;
    }

    public int getPy() {
        return py;
    }

    public Square getSquare() {
        return square;
    }

    public void setPx(int px) {
        this.px = px;
    }

    public void setPy(int py) {
        this.py = py;
    }

    public void setSquare(Square square) {
        this.square = square;
    }
}
