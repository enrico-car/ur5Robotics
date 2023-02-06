package com.castle.castle_build_path.view;

import com.castle.castle_build_path.block.Block;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Rectangle;
import javafx.util.Pair;

public class GridBlock extends GridPane {

    private int c, r;
    private Square gridRec[][];
    private static final Paint backColor = Color.rgb(61, 61, 84);

    public GridBlock(int c, int r) {
        super();
        this.c = c;
        this.r = r;
        this.setHgap(1);
        this.setVgap(1);
//        gridPane = new GridPane();
        gridRec = new Square[c][r];
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < r; j++) {
                Square square = new Square(backColor, false);
                gridRec[i][j] = square;
                this.add(square, i, j);
            }
        }
    }

    public void addBlock(Block block) {
        for (var x : block.getPosition()) {
            this.add(x.getSquare(), x.getPx(), x.getPy());
        }
    }

    public void removeBlock(Block block) {
        for (var x : block.getPosition()) {
            Square square = new Square(gridRec[x.getPx()][x.getPy()]);
            this.add(square, x.getPx(), x.getPy());
        }
    }

    public void savePosition(Block block) {
        if (isGoodPosition(block)) {
            for (var x : block.getPosition()) {
                gridRec[x.getPx()][x.getPy()] = x.getSquare();
                this.add(x.getSquare(), x.getPx(), x.getPy());
            }
        }
    }

    private boolean isGoodPosition(Block block) {
        for (var x : block.getPosition()) {
            if (!gridRec[x.getPx()][x.getPy()].getColor().equals(backColor)) {
                return false;
            }
        }
        return true;
    }

}
