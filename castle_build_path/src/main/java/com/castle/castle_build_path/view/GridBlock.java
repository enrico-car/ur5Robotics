package com.castle.castle_build_path.view;

import com.castle.castle_build_path.block.Block;
import javafx.geometry.Insets;
import javafx.scene.layout.GridPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;

public class GridBlock extends GridPane {

    private int c, r;
    private Square gridRec[][];
    private Boolean blockPlaced[][];
    private static final Paint backColor = Color.rgb(61, 61, 84);

    public GridBlock(int c, int r) {
        super();
        this.c = c;
        this.r = r;
        this.setHgap(1);
        this.setVgap(1);
        this.setPadding(new Insets(0,0,0,20));
        blockPlaced = new Boolean[c][r];

//        gridPane = new GridPane();
        gridRec = new Square[c][r];
        for (int i = 0; i < c; i++) {
            for (int j = 0; j < r; j++) {
                Square square = new Square(backColor, false);
                gridRec[i][j] = square;
                this.add(square, i, j);
                blockPlaced[i][j] = false;
            }
        }
    }

    public void addBlock(Block block) {
        for (var x : block.getPosition()) {
            this.add(x.getSquare(), x.getPx(), x.getPy());
        }
    }

    public void addBlockBack(Block block) {
        for (var x : block.getPositionBack()) {
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

    public void savePositionBack(Block block) {
        if (isGoodPosition(block)) {
            for (var x : block.getPositionBack()) {
                gridRec[x.getPx()][x.getPy()] = x.getSquare();
                this.add(x.getSquare(), x.getPx(), x.getPy());
                blockPlaced[x.getPx()][x.getPy()] = true;
            }
        }
    }

    private boolean isGoodPosition(Block block) {
        for (var x : block.getPosition()) {
            if (blockPlaced[x.getPx()][x.getPy()]) {
                return false;
            }
        }
        return true;
    }

}
