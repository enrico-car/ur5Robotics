package com.castle.castle_build_path.view;

import com.castle.castle_build_path.block.Block;

public class CubeBlock {

    private int c, r, h;
    GridBlock[] gridBlocks;
    int gridSel;


    public CubeBlock(int c, int r, int h) {
        this.c = c;
        this.r = r;
        this.h = h;
        gridBlocks = new GridBlock[h];
        for (int i = 0; i < h; i++) {
            gridBlocks[i] = new GridBlock(c, r);
        }
        gridSel = 0;
    }

    public GridBlock getUpperGrid() {
        if (gridSel < h - 1) {
            gridSel++;
        }
        System.out.printf("sono a livello " + gridSel + "\n");
        return gridBlocks[gridSel];

    }

    public GridBlock getLowerGrid() {
        if (gridSel > 0) {
            gridSel--;
        }
        System.out.printf("sono a livello " + gridSel + "\n");
        return gridBlocks[gridSel];
    }

    public GridBlock getGrid() {
        return gridBlocks[gridSel];

    }

    public void addBlock(Block block) {
        if (block.getZ() == 1) {
            gridBlocks[gridSel].addBlock(block);
        }
        if (block.getZ() == 2) {
            gridBlocks[gridSel].addBlock(block);
            gridBlocks[gridSel + 1].addBlock(block);
        }
    }

    public void removeBlock(Block block) {
        if (block.getZ() == 1) {
            gridBlocks[gridSel].removeBlock(block);
        }
        if (block.getZ() == 2) {
            gridBlocks[gridSel].removeBlock(block);
            gridBlocks[gridSel + 1].removeBlock(block);
        }
    }

    public void savePosition(Block block)
    {
        if (block.getZ() == 1) {
            gridBlocks[gridSel].savePosition(block);
        }
        if (block.getZ() == 2) {
            gridBlocks[gridSel].savePosition(block);
            gridBlocks[gridSel + 1].savePosition(block);
        }
    }

    public int getC() {
        return c;
    }

    public int getR() {
        return r;
    }

    public int getH() {
        return h;
    }
}
