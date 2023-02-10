package com.castle.castle_build_path.view;

import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;


import com.castle.castle_build_path.block.Block;
import com.castle.castle_build_path.block.*;

import java.util.ArrayList;
import java.util.List;

public class ButtonColumn extends GridPane {
    int dim = 2;
    Block[] blocks;
    CubeBlock cubeBlock;

    Button oldButton = null;

    int currentBlock;


    public ButtonColumn(CubeBlock cubeBlock) {
        this.cubeBlock = cubeBlock;
        blocks = new Block[dim];
        for (int i = 0; i < dim; i++) {
            Button button = new Button("ciao" + (i + 1));
            if (i == 0) {
                blocks[i] = new X1_Y2_Z1(cubeBlock.getC(), cubeBlock.getR());
            } else {
                blocks[i] = new X2_Y2_Z2(cubeBlock.getC(), cubeBlock.getR());
            }

            final int x = i;
            button.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if(oldButton != null)
                    {
                        oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlock]);
                    }
                    currentBlock = x;
                    cubeBlock.addBlock(blocks[x]);
                    oldButton = button;
                }
            });

            this.add(button, 0, i);
        }
    }

    public Block getBlock() {
        return blocks[currentBlock];
    }
}
