package com.castle.castle_build_path.view;

import com.castle.castle_build_path.tolls.JsonParser;
import javafx.event.EventHandler;
import javafx.scene.control.Button;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;


import com.castle.castle_build_path.block.Block;
import com.castle.castle_build_path.block.*;


public class ButtonColumn extends GridPane {

    Block[] blocks;
    CubeBlock cubeBlock;

    Button oldButton = null;

    int currentBlockIndex;

    Long[] counter;
    JsonParser jsonParser = new JsonParser();
    int dim = jsonParser.getDim();

    public ButtonColumn(CubeBlock cubeBlock) {

        this.setVgap(10);
        currentBlockIndex = -1;
        counter = jsonParser.getCounter();

        this.cubeBlock = cubeBlock;
        blocks = new Block[dim];

        int ind = 0;
        if (counter[0] != 0) {
            Button b0 = new Button("X1-Y1-Z2");
            for (int i = 0; i < counter[0]; i++) {
                blocks[ind] = new X1_Y1_Z2(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb0 = new PrettyButton(b0, counter[0], ind, 0);
            b0.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb0.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb0.current_ind - 1]);
                    oldButton = b0;
                    pb0.update_text();
                    if (pb0.numMax == 0) {
                        b0.setDisable(true);
                    }
                }
            });
            this.add(pb0, 0, 0);
        }

        if (counter[1] != 0) {
            Button b1 = new Button("X1-Y2-Z1");
            for (int i = 0; i < counter[1]; i++) {
                blocks[ind] = new X1_Y2_Z1(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb1 = new PrettyButton(b1, counter[1], ind, 1);
            b1.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb1.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb1.current_ind - 1]);
                    oldButton = b1;
                    pb1.update_text();
                    if (pb1.numMax == 0) {
                        b1.setDisable(true);
                    }
                }
            });
            this.add(pb1, 0, 1);
        }
        if (counter[2] != 0) {
            Button b2 = new Button("X1-Y2-Z2");
            for (int i = 0; i < counter[2]; i++) {
                blocks[ind] = new X1_Y2_Z2(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb2 = new PrettyButton(b2, counter[2], ind, 2);
            b2.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb2.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb2.current_ind - 1]);
                    oldButton = b2;
                    pb2.update_text();
                    if (pb2.numMax == 0) {
                        b2.setDisable(true);
                    }
                }
            });
            this.add(pb2, 0, 2);
        }

        if (counter[3] != 0) {
            Button b3 = new Button("X1-Y2-Z2-CHAMFER");
            for (int i = 0; i < counter[3]; i++) {
                blocks[ind] = new X1_Y2_Z2_CHAMPER(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb3 = new PrettyButton(b3, counter[3], ind, 3);
            b3.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb3.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb3.current_ind - 1]);
                    oldButton = b3;
                    pb3.update_text();
                    if (pb3.numMax == 0) {
                        b3.setDisable(true);
                    }
                }
            });
            this.add(pb3, 0, 3);
        }

        if (counter[4] != 0) {
            Button b4 = new Button("X1-Y2-Z2-TWINFILLET");
            for (int i = 0; i < counter[4]; i++) {
                blocks[ind] = new X1_Y2_Z2_TWINFILLET(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb4 = new PrettyButton(b4, counter[4], ind, 4);
            b4.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb4.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb4.current_ind - 1]);
                    oldButton = b4;
                    pb4.update_text();
                    if (pb4.numMax == 0) {
                        b4.setDisable(true);
                    }
                }
            });
            this.add(pb4, 0, 4);
        }
        if (counter[5] != 0) {
            Button b5 = new Button("X1-Y3-Z2");
            for (int i = 0; i < counter[5]; i++) {
                blocks[ind] = new X1_Y3_Z2(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb5 = new PrettyButton(b5, counter[5], ind, 5);
            b5.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb5.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb5.current_ind - 1]);
                    oldButton = b5;
                    pb5.update_text();
                    if (pb5.numMax == 0) {
                        b5.setDisable(true);
                    }
                }
            });
            this.add(pb5, 0, 5);
        }
        if (counter[6] != 0) {
            Button b6 = new Button("X1-Y3-Z2-FILLET");
            for (int i = 0; i < counter[6]; i++) {
                blocks[ind] = new X1_Y3_Z2_FILLET(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb6 = new PrettyButton(b6, counter[6], ind, 6);
            b6.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb6.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb6.current_ind - 1]);
                    oldButton = b6;
                    pb6.update_text();
                    if (pb6.numMax == 0) {
                        b6.setDisable(true);
                    }
                }
            });
            this.add(pb6, 0, 6);
        }
        if (counter[7] != 0) {
            Button b7 = new Button("X1-Y4-Z1");
            for (int i = 0; i < counter[7]; i++) {
                blocks[ind] = new X1_Y4_Z1(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb7 = new PrettyButton(b7, counter[7], ind, 7);
            b7.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb7.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb7.current_ind - 1]);
                    oldButton = b7;
                    pb7.update_text();
                    if (pb7.numMax == 0) {
                        b7.setDisable(true);
                    }
                }
            });
            this.add(pb7, 0, 7);
        }
        if (counter[8] != 0) {
            Button b8 = new Button("X1-Y4-Z2");
            for (int i = 0; i < counter[8]; i++) {
                blocks[ind] = new X1_Y4_Z2(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb8 = new PrettyButton(b8, counter[8], ind, 8);
            b8.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb8.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb8.current_ind - 1]);
                    oldButton = b8;
                    pb8.update_text();
                    if (pb8.numMax == 0) {
                        b8.setDisable(true);
                    }
                }
            });
            this.add(pb8, 0, 8);
        }
        if (counter[9] != 0) {
            Button b9 = new Button("X2-Y2-Z2");
            for (int i = 0; i < counter[9]; i++) {
                blocks[ind] = new X2_Y2_Z2(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb9 = new PrettyButton(b9, counter[9], ind, 9);
            b9.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb9.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb9.current_ind - 1]);
                    oldButton = b9;
                    pb9.update_text();
                    if (pb9.numMax == 0) {
                        b9.setDisable(true);
                    }
                }
            });
            this.add(pb9, 0, 9);
        }
        if (counter[10] != 0) {
            Button b10 = new Button("X2-Y2-Z2-FILLET");
            for (int i = 0; i < counter[10]; i++) {
                blocks[ind] = new X2_Y2_Z2_FILLET(cubeBlock.getC(), cubeBlock.getR());
                ind++;
            }
            PrettyButton pb10 = new PrettyButton(b10, counter[10], ind, 10);
            b10.setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    if (oldButton != null && currentBlockIndex != -1) {
                        //oldButton.setDisable(true);
                        cubeBlock.savePosition(blocks[currentBlockIndex]);
                    }
                    currentBlockIndex = pb10.current_ind - 1;
                    cubeBlock.addBlock(blocks[pb10.current_ind - 1]);
                    oldButton = b10;
                    pb10.update_text();
                    if (pb10.numMax == 0) {
                        b10.setDisable(true);
                    }
                }
            });
            this.add(pb10, 0, 10);
        }
    }

    public Block getCurrentBlock() {
        return blocks[currentBlockIndex];
    }

    public int getCurrentBlockIndex() {
        return currentBlockIndex;
    }

    public void resetCurrentBlockIndex(){
        currentBlockIndex = -1;
    }
}