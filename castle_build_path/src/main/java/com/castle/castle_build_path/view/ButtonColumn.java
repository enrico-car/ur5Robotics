package com.castle.castle_build_path.view;

import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.control.Button;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;

import static com.castle.castle_build_path.CastleBuildPath.gridBlock;
import static com.castle.castle_build_path.CastleBuildPath.block;
import com.castle.castle_build_path.block.*;

public class ButtonColumn extends GridPane {

    public GridPane button_col=new GridPane();
    private Button cl_0=new Button("X1-Y1-Z2");
    private Button cl_1=new Button("X1-Y2-Z1");
    private Button cl_2=new Button("X1-Y2-Z2");
    private Button cl_3=new Button("X1-Y2-Z2-CHAMFER");
    private Button cl_4=new Button("X1-Y2-Z2-TWINFILLET");
    private Button cl_5=new Button("X1-Y3-Z2");
    private Button cl_6=new Button("X1-Y3-Z2-FILLET");
    private Button cl_7=new Button("X1-Y4-Z1");
    private Button cl_8=new Button("X1-Y4-Z2");
    private Button cl_9=new Button("X2-Y2-Z2");
    private Button cl_10=new Button("X2-Y2-Z2-FILLET");
    public ButtonColumn(){
        this.button_col.add(cl_0, 0,0);
        this.button_col.add(cl_1, 0,1);
        this.button_col.add(cl_2, 0,2);
        this.button_col.add(cl_3, 0,3);
        this.button_col.add(cl_4, 0,4);
        this.button_col.add(cl_5, 0,5);
        this.button_col.add(cl_6, 0,6);
        this.button_col.add(cl_7, 0,7);
        this.button_col.add(cl_8, 0,8);
        this.button_col.add(cl_9, 0,9);
        this.button_col.add(cl_10, 0,10);
        this.button_col.setAlignment(Pos.CENTER);

        cl_0.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y1_Z2();
                gridBlock.addBlock(block);
                cl_0.setDisable(true);
            }
        });

        cl_1.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block= new X1_Y2_Z1();
                gridBlock.addBlock(block);
                cl_1.setDisable(true);
            }
        });

        cl_2.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y2_Z2();
                gridBlock.addBlock(block);
                cl_2.setDisable(true);
            }
        });

        /* CLASS NOT CREATED YET
        cl_3.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y2_Z2_CHAMPFER();
                gridBlock.addBlock(block);
                cl_3.setDisable(true);
            }
        });*/

        /* CLASS NOT CREATED YET
        cl_4.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y2_Z2_TWINFILLET();
                gridBlock.addBlock(block);
                cl_4.setDisable(true);
            }
        });*/

        cl_5.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y3_Z2();
                gridBlock.addBlock(block);
                cl_5.setDisable(true);
            }
        });

        /* CLASS NOT CREATED YET
        cl_6.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y3_Z2_FILLET();
                gridBlock.addBlock(block);
                cl_6.setDisable(true);
            }
        });*/

        cl_7.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y4_Z1();
                gridBlock.addBlock(block);
                cl_7.setDisable(true);
            }
        });

        cl_8.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X1_Y4_Z2();
                gridBlock.addBlock(block);
                cl_8.setDisable(true);
            }
        });

        cl_9.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X2_Y2_Z2();
                gridBlock.addBlock(block);
                cl_9.setDisable(true);
            }
        });

        /*CLASS NOT CREATED YET
        cl_10.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                block=new X2_Y2_Z2_FILLET();
                gridBlock.addBlock(block);
                cl_10.setDisable(true);
            }
        });*/


    }






}
