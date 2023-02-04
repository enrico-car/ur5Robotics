package com.castle.castle_build_path;

import com.castle.castle_build_path.block.*;
import com.castle.castle_build_path.view.ButtonColumn;
import com.castle.castle_build_path.view.GridBlock;
import javafx.application.Application;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;

public class CastleBuildPath extends Application {

    public static GridBlock gridBlock;
    public static Block block;
    ArrayList<Block> bl=new ArrayList<Block>(Arrays.asList(new X1_Y1_Z2(), new X2_Y2_Z2()));
    ButtonColumn button_cl=new ButtonColumn(bl);
    //public Block handled= button_cl.getBlock();
    //private Block block2 = new X1_Y4_Z2();


    @Override
    public void start(Stage stage) throws IOException {


        gridBlock=button_cl.campo;
        BorderPane root = new BorderPane();
        root.setPadding(new Insets(10));
        root.setLeft(button_cl.button_col);

        root.setCenter(gridBlock);

        Scene scene = new Scene(root, 800, 600);
        scene.setOnKeyPressed(keyHandler);
        stage.setTitle("Hello!");
        stage.setScene(scene);
        stage.show();
    }


    public EventHandler<KeyEvent> keyHandler = new EventHandler<KeyEvent>() {
        @Override
        public void handle(KeyEvent keyEvent) {
            if (keyEvent.getCode() == KeyCode.D && bl.get(ind).getPx()+ bl.get(ind).getX() < gridBlock.getC()) {
                //ind=button_cl.getIndex();
                System.out.println(ind);
                gridBlock.removeBlock(bl.get(ind));
                bl.get(ind).right();
                gridBlock.addBlock(bl.get(ind));
            }
            if (keyEvent.getCode() == KeyCode.A && bl.get(ind).getPx() > 0) {
                gridBlock.removeBlock(bl.get(ind));
                bl.get(ind).left();
                gridBlock.addBlock(bl.get(ind));
            }
            if (keyEvent.getCode() == KeyCode.W && bl.get(ind).getPy() > 0) {
                gridBlock.removeBlock(bl.get(ind));
                bl.get(ind).up();
                gridBlock.addBlock(bl.get(ind));
            }
            if (keyEvent.getCode() == KeyCode.S && bl.get(ind).getPy()+bl.get(ind).getY() < gridBlock.getR()) {
                gridBlock.removeBlock(bl.get(ind));
                bl.get(ind).down();
                gridBlock.addBlock(bl.get(ind));
            }
            if (keyEvent.getCode() == KeyCode.R) {
                // TODO add control to fix rotation problem
                gridBlock.removeBlock(bl.get(ind));
                bl.get(ind).rotate();
                gridBlock.addBlock(bl.get(ind));
            }
        }
    };

    public static void main(String[] args) {
        launch();
    }
}