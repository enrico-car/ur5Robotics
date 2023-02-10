package com.castle.castle_build_path;

import com.castle.castle_build_path.block.*;
import com.castle.castle_build_path.view.ButtonColumn;
import com.castle.castle_build_path.view.CubeBlock;
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
import javafx.scene.layout.GridPane;
import javafx.stage.Stage;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;


public class CastleBuildPath extends Application {

    static final int Cgrid = 10;
    static final int Rgrid = 10;
    static final int Hgrid = 5;
    CubeBlock cubeBlock = new CubeBlock(Cgrid, Rgrid, Hgrid);
    ButtonColumn buttonColumn = new ButtonColumn(cubeBlock);

    @Override
    public void start(Stage stage) throws IOException {

        BorderPane root = new BorderPane();
        root.setPadding(new Insets(10));
        root.setCenter(cubeBlock.getGrid());
        root.setLeft(buttonColumn);

        GridPane buttons = new GridPane();
        buttons.setPadding(new Insets(10));
        buttons.setHgap(5);
        Button button1 = new Button("up");
        Button button2 = new Button("down");

        button1.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                root.setCenter(cubeBlock.getUpperGrid());

            }
        });
        button2.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                root.setCenter(cubeBlock.getLowerGrid());
            }
        });
        buttons.add(button1,0,0);
        buttons.add(button2,1,0);

        root.setTop(buttons);


        Scene scene = new Scene(root, 800, 600);
        scene.setOnKeyPressed(keyHandler);
        stage.setTitle("Hello!");
        stage.setScene(scene);
        stage.show();
    }


    private EventHandler<KeyEvent> keyHandler = new EventHandler<KeyEvent>() {
        @Override
        public void handle(KeyEvent keyEvent) {
            Block block = buttonColumn.getBlock();
            if (keyEvent.getCode() == KeyCode.D && block.getPx() + block.getX() < Cgrid) {
                cubeBlock.removeBlock(block);
                block.right();
                cubeBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.A && block.getPx() > 0) {
                cubeBlock.removeBlock(block);
                block.left();
                cubeBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.W && block.getPy() > 0) {
                cubeBlock.removeBlock(block);
                block.up();
                cubeBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.S && block.getPy() + block.getY() < Rgrid) {
                cubeBlock.removeBlock(block);
                block.down();
                cubeBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.R) {
                cubeBlock.removeBlock(block);
                block.rotate();
                cubeBlock.addBlock(block);
            }
//            if (keyEvent.getCode() == KeyCode.Q) {
//                cubeBlock.savePosition(block);
//            }
        }
    };

    public static void main(String[] args) {
        launch();
    }
}