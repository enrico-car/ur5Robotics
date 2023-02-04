package com.castle.castle_build_path;

import com.castle.castle_build_path.block.*;
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

public class CastleBuildPath extends Application {

    private GridBlock gridBlock = new GridBlock(10, 10);
    private Block block = new X1_Y4_Z2();
    private Block block2 = new X1_Y4_Z2();

    @Override
    public void start(Stage stage) throws IOException {
        BorderPane root = new BorderPane();
        root.setPadding(new Insets(10));
        Button button = new Button("Blocco");
        Button button2 = new Button("Blocco2");
        root.setLeft(button);
        root.setTop(button2);



        root.setCenter(gridBlock);


        button.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                gridBlock.addBlock(block);
            }
        });
        button2.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                gridBlock.addBlock(block2);
            }
        });


        Scene scene = new Scene(root, 600, 600);
        scene.setOnKeyPressed(keyHandler);
        stage.setTitle("Hello!");
        stage.setScene(scene);
        stage.show();
    }


    private EventHandler<KeyEvent> keyHandler = new EventHandler<KeyEvent>() {
        @Override
        public void handle(KeyEvent keyEvent) {
            if (keyEvent.getCode() == KeyCode.D && block.getPx()+ block.getX() < gridBlock.getC()) {
                gridBlock.removeBlock(block);
                block.right();
                gridBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.A && block.getPx() > 0) {
                gridBlock.removeBlock(block);
                block.left();
                gridBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.W && block.getPy() > 0) {
                gridBlock.removeBlock(block);
                block.up();
                gridBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.S && block.getPy()+block.getY() < gridBlock.getR()) {
                gridBlock.removeBlock(block);
                block.down();
                gridBlock.addBlock(block);
            }
            if (keyEvent.getCode() == KeyCode.R) {
                // TODO add control to fix rotation problem
                gridBlock.removeBlock(block);
                block.rotate();
                gridBlock.addBlock(block);
            }
        }
    };

    public static void main(String[] args) {
        launch();
    }
}