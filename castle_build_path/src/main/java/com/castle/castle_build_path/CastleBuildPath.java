package com.castle.castle_build_path;

import com.castle.castle_build_path.block.Block;
import com.castle.castle_build_path.view.ButtonColumn;
import com.castle.castle_build_path.view.CubeBlock;
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
import javafx.scene.text.Text;
import javafx.stage.Stage;

import java.io.IOException;


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
        buttons.setPadding(new Insets(20));
        buttons.setHgap(5);
        Button button1 = new Button("up");
        Button button2 = new Button("down");
        Text level = new Text("level 0");

        button1.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                if (buttonColumn.getCurrentBlockIndex() != -1) {
                    cubeBlock.savePosition(buttonColumn.getCurrentBlock());
                    buttonColumn.resetCurrentBlockIndex();
                }
                root.setCenter(cubeBlock.getUpperGrid());
                level.setText("level " + cubeBlock.getGridSel());
            }
        });
        button2.setOnMouseClicked(new EventHandler<MouseEvent>() {
            @Override
            public void handle(MouseEvent mouseEvent) {
                if (buttonColumn.getCurrentBlockIndex() != -1) {
                    cubeBlock.savePosition(buttonColumn.getCurrentBlock());
                    buttonColumn.resetCurrentBlockIndex();
                }
                root.setCenter(cubeBlock.getLowerGrid());
                level.setText("level " + cubeBlock.getGridSel());
            }
        });
        buttons.add(button1, 0, 0);
        buttons.add(button2, 1, 0);
        buttons.add(level, 2, 0);

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
            if (buttonColumn.getCurrentBlockIndex() != -1) {
                Block block = buttonColumn.getCurrentBlock();
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
            }
        }
    };

    public static void main(String[] args) {
        launch();
    }
}