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

    public GridPane button_col=new GridPane();
    public GridBlock campo=new GridBlock(10,10);
    private int dim=2;
    private ArrayList<Button> bottoni=new ArrayList<Button>(dim);
    public int ind=-1;
    public Integer ind_to_pass=-1;

    public ButtonColumn(ArrayList<Block> el){

        for(int i=0; i<dim; i++){
            ind=i;
            String cl_name= String.valueOf(el.get(i).getClass());
            Class cl= el.get(i).getClass();
            Button b=new Button(cl_name);
            bottoni.add(b);
            this.button_col.add(b, 0,i);

            bottoni.get(i).setOnMouseClicked(new EventHandler<MouseEvent>() {
                @Override
                public void handle(MouseEvent mouseEvent) {
                    campo.addBlock(el.get(ind));
                    bottoni.get(ind).setDisable(true);
                }
            });
        }

        this.button_col.setAlignment(Pos.CENTER);

    }

    public Integer getIndex() {
        return ind_to_pass;
    }

    public GridBlock getCampo() {
        return campo;
    }
}
