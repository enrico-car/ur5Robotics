package com.castle.castle_build_path.view;

import javafx.beans.value.ObservableStringValue;
import javafx.scene.control.Button;
import javafx.scene.layout.GridPane;
import javafx.scene.text.Text;

public class PrettyButton extends GridPane {
    int current_ind;
    int numMax; //fisso
    int ind_block;
    Button button;
    Text text;

    public PrettyButton(Button b, long numMax, int ind, int ib){
        this.button=b;
        this.numMax=Math.toIntExact(numMax);
        this.current_ind= ind;
        text=new Text(String.valueOf(numMax));
        ind_block=ib;
        this.setHgap(10);
        this.add(button, 1,ib);
        this.add(text, 0,ib);
    }

    void update_text(){
        current_ind--;
        numMax--;
        text.setText(String.valueOf(numMax));
    }





}
