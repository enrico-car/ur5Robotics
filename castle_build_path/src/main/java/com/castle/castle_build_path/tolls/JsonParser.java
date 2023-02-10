package com.castle.castle_build_path.tolls;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.File;
import java.io.FileReader;

public class JsonParser {

    private Long[] counter=new Long[11];
    private int dim;

    public JsonParser(){
        JSONParser parser = new JSONParser();
        dim=0;
        try {
            Object obj = parser.parse(new FileReader("/home/rico/Documents/castle_build_path/src/main/java/com/castle/castle_build_path/tolls/current.json")); //TO CHANGE
            JSONObject jsonObject = (JSONObject)obj;
            for(int i=0; i<11; i++){
                final int ind=i;
                String get_= String.valueOf(ind);
                long cl=(long)jsonObject.get(get_);
                counter[ind]=cl;
                dim+=counter[ind];
                //System.out.println(counter[ind]);
            }
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    public Long[] getCounter() {
        return counter;
    }

    public int getDim() {return dim;}
}