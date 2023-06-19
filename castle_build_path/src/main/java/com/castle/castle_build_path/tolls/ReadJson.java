package com.castle.castle_build_path.tolls;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.FileReader;

public class ReadJson {

    private Long[] counter=new Long[11];
    private int dim;

    public ReadJson(){
        JSONParser parser = new JSONParser();
        dim=0;
        try {
            Object obj = parser.parse(new FileReader("current.json")); //TO CHANGE
            JSONObject jsonObject = (JSONObject)obj;
            for(int i=0; i<11; i++){
                final int ind=i;
                String get_= String.valueOf(ind);
                long cl=(long)jsonObject.get(get_);
                counter[ind]=cl;
                dim+=counter[ind];
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
