package com.castle.castle_build_path.tolls;

import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;

import java.io.FileReader;

public class JsonParser {

    private Long[] counter=new Long[11];

    public JsonParser(){
        JSONParser parser = new JSONParser();
        try {
            Object obj = parser.parse(new FileReader("/home/annachiara/IdeaProjects/current.json")); //TO CHANGE
            JSONObject jsonObject = (JSONObject)obj;
            for(int i=0; i<11; i++){
                final int ind=i;
                String get_= String.valueOf(ind);
                long cl=(long)jsonObject.get(get_);
                counter[ind]=cl;
                System.out.println(counter[ind]);
            }
        } catch(Exception e) {
            e.printStackTrace();
        }
    }

    public Long[] getCounter() {
        return counter;
    }
}
