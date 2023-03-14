package com.castle.castle_build_path.tolls;

import com.castle.castle_build_path.view.CubeBlock;
import com.google.gson.JsonElement;
import com.google.gson.JsonParser;
import javafx.util.Pair;
import org.json.simple.JSONObject;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import org.json.simple.parser.JSONParser;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Map;

public class WriteJson {
    private static FileWriter file;

    public static void writeJson(CubeBlock cubeBlock) {
        JSONObject jsonObject = new JSONObject();
        int i=1;
        for (var grid : cubeBlock.getGridBlocks()) {
            for (var block : grid.getSavedBlocks()) {
                JSONObject position = new JSONObject();
                Pair<Integer, Integer> pair = block.getRealPosition();
                position.put("x", pair.getKey());
                position.put("y", pair.getValue());
                position.put("z", block.getRealHeight());
                position.put("r", block.getRotation());
                position.put("class",block.getClass().getSimpleName());
                jsonObject.put(i,position);
                i++;
            }
        }
        jsonObject.put("size",i-1);
        System.out.println(jsonObject.toJSONString());
        try (PrintWriter out = new PrintWriter(new FileWriter("/home/carro/ur5Robotics/castle_build_path/src/main/java/com/castle/castle_build_path/tolls/output.json"))) {
            Gson gson = new GsonBuilder().setPrettyPrinting().create();
            JsonElement je = JsonParser.parseString(jsonObject.toJSONString());
            out.write(gson.toJson(je));
        } catch (Exception e) {
            e.printStackTrace();
        }

    }
}
