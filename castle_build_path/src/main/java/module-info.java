module com.castle.castle_build_path {
    requires javafx.controls;
    requires javafx.fxml;
    requires json.simple;
    requires com.google.gson;


    opens com.castle.castle_build_path to javafx.fxml;
    exports com.castle.castle_build_path;
    exports com.castle.castle_build_path.view;
    opens com.castle.castle_build_path.view to javafx.fxml;
}