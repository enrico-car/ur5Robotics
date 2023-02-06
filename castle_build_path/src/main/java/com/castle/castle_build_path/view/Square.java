package com.castle.castle_build_path.view;

import javafx.geometry.Pos;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.scene.paint.Paint;
import javafx.scene.shape.Circle;
import javafx.scene.shape.Rectangle;

public class Square extends StackPane {

    // TODO twinfillet problem
    private static final int dim = 50;
    Rectangle rectangle;
    Circle circle;
    Paint color;
    Boolean hasCircle;

    public Square(Paint color, Boolean hasCircle) {
        super();
        this.color = color;
        this.rectangle = new Rectangle(dim, dim, color);
        this.hasCircle = hasCircle;
        this.getChildren().add(rectangle);
        this.setAlignment(Pos.CENTER);
        if (hasCircle) {
            circle = new Circle(15);
            circle.setStroke(Color.BLACK);
            circle.setStrokeWidth(2.0);
            circle.setFill(color);
            this.getChildren().add(circle);
        }
    }

    public Square(Square square) {      // Copy constructor
        super();
        this.rectangle = new Rectangle(dim, dim, square.rectangle.getFill());
        this.hasCircle = square.hasCircle;
        this.circle = square.circle;
        this.getChildren().add(rectangle);
        this.setAlignment(Pos.CENTER);
        if (hasCircle) {
            circle = new Circle(15);
            circle.setStroke(Color.BLACK);
            circle.setStrokeWidth(2.0);
            circle.setFill(square.rectangle.getFill());
            this.getChildren().add(circle);
        }
    }

    public Rectangle getRectangle() {
        return rectangle;
    }

    public Boolean getHasCircle() {
        return hasCircle;
    }

    public Paint getColor() {
        return color;
    }
}
