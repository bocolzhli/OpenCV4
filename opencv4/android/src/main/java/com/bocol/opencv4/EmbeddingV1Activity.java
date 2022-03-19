package com.bocol.opencv4;
import android.os.Bundle;
import io.flutter.app.FlutterActivity;

public class EmbeddingV1Activity extends FlutterActivity {
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        OpenCV4Plugin.registerWith(registrarFor("com.bocol.opencv4.OpenCV4Plugin"));
    }

}