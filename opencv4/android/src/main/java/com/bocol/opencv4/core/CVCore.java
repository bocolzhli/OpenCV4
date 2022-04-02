package com.bocol.opencv4.core;

import android.annotation.SuppressLint;

import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.core.MatOfByte;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint2f;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.lang.Math;
import java.util.Random;

import androidx.annotation.NonNull;
import io.flutter.Log;
import io.flutter.embedding.engine.plugins.FlutterPlugin;
import io.flutter.plugin.common.MethodCall;
import io.flutter.plugin.common.MethodChannel;
import io.flutter.plugin.common.MethodChannel.MethodCallHandler;
import io.flutter.plugin.common.MethodChannel.Result;
import io.flutter.plugin.common.PluginRegistry.Registrar;

/**
 * OpenCV4Plugin
 */
public class CVCore {

    @SuppressLint("MissingPermission")
    public byte[] cvtColor(byte[] byteData, int outputType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Convert the image color
            Imgproc.cvtColor(src, dst, outputType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] blur(byte[] byteData, ArrayList kernelSize, ArrayList anchorPoint, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(
                    new MatOfByte(byteData),
//                    Imgcodecs.IMREAD_UNCHANGED
                    Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH
            );

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            Point point = new Point((double) anchorPoint.get(0), (double) anchorPoint.get(1));
            // Convert the image to Gray
            Imgproc.blur(src, dst, size, point, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] gaussianBlur(byte[] byteData, ArrayList kernelSize, double sigmaX) {
        byte[] byteArray = new byte[0];
        try {
            gaussianBlur(byteData, kernelSize, sigmaX, 0.0);
        }catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }
    @SuppressLint("MissingPermission")
    public byte[] gaussianBlur(byte[] byteData, ArrayList kernelSize, double sigmaX, double sigmaY) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            // Convert the image to Gray
            Imgproc.GaussianBlur(src, dst, size, sigmaX, sigmaY);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] medianBlur(byte[] byteData, int kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Convert the image to Gray
            Imgproc.medianBlur(src, dst, kernelSize);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] bilateralFilter(byte[] byteData, int diameter, int sigmaColor, int sigmaSpace, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Convert the image to Gray
            Imgproc.bilateralFilter(src, dst, diameter, sigmaColor, sigmaSpace, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] boxFilter(byte[] byteData, int outputDepth, ArrayList kernelSize, ArrayList anchorPoint,
                            boolean normalize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));
            Point point = new Point((double) anchorPoint.get(0), (double) anchorPoint.get(1));

            // Convert the image to Gray
            Imgproc.boxFilter(src, dst, outputDepth, size, point, normalize, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] sqrBoxFilter(byte[] byteData, int outputDepth, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            Size size = new Size((double) kernelSize.get(0), (double) kernelSize.get(1));

            // Convert the image to Gray
            Imgproc.sqrBoxFilter(src, dst, outputDepth, size);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] filter2D(byte[] byteData, int outputDepth, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Creating kernel matrix
            Mat kernel = Mat.ones((int) kernelSize.get(0), (int) kernelSize.get(1), CvType.CV_32F);

            for (int i = 0; i < kernel.rows(); i++) {
                for (int j = 0; j < kernel.cols(); j++) {
                    double[] m = kernel.get(i, j);

                    for (int k = 1; k < m.length; k++) {
                        m[k] = m[k] / (2 * 2);
                    }
                    kernel.put(i, j, m);
                }
            }
            // Convert the image to Gray
            Imgproc.filter2D(src, dst, outputDepth, kernel);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] filter2DV2(byte[] byteData, int outputDepth, ArrayList kernelSize, ArrayList kernelData, ArrayList anchor, double delta) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Creating kernel matrix
            Mat kernel = Mat.ones((int) kernelSize.get(0), (int) kernelSize.get(1), CvType.CV_32F);
            for (int i = 0; i < kernel.rows(); i++) {
                for (int j = 0; j < kernel.cols(); j++) {
                    double m = (double)kernelData.get(i*kernel.rows()+j);
                    kernel.put(i, j, m);
                }
            }

            Point _anchor = new Point((double)anchor.get(0), (double)anchor.get(1));
            Imgproc.filter2D(src, dst, outputDepth, kernel, _anchor, delta);

            /*
            double minR=300.0, minG=300.0, minB=300.0, maxR=0.0, maxG=0.0, maxB=0.0;
            for(int i = 0; i<dst.rows(); i++) {
                for(int j = 0; j<dst.cols(); j++) {
                    double[] rgb = dst.get(i, j);

                    if(minR > rgb[0]) { minR = rgb[0];}
                    if(maxR < rgb[0]) { maxR = rgb[0];}
                    if(minG > rgb[1]) { minG = rgb[1];}
                    if(maxG < rgb[1]) { maxG = rgb[1];}
                    if(minB > rgb[2]) { minB = rgb[2];}
                    if(maxB < rgb[2]) { maxB = rgb[2];}
                }
            }
            for(int i = 0; i<dst.rows(); i++) {
                for(int j = 0; j<dst.cols(); j++) {
                    double[] rgb = dst.get(i, j);

                    rgb[0] = 255.0 * (rgb[0] - minR) / (maxR - minR);
                    rgb[1] = 255.0 + 128.0 * (rgb[1] - minG) / (maxG - minG);
                    rgb[2] = 255.0 + 128.0 * (rgb[2] - minB) / (maxB - minB);

                    int Gray = (((int)rgb[0])*38 + ((int)rgb[0]*75) + ((int)rgb[0]*15)) >> 7;
                    rgb[0] = rgb[1] = rgb[2] = Gray;

                    dst.put(i, j, rgb);
                }
            }
            System.out.println("min max is [" + minR + "," + maxR + "],[" + minG + "," + maxG + "],[" + minB +"," + maxB + "]");
            */

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] dilate(byte[] byteData, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Preparing the kernel matrix object
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new Size(((int) kernelSize.get(0) * (int) kernelSize.get(1)) + 1,
                            ((int) kernelSize.get(0) * (int) kernelSize.get(1)) + 1));

            // Convert the image to Gray
            Imgproc.dilate(src, dst, kernel);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] erode(byte[] byteData, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Preparing the kernel matrix object
            Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT,
                    new Size(((int) kernelSize.get(0) * (int) kernelSize.get(1)) + 1,
                            ((int) kernelSize.get(0) * (int) kernelSize.get(1)) + 1));

            // Convert the image to Gray
            Imgproc.erode(src, dst, kernel);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] morphologyEx(byte[] byteData, int operation, ArrayList kernelSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Creating kernel matrix
            Mat kernel = Mat.ones((int) kernelSize.get(0), (int) kernelSize.get(0), CvType.CV_32F);

            // Morphological operation
            Imgproc.morphologyEx(src, dst, operation, kernel);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrUp(byte[] byteData, ArrayList kernelSize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Size of the new image
            Size size = new Size((int) kernelSize.get(0), (int) kernelSize.get(1));

            // pyrUp operation
            Imgproc.pyrUp(src, dst, size, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrDown(byte[] byteData, ArrayList kernelSize, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Size of the new image
            Size size = new Size((int) kernelSize.get(0), (int) kernelSize.get(1));

            // pyrDown operation
            Imgproc.pyrDown(src, dst, size, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] pyrMeanShiftFiltering(byte[] byteData, double spatialWindowRadius, double colorWindowRadius) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // pyrMeanShiftFiltering operation
            Imgproc.pyrMeanShiftFiltering(src, dst, spatialWindowRadius, colorWindowRadius);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] threshold(byte[] byteData, double thresholdValue, double maxThresholdValue, int thresholdType) {
        byte[] byteArray = new byte[0];
        try {
            Mat srcGray = new Mat();
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            // Convert the image to Gray
            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);

            // Thresholding
            Imgproc.threshold(srcGray, dst, thresholdValue, maxThresholdValue, thresholdType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] adaptiveThreshold(byte[] byteData, double maxValue, int adaptiveMethod, int thresholdType,
                                    int blockSize, double constantValue) {
        byte[] byteArray = new byte[0];
        try {
            Mat srcGray = new Mat();
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Convert the image to Gray
            Imgproc.cvtColor(src, srcGray, Imgproc.COLOR_BGR2GRAY);

            // Adaptive Thresholding
            Imgproc.adaptiveThreshold(srcGray, dst, maxValue, adaptiveMethod, thresholdType, blockSize, constantValue);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] copyMakeBorder(byte[] byteData, int top, int bottom, int left, int right, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // copyMakeBorder operation
            Core.copyMakeBorder(src, dst, top, bottom, left, right, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] sobel(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Sobel operation
            Imgproc.Sobel(src, dst, depth, dx, dy);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] scharr(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Scharr operation
            Imgproc.Scharr(src, dst, depth, dx, dy);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }
    @SuppressLint("MissingPermission")
    public byte[] scharrV2(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            Mat dst0 = new Mat();
            src.convertTo(dst0, CvType.CV_32F);//Mat to be accumulateWeighted must be of type CV_32F

            // Scharr operation
            Mat dst1 = new Mat();
            Imgproc.Scharr(dst0, dst1, depth, dx, 0);

            Mat dst2 = new Mat();
            Imgproc.Scharr(dst0, dst2, depth, 0, dy);

            Imgproc.accumulateWeighted(dst1, dst2, 0.5);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst2, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }
    public byte[] scharrV3(byte[] byteData, int depth, int dx, int dy) {
        byte[] byteArray = new byte[0];
        try {
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            //gray styled
            Mat grayMat = new Mat();
            Imgproc.cvtColor(src, grayMat, Imgproc.COLOR_RGB2GRAY);


            //blur
            Mat blurMat = new Mat();
            Imgproc.blur(grayMat, blurMat, new Size(3, 3));
//            Imgproc.GaussianBlur(grayMat, blurMat, new Size(0.0, 0.0), 5.0, 0.0);
            Mat cannyMat = new Mat();
//            Imgproc.Canny(src, cannyMat, 100.0, 300.0, 3);
            Imgproc.Canny(blurMat, cannyMat, 100, 300);


            // Scharr operation
//            Mat CV32FMat = new Mat();
//            grayMat.convertTo(CV32FMat, CvType.CV_32F);//Mat to be accumulateWeighted must be of type CV_32F
//            Mat scharrMat1 = new Mat();
//            Imgproc.Scharr(CV32FMat, scharrMat1, depth, dx, 0);
//            Mat scharrMat2 = new Mat();
//            Imgproc.Scharr(CV32FMat, scharrMat2, depth, 0, dy);
//            Imgproc.accumulateWeighted(scharrMat1, scharrMat2, 0.5);

            //find contours
            List<MatOfPoint> contours = new ArrayList<MatOfPoint>();
            Mat hierarchy = new Mat();
            Mat contourMap = new Mat();
            cannyMat.convertTo(contourMap, CvType.CV_8UC1);
            Imgproc.findContours(contourMap, contours, hierarchy, dx/*Imgproc.RETR_CCOMP*/, dy/*Imgproc.CHAIN_APPROX_NONE*/);
            System.out.println("contour size is " + contours.size());
            Mat drawMat = new Mat();
            src.convertTo(drawMat, CvType.CV_32FC3);

            ArrayList<Integer> indices = new ArrayList<Integer>(contours.size());
            System.out.println("hierarchy is " + hierarchy.toString());
            for(int i = 0; i < contours.size(); i++) {
                double area = Imgproc.contourArea(contours.get(i));
                if(area>1.0) {
                    indices.add(i);

                    System.out.println("hierarchy " + i + " is [" + hierarchy.get(0, i)[0]
                            + "," + hierarchy.get(0, i)[1] + "," + hierarchy.get(0, i)[2]
                            + "," + hierarchy.get(0, i)[3] + "]");
                    System.out.println("Contour area is [" + area + "]");
                }
//                if(true || Math.floor(hierarchy.get(0, i)[0]) == -1) {
//                    indices.add(i);
//                }
            }
            for(int i = 0; i < indices.size(); i++) {
                Random random = new Random(Calendar.getInstance().getTimeInMillis());
                Imgproc.drawContours(drawMat, contours, indices.get(i), new Scalar(random.nextInt(255)
                        , random.nextInt(255), random.nextInt(255), 0.0), 2
                , -1, hierarchy, 0);
            }

            Mat outImage = drawMat;
            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", outImage, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] laplacian(byte[] byteData, int depth) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Laplacian operation
            Imgproc.Laplacian(src, dst, depth);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] laplacianV2(byte[] byteData, int depth, int ksize, double scale, double delta, int borderType) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Laplacian operation
            Imgproc.Laplacian(src, dst, depth, ksize, scale, delta, borderType);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] distanceTransform(byte[] byteData, int distanceType, int maskSize) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // distanceTransform operation
            Imgproc.distanceTransform(src, dst, distanceType, maskSize);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] resize(byte[] byteData, ArrayList outputSize, double fx, double fy, int interpolation) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // Size of the new image
            Size size = new Size((int) outputSize.get(0), (int) outputSize.get(1));

            // resize operation
            Imgproc.resize(src, dst, size, fx, fy, interpolation);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] applyColorMap(byte[] byteData, int colorMap) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // resize operation
            Imgproc.applyColorMap(src, dst, colorMap);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] canny(byte[] byteData, double threshold1, double threshold2) {
        byte[] byteArray = new byte[0];
        try {
            Mat dst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            // resize operation
            Imgproc.Canny(src, dst, threshold1, threshold2);

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", dst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] houghLines(byte[] byteData, double rho, double theta, int threshold, double srn, double stn,
                             double minTheta, double maxTheta, String lineColor, int lineThickness, int lineType, int shift) {
        byte[] byteArray = new byte[0];
        try {
            Mat cdst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            // Check if image is loaded fine
            // Copy edges to the images that will display the results in BGR
            Imgproc.cvtColor(src, cdst, Imgproc.COLOR_GRAY2BGR);

            // Standard Hough Line Transform
            Mat lines = new Mat(); // will hold the results of the detection
            Imgproc.HoughLines(src, lines, rho, theta, threshold, srn, stn, minTheta, maxTheta); // runs the actual
            // detection
            System.out.println("lines: " + lines);
            // Draw the lines
            for (int x = 0; x < lines.rows(); x++) {
                double rho2 = lines.get(x, 0)[0], theta2 = lines.get(x, 0)[1];
                double a = Math.cos(theta2), b = Math.sin(theta2);
                double x0 = a * rho2, y0 = b * rho2;
                Point pt1 = new Point(Math.round(x0 + 1000 * (-b)), Math.round(y0 + 1000 * (a)));
                Point pt2 = new Point(Math.round(x0 - 1000 * (-b)), Math.round(y0 - 1000 * (a)));
                Imgproc.line(cdst, pt1, pt2, convertColorToScalar(lineColor), lineThickness, lineType, shift);
            }
            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", cdst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] houghLinesProbabilistic(byte[] byteData, double rho, double theta, int threshold,
                                          double minLineLength, double maxLineGap, String lineColor, int lineThickness, int lineType, int shift) {
        byte[] byteArray = new byte[0];
        try {
            Mat cdst = new Mat();
            // Decode image from input byte array
            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            // Check if image is loaded fine
            // Copy edges to the images that will display the results in BGR
            Imgproc.cvtColor(src, cdst, Imgproc.COLOR_GRAY2BGR);

            // Probabilistic Line Transform
            Mat linesP = new Mat(); // will hold the results of the detection
            Imgproc.HoughLinesP(src, linesP, rho, theta, threshold, minLineLength, maxLineGap); // runs the actual
            // detection
            // Draw the lines
            for (int x = 0; x < linesP.rows(); x++) {
                double[] l = linesP.get(x, 0);
                Imgproc.line(cdst, new Point(l[0], l[1]), new Point(l[2], l[3]), convertColorToScalar(lineColor),
                        lineThickness, lineType, shift);
            }

            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", cdst, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] houghCircles(byte[] byteData, int method, double dp, double minDist, double param1, double param2,
                               int minRadius, int maxRadius, int centerWidth, String centerColor, int circleWidth, String circleColor) {
        byte[] byteArray = new byte[0];
        try {
            Mat circles = new Mat();
            // Decode image from input byte array
            Mat input = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            // Imgproc.medianBlur(input, input, 5);
            // resize operation
            Imgproc.HoughCircles(input, circles, method, dp, minDist, param1, param2, minRadius, maxRadius);

            if (circles.cols() > 0) {
                for (int x = 0; x < (circles.cols()); x++) {
                    double circleVec[] = circles.get(0, x);

                    if (circleVec == null) {
                        break;
                    }

                    Point center = new Point((int) circleVec[0], (int) circleVec[1]);
                    int radius = (int) circleVec[2];

                    Imgproc.circle(input, center, 3, convertColorToScalar(centerColor), centerWidth);
                    Imgproc.circle(input, center, radius, convertColorToScalar(circleColor), circleWidth);
                }
            }
            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", input, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] warpPerspectiveTransform(byte[] byteData, ArrayList sourcePoints, ArrayList destinationPoints, ArrayList outputSize) {
        byte[] byteArray = new byte[0];
        try {
            // Decode image from input byte array
            List<Double> s = new ArrayList<>();
            List<Double> t = new ArrayList<>();
            for(int i = 0; i<sourcePoints.size(); i++) {
                s.add((double) ((Integer) sourcePoints.get(i)));
            }
            for(int i = 0; i<destinationPoints.size(); i++) {
                t.add((double) ((Integer) destinationPoints.get(i)));
            }
            Mat input = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);
            MatOfPoint2f src = new MatOfPoint2f(new Point(s.get(0), s.get(1)),
                    new Point(s.get(2), s.get(3)), new Point(s.get(4), s.get(5)),
                    new Point(s.get(6), s.get(7)));
            MatOfPoint2f dst = new MatOfPoint2f(new Point(t.get(0), t.get(1)),
                    new Point(t.get(2), t.get(3)), new Point(t.get(4), t.get(5)),
                    new Point(t.get(6), t.get(7)));
            Mat warpMat = Imgproc.getPerspectiveTransform(src, dst);
            // This is you new image as Mat
            Mat destImage = new Mat();
            Imgproc.warpPerspective(input, destImage, warpMat, new Size((double) outputSize.get(0), (double) outputSize.get(1)));
            // instantiating an empty MatOfByte class
            MatOfByte matOfByte = new MatOfByte();
            // Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", destImage, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public byte[] grabCut(byte[] byteData, int px, int py, int qx, int qy, int itercount, int mode) {
        byte[] byteArray = new byte[0];
        try {
            // Decode image from input byte array
            Mat dst = new Mat();
            Mat background = new Mat();
            Mat foreground = new Mat();

            Mat src = Imgcodecs.imdecode(new MatOfByte(byteData), Imgcodecs.IMREAD_ANYCOLOR | Imgcodecs.IMREAD_ANYDEPTH);

            Rect rect = new Rect(px, px, qx, qy);

            Imgproc.grabCut(src, dst, rect, background, foreground, itercount, mode);  //0 = start with a rectangle

            Mat source = new Mat(1, 1, CvType.CV_8U, new Scalar(3.0));
            Core.compare(dst, source, dst, Core.CMP_EQ);

            Mat foreground2 = new Mat(src.size(), CvType.CV_8UC3, new Scalar(255,
                    255, 255,255));
            src.copyTo(foreground2, dst);

            MatOfByte matOfByte = new MatOfByte();
            //Converting the Mat object to MatOfByte
            Imgcodecs.imencode(".jpg", foreground2, matOfByte);
            byteArray = matOfByte.toArray();
        } catch (Exception e) {
            System.out.println("OpenCV Error: " + e.toString());
        }
        return byteArray;
    }

    @SuppressLint("MissingPermission")
    public Scalar convertColorToScalar(String color) {
        return new Scalar(Integer.valueOf(color.substring(1, 3), 16), Integer.valueOf(color.substring(3, 5), 16),
                Integer.valueOf(color.substring(5, 7), 16));
    }
}
