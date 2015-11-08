/*
 * Copyright 2014 Google Inc. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
package com.projecttango.experiments.augmentedrealitysample;

import android.app.Activity;
import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.widget.Toast;

import com.google.atap.tango.ux.TangoUx;
import com.google.atap.tango.ux.TangoUxLayout;
import com.google.atap.tango.ux.UxExceptionEvent;
import com.google.atap.tango.ux.UxExceptionEventListener;
import com.google.atap.tangoservice.Tango;
import com.google.atap.tangoservice.Tango.OnTangoUpdateListener;
import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoConfig;
import com.google.atap.tangoservice.TangoCoordinateFramePair;
import com.google.atap.tangoservice.TangoErrorException;
import com.google.atap.tangoservice.TangoEvent;
import com.google.atap.tangoservice.TangoInvalidException;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.rajawali.ar.TangoRajawaliView;
import com.projecttango.tangosupport.TangoSupport;

import java.nio.FloatBuffer;
import java.util.ArrayList;

/**
 * An example showing how to build a very simple augmented reality application in Java.
 * It uses Rajawali to do the rendering through the utility classes
 * <code>TangoRajawaliRenderer</code> and <code>TangoRajawaliView</code> from TangoUtils.
 * It also uses the TangoSupportLibrary to do plane fitting using the PointCloud data. Whenever the
 * user clicks on the camera display, plane detection will be done on the surface closest to the
 * click location and a 3D object will be placed in the scene anchored in that location.
 * <p/>
 * TangoRajawaliView is used in the same way as the TangoCamaraPreview: we first need initialize the
 * TangoRajawaliView class with the activity's context and connect to the camera we want by using
 * connectToTangoCamera method. Once the connection is established we need to update
 * the view's texture by using the onFrameAvailable callbacks.
 * <p/>
 * The TangoRajawaliRenderer class is used the same way as a RajawaliRenderer. We need to create it
 * with a reference to the activity's context and then pass it to the view with the view's
 * setSurfaceRenderer method.
 * The implementation of the 3D world is done by subclassing the Renderer, just like any other
 * Rajawali application.
 * <p/>
 * Note that it is important to include the KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION configuration
 * parameter in order to achieve best results synchronizing the Rajawali virtual world with
 * the RGB camera.
 */
public class AugmentedRealityActivity extends Activity implements View.OnTouchListener {
    private static final int SECS_TO_MILLISECS = 1000;

    private static final String TAG = "AugmentedRealityActiv";
    private TangoRajawaliView mGLView;
    private AugmentedRealityRenderer mRenderer;
    private PointCloudManager mPointCloudManager;
    private Tango mTango;
    private boolean mIsConnected;
    private boolean mIsPermissionGranted;
    private float mCurrentTimeStamp;
    private TangoPoseData mPose;
    private float mDeltaTime;
    private float mPosePreviousTimeStamp;
    private int mPreviousPoseStatus;
    private int mCount;
    private float mPointCloudFrameDelta;
    private float mXyIjPreviousTimeStamp;
    private float mAverageDepth;
    private int mPointCount;
    private TangoUx mTangoUx;
    private Object mUiPoseLock = new Object();
    private Object mUiDepthLock = new Object();


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        mTango = new Tango(this);
        // Set-up point cloud plane fitting library helper class
        mPointCloudManager = new PointCloudManager(mTango.getCameraIntrinsics(
                TangoCameraIntrinsics.TANGO_CAMERA_COLOR));
        mRenderer = new AugmentedRealityRenderer(this, mPointCloudManager);
        mGLView = new TangoRajawaliView(this);
        mGLView.setSurfaceRenderer(mRenderer);
        mGLView.setOnTouchListener(this);

        mTangoUx = setupTangoUxAndLayout();
        startActivityForResult(
                Tango.getRequestPermissionIntent(Tango.PERMISSIONTYPE_MOTION_TRACKING),
                Tango.TANGO_INTENT_ACTIVITYCODE);
        setContentView(mGLView);
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // Check which request we're responding to
        if (requestCode == Tango.TANGO_INTENT_ACTIVITYCODE) {
            // Make sure the request was successful
            if (resultCode == RESULT_CANCELED) {
                Toast.makeText(this, "Motion Tracking Permissions Required!",
                        Toast.LENGTH_SHORT).show();
                finish();
            } else {
                startAugmentedreality();
                mIsPermissionGranted = true;
            }
        }
    }

    // Augmented reality view and renderer
    private void startAugmentedreality() {
        if (!mIsConnected) {
            mIsConnected = true;
            // Connect to color camera
            mGLView.connectToTangoCamera(mTango, TangoCameraIntrinsics.TANGO_CAMERA_COLOR);

            // Use default configuration for Tango Service, plus low latency IMU integration.
            TangoConfig config = mTango.getConfig(TangoConfig.CONFIG_TYPE_DEFAULT);
            // NOTE: low latency integration is necessary to achieve a precise alignment of
            // virtual objects with the RBG image and produce a good AR effect.

            config.putBoolean(TangoConfig.KEY_BOOLEAN_LOWLATENCYIMUINTEGRATION, true);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_DEPTH, true);
            config.putBoolean(TangoConfig.KEY_BOOLEAN_LEARNINGMODE, true);
            mTango.connect(config);

            setTangoListeners();

            // Get extrinsics from device for use in transforms
            // This needs to be done after connecting Tango and listeners
            setupExtrinsics();

        }
    }

    private void setTangoListeners() {
        // Configure the Tango coordinate frame pair
        final ArrayList<TangoCoordinateFramePair> framePairs = new ArrayList<TangoCoordinateFramePair>();
        framePairs.add(new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE));
        // Listen for new Tango data
        mTango.connectListener(framePairs, new OnTangoUpdateListener() {

            @Override
            public void onPoseAvailable(final TangoPoseData pose) {
                // Passing in the pose data to UX library produce exceptions.
                if (mTangoUx != null) {
                    mTangoUx.updatePoseStatus(pose.statusCode);
                }
                // Make sure to have atomic access to Tango Pose Data so that
                // render loop doesn't interfere while Pose call back is updating
                // the data.
                synchronized (mUiPoseLock) {
                    mPose = pose;
                    // Calculate the delta time from previous pose.
                    mDeltaTime = (float) (pose.timestamp - mPosePreviousTimeStamp)
                            * SECS_TO_MILLISECS;
                    mPosePreviousTimeStamp = (float) pose.timestamp;
                    if (mPreviousPoseStatus != pose.statusCode) {
                        mCount = 0;
                    }
                    mCount++;
                    mPreviousPoseStatus = pose.statusCode;
                }
                mRenderer.updateDevicePose(pose);
            }

            @Override
            public void onXyzIjAvailable(final TangoXyzIjData xyzIj) {
                // ------
                if (mTangoUx != null) {
                    mTangoUx.updateXyzCount(xyzIj.xyzCount);
                }

                    /*
                float[] xx = new float[xyzIj.xyzCount * 3];
                xyzIj.xyz.get(xx);
                float maxX = Float.NEGATIVE_INFINITY;
                float minX = Float.POSITIVE_INFINITY;
                float maxY = Float.NEGATIVE_INFINITY;
                float minY = Float.POSITIVE_INFINITY;
                float maxZ = Float.NEGATIVE_INFINITY;
                float minZ = Float.POSITIVE_INFINITY;

                for (int i = 0; i < xx.length; i += 3) {
                    float x = xx[i];
                    float y = xx[i + 1];
                    float z = xx[i + 2];
                    if (x > maxX) {
                        maxX = x;
                    }
                    if (x < minX) {
                        minX = x;
                    }
                    if (y > maxY) {
                        maxY = y;
                    }
                    if (y < minY) {
                        minY = y;
                    }
                    if (z > maxZ) {
                        maxZ = z;
                    }
                    if (z < minZ) {
                        minZ = z;
                    }
                }
                Log.e(TAG, String.format("lalala %d: X: %f -> %f Y: %f -> %f Z: %f -> %f", xyzIj.xyzCount, minX, maxX, minY, maxY, minZ, maxZ));
                    */

                mPointCloudManager.updateCallbackBufferAndSwap(xyzIj.xyz, xyzIj.xyzCount);
                TangoPoseData pointCloudPose = mTango.getPoseAtTime(mCurrentTimeStamp,
                        framePairs.get(0));
//                mRenderer.updatePointCloudPose(pointCloudPose);

                // Make sure to have atomic access to TangoXyzIjData so that
                // UI loop doesn't interfere while onXYZijAvailable callback is updating
                // the mPoint cloud data.
                synchronized (mUiDepthLock) {
                    mCurrentTimeStamp = (float) xyzIj.timestamp;
                    mPointCloudFrameDelta = (mCurrentTimeStamp - mXyIjPreviousTimeStamp)
                            * SECS_TO_MILLISECS;
                    mXyIjPreviousTimeStamp = mCurrentTimeStamp;
                    mAverageDepth = getAveragedDepth(xyzIj.xyz);
                    try {
                        mPointCount = xyzIj.xyzCount;
                    } catch (TangoErrorException e) {
                        Toast.makeText(getApplicationContext(), R.string.TangoError,
                                Toast.LENGTH_SHORT).show();
                    } catch (TangoInvalidException e) {
                        Toast.makeText(getApplicationContext(), R.string.TangoError,
                                Toast.LENGTH_SHORT).show();
                    }
                }
                // ------

                // Get the device pose at the time the point cloud was acquired
                TangoCoordinateFramePair framePair = new TangoCoordinateFramePair(
                        TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                        TangoPoseData.COORDINATE_FRAME_DEVICE);
                TangoPoseData cloudPose = mTango.getPoseAtTime(xyzIj.timestamp, framePair);

                // Save the cloud and point data for later use
                mPointCloudManager.updateXyzIjData(xyzIj, cloudPose);

                mPointCloudManager.updateCallbackBufferAndSwap(xyzIj.xyz, xyzIj.xyzCount);
                mRenderer.updatePointCloudPose(pointCloudPose);

            }

            @Override
            public void onTangoEvent(final TangoEvent event) {
                if (mTangoUx != null) {
                    mTangoUx.updateTangoEvent(event);
                }
//                runOnUiThread(new Runnable() {
//                    @Override
//                    public void run() {
//                        mTangoEventTextView.setText(event.eventKey + ": " + event.eventValue);
//                    }
//                });
            }

            @Override
            public void onFrameAvailable(int cameraId) {
                // Check if the frame available is for the camera we want and
                // update its frame on the view.
                if (cameraId == TangoCameraIntrinsics.TANGO_CAMERA_COLOR) {
                    mGLView.onFrameAvailable();
                }

            }
        });
    }

    /**
     * Calculates and stores the fixed transformations between the device and the various sensors
     * to be used later for transformations between frames.
     */
    private void setupExtrinsics() {
        // Create Camera to IMU Transform
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair();
        framePair.baseFrame = TangoPoseData.COORDINATE_FRAME_IMU;
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_COLOR;
        TangoPoseData imuTrgbPose = mTango.getPoseAtTime(0.0, framePair);

        // Create Device to IMU Transform
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_DEVICE;
        TangoPoseData imuTdevicePose = mTango.getPoseAtTime(0.0, framePair);

        // Create Depth camera to IMU Transform
        framePair.targetFrame = TangoPoseData.COORDINATE_FRAME_CAMERA_DEPTH;
        TangoPoseData imuTdepthPose = mTango.getPoseAtTime(0.0, framePair);

        mRenderer.setupExtrinsics(imuTdevicePose, imuTrgbPose, imuTdepthPose);
    }


    @Override
    protected void onPause() {
        super.onPause();
        if (mIsConnected) {
            mGLView.disconnectCamera();
            mTango.disconnect();
            mIsConnected = false;
        }
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (!mIsConnected && mIsPermissionGranted) {
            startAugmentedreality();
        }
    }

    @Override
    public boolean onTouch(View view, MotionEvent motionEvent) {
        if (motionEvent.getAction() == MotionEvent.ACTION_UP) {
            // Calculate click location in u,v (0;1) coordinates
            float u = motionEvent.getX() / view.getWidth();
            float v = motionEvent.getY() / view.getHeight();

            try {
                doFitPlane(u, v);
            } catch (Throwable t) {
                Log.e(TAG, "Exception measuring nomral", t);
            }
        }
        return true;
    }

    /**
     * Use the TangoSupport library with point cloud data to calculate the plane of
     * the world feature pointed at the location the camera is looking at and update the
     * renderer to show a 3D object in that location.
     */
    private void doFitPlane(float u, float v) {
        // Get the current device pose
        TangoCoordinateFramePair framePair = new TangoCoordinateFramePair(
                TangoPoseData.COORDINATE_FRAME_START_OF_SERVICE,
                TangoPoseData.COORDINATE_FRAME_DEVICE);
        TangoPoseData devicePose = mTango.getPoseAtTime(0.0, framePair);

        // Perform plane fitting with the latest available point cloud data
        TangoSupport.IntersectionPointPlaneModelPair planeModel =
                mPointCloudManager.fitPlane(u, v, devicePose, mRenderer.getPoseCalculator());
        mRenderer.updateObjectPose(planeModel.intersectionPoint, planeModel.planeModel, devicePose);
    }

    /**
     * Calculates the average depth from a point cloud buffer
     *
     * @param pointCloudBuffer
     * @return Average depth.
     */
    private float getAveragedDepth(FloatBuffer pointCloudBuffer) {
        mPointCount = pointCloudBuffer.capacity() / 3;
        float totalZ = 0;
        float averageZ = 0;
        for (int i = 0; i < pointCloudBuffer.capacity() - 3; i = i + 3) {
            totalZ = totalZ + pointCloudBuffer.get(i + 2);
        }
        if (mPointCount != 0)
            averageZ = totalZ / mPointCount;
        return averageZ;
    }

    /**
     * Sets up TangoUX layout and sets its listener.
     */
    private TangoUx setupTangoUxAndLayout() {
        TangoUxLayout uxLayout = (TangoUxLayout) findViewById(R.id.layout_tango);
        TangoUx tangoUx = new TangoUx(this);
        tangoUx.setLayout(uxLayout);
        tangoUx.setUxExceptionEventListener(mUxExceptionListener);
        return tangoUx;
    }

    /*
* This is an advanced way of using UX exceptions. In most cases developers can just use the in
* built exception notifications using the Ux Exception layout. In case a developer doesn't want
* to use the default Ux Exception notifications, he can set the UxException listener as shown
* below.
* In this example we are just logging all the ux exceptions to logcat, but in a real app,
* developers should use these exceptions to contextually notify the user and help direct the
* user in using the device in a way Tango service expects it.
*/
    private UxExceptionEventListener mUxExceptionListener = new UxExceptionEventListener() {

        @Override
        public void onUxExceptionEvent(UxExceptionEvent uxExceptionEvent) {
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_LYING_ON_SURFACE) {
                Log.i(TAG, "Device lying on surface ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_DEPTH_POINTS) {
                Log.i(TAG, "Very few depth points in mPoint cloud ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_FEW_FEATURES) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_INCOMPATIBLE_VM) {
                Log.i(TAG, "Device not running on ART");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOTION_TRACK_INVALID) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_MOVING_TOO_FAST) {
                Log.i(TAG, "Invalid poses in MotionTracking ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_OVER_EXPOSED) {
                Log.i(TAG, "Camera Over Exposed");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_TANGO_SERVICE_NOT_RESPONDING) {
                Log.i(TAG, "TangoService is not responding ");
            }
            if (uxExceptionEvent.getType() == UxExceptionEvent.TYPE_UNDER_EXPOSED) {
                Log.i(TAG, "Camera Under Exposed ");
            }

        }
    };

}
