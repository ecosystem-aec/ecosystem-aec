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

import com.google.atap.tangoservice.TangoCameraIntrinsics;
import com.google.atap.tangoservice.TangoPoseData;
import com.google.atap.tangoservice.TangoXyzIjData;
import com.projecttango.rajawali.ScenePoseCalcuator;
import com.projecttango.tangosupport.TangoSupport;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;

/**
 * This helper class keeps a copy of the point cloud data received in callbacks for use with the
 * plane fitting function.
 * It is implemented to be thread safe so that the caller (the Activity) doesn't need to worry
 * about locking between the Tango callback and UI threads.
 */
public class PointCloudManager {
    private static final String TAG = "PointCloudManager";
    private static final int BYTES_PER_FLOAT = 4;
    private static final int POINT_TO_XYZ = 3;

    private final TangoCameraIntrinsics mTangoCameraIntrinsics;
    private final TangoXyzIjData mXyzIjData;
    private TangoPoseData mDevicePoseAtCloudTime;
    private PointCloudData mSharedPointCloudData;
    private PointCloudData mCallbackPointCloudData;
    private PointCloudData mRenderPointCloudData;
    private Object mPointCloudLock;
    private boolean mSwapSignal;

    public PointCloudManager(TangoCameraIntrinsics intrinsics) {
        mXyzIjData = new TangoXyzIjData();
        mTangoCameraIntrinsics = intrinsics;
        mPointCloudLock = new Object();
        mSwapSignal = false;
        setupBuffers(60000);
    }

    /**
     * Sets up three buffers namely, Callback, Shared and Render buffers allocated with maximum
     * number of points a point cloud can have.
     * @param maxDepthPoints
     */
    private void setupBuffers(int maxDepthPoints){
        mCallbackPointCloudData = new PointCloudData();
        mSharedPointCloudData = new PointCloudData();
        mRenderPointCloudData = new PointCloudData();
        mCallbackPointCloudData.floatBuffer = ByteBuffer
                .allocateDirect(maxDepthPoints * BYTES_PER_FLOAT * POINT_TO_XYZ)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        mSharedPointCloudData.floatBuffer = ByteBuffer
                .allocateDirect(maxDepthPoints * BYTES_PER_FLOAT * POINT_TO_XYZ)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
        mRenderPointCloudData.floatBuffer = ByteBuffer
                .allocateDirect(maxDepthPoints * BYTES_PER_FLOAT * POINT_TO_XYZ)
                .order(ByteOrder.nativeOrder()).asFloatBuffer();
    }

    /**
     * Update the current cloud data with the provided xyzIjData from a Tango callback.
     *
     * @param from          The point cloud data
     * @param xyzIjPose     The device pose with respect to start of service at the time
     *                      the point cloud was acquired
     */
    public synchronized void updateXyzIjData(TangoXyzIjData from, TangoPoseData xyzIjPose) {
        mDevicePoseAtCloudTime = xyzIjPose;

        if (mXyzIjData.xyz == null || mXyzIjData.xyz.capacity() < from.xyzCount * 3) {
            mXyzIjData.xyz = ByteBuffer.allocateDirect(from.xyzCount * 3 * 4)
                    .order(ByteOrder.nativeOrder()).asFloatBuffer();
        } else {
            mXyzIjData.xyz.rewind();
        }

        mXyzIjData.xyzCount = from.xyzCount;
        mXyzIjData.timestamp = from.timestamp;

        from.xyz.rewind();
        mXyzIjData.xyz.put(from.xyz);
        mXyzIjData.xyz.rewind();
        from.xyz.rewind();
    }

    /**
     * Calculate the plane that best fits the current point cloud at the provided u,v coordinates
     * in the 2D projection of the point cloud data (i.e.: point cloud image).
     *
     * @param u                     u (horizontal) component of the click location
     * @param v                     v (vertical) component of the click location
     * @param devicePoseAtClickTime Device pose at the time this operation is requested
     * @param poseCalcuator         ScenePoseCalculator helper instance to calculate transforms
     * @return                      The point and plane model, in depth sensor frame
     */
    public synchronized TangoSupport.IntersectionPointPlaneModelPair fitPlane(float u, float v,
            TangoPoseData devicePoseAtClickTime, ScenePoseCalcuator poseCalcuator) {

        // We need to calculate the transform between the color camera at the time the user clicked
        // and the depth camera at the time the depth cloud was acquired.
        // This operation is currently implemented in the provided ScenePoseCalculator helper
        // class. In the future, the support library will provide a method for this calculation.
        TangoPoseData colorCameraTDepthCameraWithTime
                = poseCalcuator.calculateColorCameraTDepthWithTime(devicePoseAtClickTime, mDevicePoseAtCloudTime);

        return TangoSupport.fitPlaneModelNearClick(mXyzIjData, mTangoCameraIntrinsics,
                colorCameraTDepthCameraWithTime, u, v);
    }

    /**
     * Updates the callbackbuffer with latest pointcloud and swaps the
     * @param callbackBuffer
     * @param pointCount
     */
    public void updateCallbackBufferAndSwap(FloatBuffer callbackBuffer, int pointCount){
        mSharedPointCloudData.floatBuffer.position(0);
        mCallbackPointCloudData.floatBuffer.put(callbackBuffer);
        synchronized (mPointCloudLock){
            FloatBuffer temp = mSharedPointCloudData.floatBuffer;
            int tempCount = pointCount;
            mSharedPointCloudData.floatBuffer = mCallbackPointCloudData .floatBuffer;
            mSharedPointCloudData.pointCount = mCallbackPointCloudData.pointCount;
            mCallbackPointCloudData.floatBuffer = temp;
            mCallbackPointCloudData.pointCount = tempCount;
            mSwapSignal = true;
        }
    }

    /**
     * Returns a shallow copy of latest Point Cloud Render buffer.If there is a swap signal available
     * SharedPointCloud buffer is swapped with Render buffer and it is returned.
     * @return PointClouData which contains a reference to latest PointCloud Floatbuffer and count.
     */
    public PointCloudData updateAndGetLatestPointCloudRenderBuffer(){
        synchronized (mPointCloudLock){
            if(mSwapSignal) {
                FloatBuffer temp = mRenderPointCloudData.floatBuffer;
                int tempCount = mRenderPointCloudData.pointCount;
                mRenderPointCloudData.floatBuffer = mSharedPointCloudData.floatBuffer;
                mRenderPointCloudData.pointCount = mSharedPointCloudData.pointCount;
                mSharedPointCloudData.floatBuffer = temp;
                mSharedPointCloudData.pointCount = tempCount;
                mSwapSignal = false;
            }
        }
        return mRenderPointCloudData;
    }

    /**
     * A class to hold Depth data in a {@link FloatBuffer} and number of points associated with it.
     */
    public class PointCloudData{
        public FloatBuffer floatBuffer;
        public int pointCount;
    }
}
