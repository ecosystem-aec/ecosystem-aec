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

import android.content.Context;
import android.view.MotionEvent;

import com.google.atap.tangoservice.TangoPoseData;
import com.projecttango.rajawali.Pose;
import com.projecttango.rajawali.ScenePoseCalcuator;
import com.projecttango.rajawali.ar.TangoRajawaliRenderer;
import com.projecttango.rajawali.renderables.FrustumAxes;
import com.projecttango.rajawali.renderables.primitives.Points;

import org.rajawali3d.Object3D;
import org.rajawali3d.lights.DirectionalLight;
import org.rajawali3d.materials.Material;
import org.rajawali3d.materials.methods.DiffuseMethod;
import org.rajawali3d.materials.plugins.FogMaterialPlugin;
import org.rajawali3d.materials.textures.ATexture;
import org.rajawali3d.materials.textures.Texture;
import org.rajawali3d.math.vector.Vector3;
import org.rajawali3d.primitives.Cube;
import org.rajawali3d.primitives.RectangularPrism;

/**
 * Very simple example augmented reality renderer which displays two objects in a fixed position
 * in the world and the uses the Tango position tracking to keep them in place.
 * <p/>
 * This follows the same development model than any regular Rajawali application with the following
 * peculiarities:
 * - It extends <code>TangoRajawaliArRenderer</code>
 * - It calls <code>super.initScene()</code> in the initialization
 * - It doesn't do anything with the camera, since that is handled automatically by Tango
 */
public class AugmentedRealityRenderer extends TangoRajawaliRenderer {

    private static final float CUBE_SIDE_LENGTH = 0.12f;
    private static final int MAX_NUMBER_OF_POINTS = 60000;

    private Pose mPlanePose;
    private Pose mPointCloudPose;
    private Pose mCameraPose;

    private FrustumAxes mFrustumAxes;
    private boolean mPlanePoseUpdated = false;

    private Object3D mObject;
    private Object3D mObject2;
    private DirectionalLight light;
    private DirectionalLight light2;
    private Points mPoints;
    private PointCloudManager mPointCloudManager;

    public AugmentedRealityRenderer(Context context, PointCloudManager pointCloudManager) {
        super(context);
        mPointCloudManager = pointCloudManager;

    }

    @Override
    protected void initScene() {
        // Remember to call super.initScene() to allow TangoRajawaliArRenderer to set-up
        super.initScene();

        mPoints = new Points(MAX_NUMBER_OF_POINTS);
        getCurrentScene().addChild(mPoints);

        mFrustumAxes = new FrustumAxes(3);
        getCurrentScene().addChild(mFrustumAxes);

        // Add a directional light in an arbitrary direction
        light = new DirectionalLight(1, 0.2, -1);
        light.setColor(0, 0, 1);
        light.setPower(0.8f);
        light.setPosition(3, 2, 4);
        getCurrentScene().addLight(light);

        light2 = new DirectionalLight(-1, -0.2, 1);
        light2.setColor(1, 0, 0);
        light2.setPower(0.8f);
        light2.setPosition(3, 2, 4);
        getCurrentScene().addLight(light2);


//        int fogColor = 0x009900;
//        getCurrentScene().setBackgroundColor(fogColor);

//        getCurrentScene().setFog(new FogMaterialPlugin.FogParams(FogMaterialPlugin.FogType.LINEAR, fogColor, 0, 1));

        // Set-up a material: green with application of the light and instructions
        Material material = new Material();
        material.setColor(0x80808080);
        try {
            Texture t = new Texture("f0g", R.drawable.f0g);
            material.addTexture(t);
        } catch (ATexture.TextureException e) {
            e.printStackTrace();
        }
        /*
        */
        material.setColorInfluence(0.9f);
        material.enableLighting(true);
        material.setDiffuseMethod(new DiffuseMethod.Lambert());

        mObject = new RectangularPrism(CUBE_SIDE_LENGTH, CUBE_SIDE_LENGTH*4, CUBE_SIDE_LENGTH);
        mObject.setMaterial(material);
        mObject.setPosition(0, CUBE_SIDE_LENGTH*2, 0);
        mObject.setRotation(Vector3.Axis.Z, 180);
        mObject.setTransparent(true);
        getCurrentScene().addChild(mObject);
        mObject2 = new RectangularPrism(CUBE_SIDE_LENGTH*1.6f, CUBE_SIDE_LENGTH/3, CUBE_SIDE_LENGTH*1.6f);
        mObject2.setMaterial(material);
        mObject2.setPosition(0, 0, 0);
        mObject2.setRotation(Vector3.Axis.Z, 180);
        mObject2.setTransparent(true);
        getCurrentScene().addChild(mObject2);
    }

    @Override
    protected void onRender(long ellapsedRealtime, double deltaTime) {
        super.onRender(ellapsedRealtime, deltaTime);

        synchronized (this) {
            if (mPlanePoseUpdated == true) {
                mPlanePoseUpdated = false;
                // Place the 3D object in the location of the detected plane
                Vector3 mPlanePosePosition = mPlanePose.getPosition();
                mObject.setPosition(mPlanePosePosition);
                mObject.setPosition(mPlanePosePosition.x, mPlanePosePosition.y + CUBE_SIDE_LENGTH*2, mPlanePosePosition.z);
                mObject.setOrientation(mPlanePose.getOrientation());
                mObject2.setPosition(mPlanePosePosition.x, mPlanePosePosition.y, mPlanePosePosition.z);
                mObject2.setOrientation(mPlanePose.getOrientation());

                light.setPosition(mPlanePosePosition);
                light2.setPosition(mPlanePosePosition);

                // Move it forward by half of the size of the cube to make it flush with the plane
                // surface
//                mObject.moveForward(CUBE_SIDE_LENGTH / 2.0f);
//                mObject2.moveForward(CUBE_SIDE_LENGTH / 2.0f);
            }
        }
        PointCloudManager.PointCloudData renderPointCloudData
                = mPointCloudManager.updateAndGetLatestPointCloudRenderBuffer();
        mPoints.updatePoints(renderPointCloudData.floatBuffer, renderPointCloudData.pointCount);
        if(mCameraPose==null || mPointCloudPose == null){
            return;
        }
        // Update the scene objects with the latest device position and orientation information.
        // Synchronize to avoid concurrent access from the Tango callback thread below.
        synchronized (this) {
            mFrustumAxes.setPosition(mCameraPose.getPosition());
            mFrustumAxes.setOrientation(mCameraPose.getOrientation());

            mPoints.setPosition(mPointCloudPose.getPosition());
            mPoints.setOrientation(mPointCloudPose.getOrientation());
//            mTouchViewHandler.updateCamera(mCameraPose.getPosition(), mCameraPose.getOrientation());
        }

    }

    /**
     * Update the 3D object based on the provided measurement point, normal (in depth frame) and
     * device pose at the time of measurement.
     */
    public synchronized void updateObjectPose(double[] point, double[] normal,
                                              TangoPoseData devicePose) {
        mPlanePose = mScenePoseCalcuator.planeFitToOpenGLPose(point, normal, devicePose);
        mPlanePoseUpdated = true;
    }

    /**
     * Provide access to scene calculator helper class to perform necessary transformations.
     * NOTE: This won't be necessary once transformation functions are available through the
     * support library
     */
    public ScenePoseCalcuator getPoseCalculator() {
        return mScenePoseCalcuator;
    }

    @Override
    public void onOffsetsChanged(float xOffset, float yOffset, float xOffsetStep, float yOffsetStep, int xPixelOffset, int yPixelOffset) {

    }

    @Override
    public void onTouchEvent(MotionEvent event) {

    }

    public synchronized void updatePointCloudPose(TangoPoseData pointCloudPose) {
        mPointCloudPose = mScenePoseCalcuator.toOpenGLPointCloudPose(pointCloudPose);
    }

    /**
     * Updates our information about the current device pose.
     * This is called from the Tango service thread through the callback API. Synchronize to avoid
     * concurrent access from the OpenGL thread above.
     */
    public synchronized void updateDevicePose(TangoPoseData tangoPoseData) {
        mCameraPose = mScenePoseCalcuator.toOpenGLCameraPose(tangoPoseData);
    }

}
