package com.example.cameraeeg;

import android.Manifest;
import android.app.Activity;
import android.app.AlertDialog;
import android.app.Dialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.content.DialogInterface;
import android.content.Intent;
import android.content.pm.PackageManager;
import android.content.res.Configuration;
import android.graphics.Matrix;
import android.graphics.RectF;
import android.graphics.SurfaceTexture;
import android.hardware.camera2.CameraAccessException;
import android.hardware.camera2.CameraCaptureSession;
import android.hardware.camera2.CameraCharacteristics;
import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.CameraManager;
import android.hardware.camera2.CameraMetadata;
import android.hardware.camera2.CaptureRequest;
import android.hardware.camera2.params.StreamConfigurationMap;
import android.media.MediaRecorder;
import android.os.Bundle;
import android.os.Environment;
import android.os.Handler;
import android.os.HandlerThread;
import android.os.SystemClock;
import android.text.format.DateFormat;
import android.util.Log;
import android.util.Size;
import android.util.SparseIntArray;
import android.view.KeyEvent;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.view.Surface;
import android.view.TextureView;
import android.view.View;
import android.widget.Button;
import android.widget.Chronometer;
import android.widget.Toast;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.appcompat.app.AppCompatActivity;
import androidx.core.app.ActivityCompat;
import androidx.fragment.app.DialogFragment;

import com.retrocode.smarting.android.services.SmartingDevice;
import com.retrocode.smarting.common.DeviceBuffer;
import com.retrocode.smarting.common.DeviceChannelBuffer;
import com.retrocode.smarting.common.callback.DeviceCommandCallback;
import com.retrocode.smarting.common.callback.DeviceDataCallback;
import com.retrocode.smarting.common.callback.DeviceInitCallback;
import com.retrocode.smarting.common.command.DeviceChannel;
import com.retrocode.smarting.common.command.DeviceCommand;
import com.retrocode.smarting.common.command.DeviceCommandEnum;
import com.retrocode.smarting.common.command.DeviceCommandResult;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.PrintWriter;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Set;
import java.util.concurrent.Semaphore;
import java.util.concurrent.TimeUnit;

public class MainActivity extends AppCompatActivity implements DeviceCommandCallback, DeviceInitCallback, DeviceDataCallback {

    private static final int SENSOR_ORIENTATION_DEFAULT_DEGREES = 90;


    private static final int SENSOR_ORIENTATION_INVERSE_DEGREES = 270;
    private static final SparseIntArray DEFAULT_ORIENTATIONS = new SparseIntArray();
    private static final SparseIntArray INVERSE_ORIENTATIONS = new SparseIntArray();

    private static final String TAG = "Main";
    private static final int REQUEST_VIDEO_PERMISSIONS = 1;
    private static final String FRAGMENT_DIALOG = "dialog";

    static String[] REQUESTED_PERMISSIONS = {
            Manifest.permission.CAMERA,
            Manifest.permission.RECORD_AUDIO,
            Manifest.permission.WRITE_EXTERNAL_STORAGE};

    String[] write_permission = {Manifest.permission.WRITE_EXTERNAL_STORAGE};

    static String[] BLUETOOTH_PERMISSIONS = {Manifest.permission.BLUETOOTH,
            Manifest.permission.INTERNET,
            Manifest.permission.BLUETOOTH_ADMIN,
            Manifest.permission.READ_EXTERNAL_STORAGE,
            Manifest.permission.MOUNT_UNMOUNT_FILESYSTEMS,
            Manifest.permission.WAKE_LOCK};

    static final int REQUEST_CODE = 77;

    static final int BLUETOOTH_REQUEST_CODE = 11;

    static final int WRITE_PERMISSION_CODE = 911;

    private String FILENAME;

    static {
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_0, 90);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_90, 0);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_180, 270);
        DEFAULT_ORIENTATIONS.append(Surface.ROTATION_270, 180);
    }

    static {
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_0, 270);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_90, 180);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_180, 90);
        INVERSE_ORIENTATIONS.append(Surface.ROTATION_270, 0);
    }

    /**
     * An {@link AutoFitTextureView} for camera preview.
     */
    private AutoFitTextureView mTextureView;

    /**
     * Button to record video
     */
    private Button mButtonVideo;

    /**
     * A reference to the opened {@link android.hardware.camera2.CameraDevice}.
     */
    private CameraDevice mCameraDevice;

    /**
     * A reference to the current {@link android.hardware.camera2.CameraCaptureSession} for
     * preview.
     */
    private CameraCaptureSession mPreviewSession;

    /**
     * {@link TextureView.SurfaceTextureListener} handles several lifecycle events on a
     * {@link TextureView}.
     */
    private TextureView.SurfaceTextureListener mSurfaceTextureListener
            = new TextureView.SurfaceTextureListener() {

        @Override
        public void onSurfaceTextureAvailable(SurfaceTexture surfaceTexture,
                                              int width, int height) {
            openCamera(width, height);
        }

        @Override
        public void onSurfaceTextureSizeChanged(SurfaceTexture surfaceTexture,
                                                int width, int height) {
            configureTransform(width, height);
        }

        @Override
        public boolean onSurfaceTextureDestroyed(SurfaceTexture surfaceTexture) {
            return true;
        }

        @Override
        public void onSurfaceTextureUpdated(SurfaceTexture surfaceTexture) {
        }

    };

    /**
     * The {@link android.util.Size} of camera preview.
     */
    private Size mPreviewSize;

    /**
     * The {@link android.util.Size} of video recording.
     */
    private Size mVideoSize;

    /**
     * MediaRecorder
     */
    private MediaRecorder mMediaRecorder;

    /**
     * Whether the app is recording video now
     */
    private boolean mIsRecordingVideo;

    /**
     * An additional thread for running tasks that shouldn't block the UI.
     */
    private HandlerThread mBackgroundThread;

    /**
     * A {@link Handler} for running tasks in the background.
     */
    private Handler mBackgroundHandler;

    /**
     * A {@link Semaphore} to prevent the app from exiting before closing the camera.
     */
    private Semaphore mCameraOpenCloseLock = new Semaphore(1);

    /**
     * {@link CameraDevice.StateCallback} is called when {@link CameraDevice} changes its status.
     */
    private CameraDevice.StateCallback mStateCallback = new CameraDevice.StateCallback() {

        @Override
        public void onOpened(@NonNull CameraDevice cameraDevice) {
            mCameraDevice = cameraDevice;
            startPreview();
            mCameraOpenCloseLock.release();
            if (null != mTextureView) {
                configureTransform(mTextureView.getWidth(), mTextureView.getHeight());
            }
        }

        @Override
        public void onDisconnected(@NonNull CameraDevice cameraDevice) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
        }

        @Override
        public void onError(@NonNull CameraDevice cameraDevice, int error) {
            mCameraOpenCloseLock.release();
            cameraDevice.close();
            mCameraDevice = null;
            finish();
        }


    };
    private Integer mSensorOrientation;
    private String mNextVideoAbsolutePath;
    private CaptureRequest.Builder mPreviewBuilder;


    /**
     * In this sample, we choose a video size with 3x4 aspect ratio. Also, we don't use sizes
     * larger than 1080p, since MediaRecorder cannot handle such a high-resolution video.
     *
     * @param choices The list of available sizes
     * @return The video size
     */
    private static Size chooseVideoSize(Size[] choices) {
        for (Size size : choices) {
            if (size.getWidth() == size.getHeight() * 4 / 3 && size.getWidth() <= 480) {
                return size;
            }
        }
        Log.e(TAG, "Couldn't find any suitable video size");
        return choices[choices.length - 1];
    }

    /**
     * Given {@code choices} of {@code Size}s supported by a camera, chooses the smallest one whose
     * width and height are at least as large as the respective requested values, and whose aspect
     * ratio matches with the specified value.
     *
     * @param choices     The list of sizes that the camera supports for the intended output class
     * @param width       The minimum desired width
     * @param height      The minimum desired height
     * @param aspectRatio The aspect ratio
     * @return The optimal {@code Size}, or an arbitrary one if none were big enough
     */
    private static Size chooseOptimalSize(Size[] choices, int width, int height, Size aspectRatio) {
        // Collect the supported resolutions that are at least as big as the preview Surface
        List<Size> bigEnough = new ArrayList<>();
        int w = aspectRatio.getWidth();
        int h = aspectRatio.getHeight();
        for (Size option : choices) {
            if (option.getHeight() == option.getWidth() * h / w &&
                    option.getWidth() >= width && option.getHeight() >= height) {
                bigEnough.add(option);
            }
        }

        // Pick the smallest of those, assuming we found any
        if (bigEnough.size() > 0) {
            return Collections.min(bigEnough, new CompareSizesByArea());
        } else {
            Log.e(TAG, "Couldn't find any suitable preview size");
            return choices[0];
        }
    }

    /***/
    SmartingDevice EEGdevice;
    DeviceBuffer EEGbuffer;
    String deviceName;
    Button mConnectionButton;
    Button mButtonMarker;
    Chronometer chronometer;

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        //return super.onCreateOptionsMenu(menu);
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.action_menu, menu);
        return true;
    }

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        EEGdevice = new SmartingDevice(this);
        EEGdevice.addInitCallback(this);
        EEGdevice.addCommandCallback(this);
        EEGdevice.setFiltering(true);

       FILENAME = null;

        mIsRecordingVideo = false;


        mTextureView =  findViewById(R.id.autoFitTextureView);
        mButtonVideo =  findViewById(R.id.record_button);
        mButtonMarker = findViewById(R.id.marker_button1);
        chronometer = findViewById(R.id.chrono_meter);

        mButtonVideo.setEnabled(false);
        mButtonVideo.setAlpha(.5f);

        mButtonMarker.setEnabled(false);
        mButtonMarker.setAlpha(.5f);

        mConnectionButton = findViewById(R.id.connect_button);
        chronometer.setBase(SystemClock.elapsedRealtime());


        mButtonMarker.setOnClickListener((View v)->{

            EEGdevice.setMarker((byte)'I',1);
            Log.i(TAG, "Marker Sent");

            Toast.makeText(this,"Marker Sent",Toast.LENGTH_SHORT).show();

        });

        mButtonVideo.setOnClickListener((View v)->{

            if (mIsRecordingVideo) {
                stopRecordingVideo();
                Log.i("EEGstop","begin to stop");


                mConnectionButton.setAlpha(1f);
                mConnectionButton.setEnabled(true);

                Log.i("EEGstop","Recorded at "+FILENAME+".bdf");
                mButtonVideo.setText(R.string.start_recording);
                chronometer.stop();


            } else {
                getFilePath();

                if(FILENAME == null)
                {
                    Toast.makeText(getApplicationContext(),"Error in FilePath initialization"
                            ,Toast.LENGTH_LONG).show();

                }
                else {
                    startRecordingVideo();
                    mButtonVideo.setText(R.string.stop_recording);

                    mConnectionButton.setAlpha(0.5f);
                    mConnectionButton.setEnabled(false);

                    chronometer.setBase(SystemClock.elapsedRealtime());
                    chronometer.start();
                }
            }

        });

        mConnectionButton.setOnClickListener((View v)->{
            deviceName = getFirstDevice();

            if(deviceName==null)
            {
                new ConnectionDialog().show(getSupportFragmentManager(),FRAGMENT_DIALOG);
                return;
            }

            else
            {
                Toast.makeText(this,deviceName+" detected",
                        Toast.LENGTH_SHORT).show();
                onPairedtoSmarting(deviceName);
            }

        });

    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        switch(item.getItemId()){
            case R.id.action_settings:
                 changeSettings();
                 return true;

            default:
               return super.onOptionsItemSelected(item);

        }

        }


    @Override
    public void onResume() {
        super.onResume();
        startBackgroundThread();
        if (mTextureView.isAvailable()) {
            openCamera(mTextureView.getWidth(), mTextureView.getHeight());
        } else {
            mTextureView.setSurfaceTextureListener(mSurfaceTextureListener);
        }
    }

    @Override
    public void onStop() {
        closeCamera();
        stopBackgroundThread();
        super.onStop();
    }

    private String getFirstDevice(){
        BluetoothAdapter adapter = BluetoothAdapter.getDefaultAdapter();
        Set<BluetoothDevice> devices = adapter.getBondedDevices();
        String[] deviceList = new String[devices.size()];
        int x = 0;
        for (BluetoothDevice bluetoothDevice : devices) {
            if(bluetoothDevice.getName().contains("SMARTING")) {
                deviceList[x] = bluetoothDevice.getName();
                x++;
            }
        }
        adapter.cancelDiscovery();
        if(deviceList.length > 0)return deviceList[0];
        return null;
    }


    /**
     * Starts a background thread and its {@link Handler}.
     */
    private void startBackgroundThread() {
        mBackgroundThread = new HandlerThread("CameraBackground");
        mBackgroundThread.start();
        mBackgroundHandler = new Handler(mBackgroundThread.getLooper());
    }

    /**
     * Stops the background thread and its {@link Handler}.
     */
    private void stopBackgroundThread() {
        mBackgroundThread.quitSafely();
        try {
            mBackgroundThread.join();
            mBackgroundThread = null;
            mBackgroundHandler = null;
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Gets whether you should show UI with rationale for requesting permissions.
     *
     * @param permissions The permissions your app wants to request.
     * @return Whether you can show permission rationale UI.
     */
    private boolean shouldShowRequestPermissionRationale(String[] permissions) {
        for (String permission : permissions) {
            if (ActivityCompat.shouldShowRequestPermissionRationale(this, permission)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Requests permissions needed for recording video.
     */
    private void requestVideoPermissions() {
        if (shouldShowRequestPermissionRationale(REQUESTED_PERMISSIONS)) {
            new ConfirmationDialog().show(getSupportFragmentManager(), FRAGMENT_DIALOG);

        } else {
            ActivityCompat.requestPermissions(this, REQUESTED_PERMISSIONS, REQUEST_CODE);
        }
    }

    @Override
    public boolean dispatchKeyEvent(KeyEvent event) {
        int action = event.getAction();
        int keyCode = event.getKeyCode();
        switch (keyCode) {
            case KeyEvent.KEYCODE_VOLUME_UP:
                if (action == KeyEvent.ACTION_DOWN) {
                    if(mButtonMarker.isEnabled())
                    {
                        EEGdevice.setMarker((byte)'J',1);
                        Log.i(TAG, "Marker Sent");

                        Toast.makeText(this,"Marker Sent",Toast.LENGTH_SHORT).show();
                    }
                }
                return true;
            case KeyEvent.KEYCODE_VOLUME_DOWN:
                if (action == KeyEvent.ACTION_DOWN) {
                    if(mButtonMarker.isEnabled())
                    {
                        EEGdevice.setMarker((byte)'K',1);
                        Log.i(TAG, "Marker Sent");

                        Toast.makeText(this,"Marker Sent",Toast.LENGTH_SHORT).show();
                    }
                }
                return true;
            default:
                return super.dispatchKeyEvent(event);
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
                                           @NonNull int[] grantResults) {
        Log.d(TAG, "onRequestPermissionsResult");
        //TODO
        switch (requestCode)
        {
            case REQUEST_VIDEO_PERMISSIONS: {
                if (grantResults.length == REQUESTED_PERMISSIONS.length) {
                    for (int result : grantResults) {
                        if (result != PackageManager.PERMISSION_GRANTED) {
                            ErrorDialog.newInstance(getString(R.string.permission_request))
                                    .show(getSupportFragmentManager(), FRAGMENT_DIALOG);

                        }
                    }
                } else {
                    ErrorDialog.newInstance(getString(R.string.permission_request))
                            .show(getSupportFragmentManager(), FRAGMENT_DIALOG);
                }
            }break;

            case  WRITE_PERMISSION_CODE :{
                if (grantResults.length == write_permission.length) {
                    for (int result : grantResults) {
                        if (result != PackageManager.PERMISSION_GRANTED) {
                            ErrorDialog.newInstance(getString(R.string.permission_request))
                                    .show(getSupportFragmentManager(), FRAGMENT_DIALOG);

                        }
                    }
                } else {
                    ErrorDialog.newInstance(getString(R.string.permission_request))
                            .show(getSupportFragmentManager(), FRAGMENT_DIALOG);
                }
            }break;

            case BLUETOOTH_REQUEST_CODE:{

                if (grantResults.length == BLUETOOTH_PERMISSIONS.length) {
                    for (int result : grantResults) {
                        if (result != PackageManager.PERMISSION_GRANTED) {
                            ErrorDialog.newInstance(getString(R.string.permission_request))
                                    .show(getSupportFragmentManager(), FRAGMENT_DIALOG);

                        }
                    }
                } else {
                    ErrorDialog.newInstance(getString(R.string.permission_request))
                            .show(getSupportFragmentManager(), FRAGMENT_DIALOG);
                }

            }break;

        default:  {
            super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        }break;
        }
    }

    private boolean hasPermissionsGranted(String[] permissions) {
        for (String permission : permissions) {
            if (ActivityCompat.checkSelfPermission(this, permission)
                    != PackageManager.PERMISSION_GRANTED) {
                return false;
            }
        }
        return true;
    }

    /**
     * Tries to open a {@link CameraDevice}. The result is listened by `mStateCallback`.
     */
    @SuppressWarnings("MissingPermission")
    private void openCamera(int width, int height) {
        if (!hasPermissionsGranted(REQUESTED_PERMISSIONS)) {
            requestVideoPermissions();
            return;
        }

        if (isFinishing()) {
            return;
        }
        CameraManager manager = (CameraManager) getSystemService(Context.CAMERA_SERVICE);
        try {
            Log.d(TAG, "tryAcquire");
            if (!mCameraOpenCloseLock.tryAcquire(2500, TimeUnit.MILLISECONDS)) {
                throw new RuntimeException("Time out waiting to lock camera opening.");
            }
            String cameraId = manager.getCameraIdList()[0];

            // Choose the sizes for camera preview and video recording
            CameraCharacteristics characteristics = manager.getCameraCharacteristics(cameraId);
            StreamConfigurationMap map = characteristics
                    .get(CameraCharacteristics.SCALER_STREAM_CONFIGURATION_MAP);
            mSensorOrientation = characteristics.get(CameraCharacteristics.SENSOR_ORIENTATION);
            if (map == null) {
                throw new RuntimeException("Cannot get available preview/video sizes");
            }
            mVideoSize = chooseVideoSize(map.getOutputSizes(MediaRecorder.class));
            mPreviewSize = chooseOptimalSize(map.getOutputSizes(SurfaceTexture.class),
                    width, height, mVideoSize);

            int orientation = getResources().getConfiguration().orientation;
            if (orientation == Configuration.ORIENTATION_LANDSCAPE) {
                mTextureView.setAspectRatio(mPreviewSize.getWidth(), mPreviewSize.getHeight());
            } else {
                mTextureView.setAspectRatio(mPreviewSize.getHeight(), mPreviewSize.getWidth());
            }
            configureTransform(width, height);
            mMediaRecorder = new MediaRecorder();
            manager.openCamera(cameraId, mStateCallback, null);
        } catch (CameraAccessException e) {
            Toast.makeText(getApplicationContext()
                    , "Cannot access the camera.", Toast.LENGTH_SHORT).show();
            finish();
        } catch (NullPointerException e) {
            // Currently an NPE is thrown when the Camera2API is used but not supported on the
            // device this code runs.
            ErrorDialog.newInstance(getString(R.string.camera2_error))
                    .show(getSupportFragmentManager(), FRAGMENT_DIALOG);
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera opening.");
        }
    }

    private void closeCamera() {
        try {
            mCameraOpenCloseLock.acquire();
            closePreviewSession();
            if (null != mCameraDevice) {
                mCameraDevice.close();
                mCameraDevice = null;
            }
            if (null != mMediaRecorder) {
                mMediaRecorder.release();
                mMediaRecorder = null;
            }
        } catch (InterruptedException e) {
            throw new RuntimeException("Interrupted while trying to lock camera closing.");
        } finally {
            mCameraOpenCloseLock.release();
        }
    }

    /**
     * Start the camera preview.
     */
    private void startPreview() {
        if (null == mCameraDevice || !mTextureView.isAvailable() || null == mPreviewSize) {
            return;
        }
        try {
            closePreviewSession();
            SurfaceTexture texture = mTextureView.getSurfaceTexture();
            assert texture != null;
            texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());
            mPreviewBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_PREVIEW);

            Surface previewSurface = new Surface(texture);
            mPreviewBuilder.addTarget(previewSurface);

            mCameraDevice.createCaptureSession(Collections.singletonList(previewSurface),
                    new CameraCaptureSession.StateCallback() {

                        @Override
                        public void onConfigured(@NonNull CameraCaptureSession session) {
                            mPreviewSession = session;
                            updatePreview();
                        }

                        @Override
                        public void onConfigureFailed(@NonNull CameraCaptureSession session) {

                                Toast.makeText(getApplicationContext(),
                                        "Failed", Toast.LENGTH_SHORT).show();

                        }
                    }, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    /**
     * Update the camera preview. {@link #startPreview()} needs to be called in advance.
     */
    private void updatePreview() {
        if (null == mCameraDevice) {
            return;
        }
        try {
            setUpCaptureRequestBuilder(mPreviewBuilder);
            HandlerThread thread = new HandlerThread("CameraPreview");
            thread.start();
            mPreviewSession.setRepeatingRequest(mPreviewBuilder.build(), null, mBackgroundHandler);
        } catch (CameraAccessException e) {
            e.printStackTrace();
        }
    }

    private void setUpCaptureRequestBuilder(CaptureRequest.Builder builder) {
        builder.set(CaptureRequest.CONTROL_MODE, CameraMetadata.CONTROL_MODE_AUTO);
    }

    /**
     * Configures the necessary {@link android.graphics.Matrix} transformation to `mTextureView`.
     * This method should not to be called until the camera preview size is determined in
     * openCamera, or until the size of `mTextureView` is fixed.
     *
     * @param viewWidth  The width of `mTextureView`
     * @param viewHeight The height of `mTextureView`
     */
    private void configureTransform(int viewWidth, int viewHeight) {

        if (null == mTextureView || null == mPreviewSize ) {
            return;
        }
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        Matrix matrix = new Matrix();
        RectF viewRect = new RectF(0, 0, viewWidth, viewHeight);
        RectF bufferRect = new RectF(0, 0, mPreviewSize.getHeight(), mPreviewSize.getWidth());
        float centerX = viewRect.centerX();
        float centerY = viewRect.centerY();
        if (Surface.ROTATION_90 == rotation || Surface.ROTATION_270 == rotation) {
            bufferRect.offset(centerX - bufferRect.centerX(), centerY - bufferRect.centerY());
            matrix.setRectToRect(viewRect, bufferRect, Matrix.ScaleToFit.FILL);
            float scale = Math.max(
                    (float) viewHeight / mPreviewSize.getHeight(),
                    (float) viewWidth / mPreviewSize.getWidth());
            matrix.postScale(scale, scale, centerX, centerY);
            matrix.postRotate(90 * (rotation - 2), centerX, centerY);
        }
        mTextureView.setTransform(matrix);
    }

    private void setUpMediaRecorder() throws IOException {

        mMediaRecorder.setAudioSource(MediaRecorder.AudioSource.MIC);
        mMediaRecorder.setVideoSource(MediaRecorder.VideoSource.SURFACE);
        mMediaRecorder.setOutputFormat(MediaRecorder.OutputFormat.MPEG_4);
        if (mNextVideoAbsolutePath == null || mNextVideoAbsolutePath.isEmpty()) {
            mNextVideoAbsolutePath = FILENAME+".mp4";
        }
        mMediaRecorder.setOutputFile(mNextVideoAbsolutePath);
        mMediaRecorder.setVideoEncodingBitRate(10000000);
        mMediaRecorder.setCaptureRate(30.0);
        mMediaRecorder.setVideoFrameRate(30);
        mMediaRecorder.setVideoSize(mVideoSize.getWidth(), mVideoSize.getHeight());
        mMediaRecorder.setVideoEncoder(MediaRecorder.VideoEncoder.H264);
        mMediaRecorder.setAudioEncoder(MediaRecorder.AudioEncoder.AAC);
        int rotation = getWindowManager().getDefaultDisplay().getRotation();
        switch (mSensorOrientation) {
            case SENSOR_ORIENTATION_DEFAULT_DEGREES:
                mMediaRecorder.setOrientationHint(DEFAULT_ORIENTATIONS.get(rotation));
                break;
            case SENSOR_ORIENTATION_INVERSE_DEGREES:
                mMediaRecorder.setOrientationHint(INVERSE_ORIENTATIONS.get(rotation));
                break;
        }
        mMediaRecorder.prepare();
    }

    private void getFilePath() {

        if(!hasPermissionsGranted(write_permission))
        {

            ActivityCompat.requestPermissions(this,write_permission , WRITE_PERMISSION_CODE);

        }

        String tag =  DateFormat.format("MM-dd-yyyyy-h-mmssaa",System.currentTimeMillis())
                .toString();

            File dir = new File(Environment.getExternalStorageDirectory(), "CameraEEG");
            if(!dir.exists())
            {
                dir.mkdirs();
            }

            FILENAME = (dir.getAbsolutePath() + "/")+ tag;
            Log.e("Main File Creator","Checking File :"+FILENAME);


    }



    private void startRecordingVideo() {
        if (null == mCameraDevice || !mTextureView.isAvailable() || null == mPreviewSize) {
            return;
        }
        try {
            closePreviewSession();
            setUpMediaRecorder();
            SurfaceTexture texture = mTextureView.getSurfaceTexture();
            assert texture != null;
            texture.setDefaultBufferSize(mPreviewSize.getWidth(), mPreviewSize.getHeight());
            mPreviewBuilder = mCameraDevice.createCaptureRequest(CameraDevice.TEMPLATE_RECORD);
            List<Surface> surfaces = new ArrayList<>();

            // Set up Surface for the camera preview
            Surface previewSurface = new Surface(texture);
            surfaces.add(previewSurface);
            mPreviewBuilder.addTarget(previewSurface);

            // Set up Surface for the MediaRecorder
            Surface recorderSurface = mMediaRecorder.getSurface();
            surfaces.add(recorderSurface);
            mPreviewBuilder.addTarget(recorderSurface);

            // Start a capture session
            // Once the session starts, we can update the UI and start recording
            mCameraDevice.createCaptureSession(surfaces, new CameraCaptureSession.StateCallback() {

                @Override
                public void onConfigured(@NonNull CameraCaptureSession cameraCaptureSession) {
                    mPreviewSession = cameraCaptureSession;
                    updatePreview();

                    runOnUiThread(new Runnable() {
                        @Override
                        public void run() {
                            // UI

                            mIsRecordingVideo = true;

                                EEGdevice.startRecording(FILENAME+".bdf",true);
                                Log.i("recordingtimestamp", String.valueOf(SystemClock.uptimeMillis()));

                                while(! EEGdevice.queryStatus().isRecording())
                                    try {
                                        Thread.sleep(10);
                                    } catch (InterruptedException e) {
                                        e.printStackTrace();
                                    }

                                // Start recording
                                mMediaRecorder.start();
                                Log.i("recordingtimestamp", String.valueOf(SystemClock.uptimeMillis()));


                            //mMediaRecorder.start();


                        }
                    });
                }

                @Override
                public void onConfigureFailed(@NonNull CameraCaptureSession cameraCaptureSession) {

                        Toast.makeText(getApplicationContext(),
                                "Failed", Toast.LENGTH_SHORT).show();

                }
            }, mBackgroundHandler);
        } catch (CameraAccessException | IOException e) {
            e.printStackTrace();
        }

    }

    private void closePreviewSession() {
        if (mPreviewSession != null) {
            mPreviewSession.close();
            mPreviewSession = null;
        }
    }

    private void stopRecordingVideo() {
        // UI
        mIsRecordingVideo = false;

        // Stop recording

        new Thread(()->{

            EEGdevice.stopRecording();

        }).start();
        mMediaRecorder.stop();
        mMediaRecorder.reset();



            Toast.makeText(this, "Video saved: " + mNextVideoAbsolutePath,
                    Toast.LENGTH_LONG).show();
            Log.d(TAG, "Video saved: " + mNextVideoAbsolutePath);

        mNextVideoAbsolutePath = null;
        startPreview();
    }

    @Override
    public void onCommandExecuted(DeviceCommand deviceCommand, DeviceCommandResult deviceCommandResult) {

        if(deviceCommand.getCommandEnum()==DeviceCommandEnum.ON &&
                deviceCommandResult == DeviceCommandResult.SUCCESS)
        {
            runOnUiThread(()->{

                mConnectionButton.setText(R.string.disconnect);
                mButtonVideo.setAlpha(1f);
                mButtonVideo.setEnabled(true);

                mButtonMarker.setEnabled(true);
                mButtonMarker.setAlpha(1f);

            });
        }

        else if(deviceCommand.getCommandEnum()==DeviceCommandEnum.OFF &&
                deviceCommandResult == DeviceCommandResult.SUCCESS){

            runOnUiThread(()->{
                mConnectionButton.setText(R.string.connect);
                mButtonVideo.setAlpha(.5f);
                mButtonVideo.setEnabled(false);

                mButtonMarker.setEnabled(false);
                mButtonMarker.setAlpha(.5f);

            });
        }

    }

    @Override
    public void onInitStateChanged(boolean b) {

        if(!b) // If EEGdevice is not initialized
        {
            runOnUiThread(()->{
                mConnectionButton.setText(R.string.connect);
            });
        }

    }

    @Override
    public void onData() {

        EEGbuffer = EEGdevice.getBuffer();


    }

    /**
     * Compares two {@code Size}s based on their areas.
     */
    static class CompareSizesByArea implements Comparator<Size> {

        @Override
        public int compare(Size lhs, Size rhs) {
            // We cast here to ensure the multiplications won't overflow
            return Long.signum((long) lhs.getWidth() * lhs.getHeight() -
                    (long) rhs.getWidth() * rhs.getHeight());
        }

    }



    /**
     * initializes EEGdevice after Bluetooth is paired to Smarting device
     * */

    private void onPairedtoSmarting(String deviceName)
    {
        if(!EEGdevice.queryStatus().isAcquisition())
        {

            ArrayList<DeviceChannel> channels = new ArrayList<>();
            Collections.addAll(channels, DeviceChannel.values());

            new Thread(()->{
                for(DeviceChannel c: DeviceChannel.values()) {
                    System.out.println(c.ordinal());
                }
            }).start();


            EEGdevice.setSelectedChannels(channels);

            Log.i(TAG, "onPairedtoSmarting: initializing...");
            
            EEGdevice.init(deviceName);

            Log.i(TAG, "onPairedtoSmarting: initialized, sending commands...");
            
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.SELECT_CHANNELS,EEGdevice));
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.NORMAL));
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.HZ500));
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.IMPOFF));
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.ON));

            Log.i(TAG, "onPairedtoSmarting: commands sent successfully...");

            mConnectionButton.setText(R.string.connecting);


        }
        else {
            EEGdevice.sendCommand(new DeviceCommand(DeviceCommandEnum.OFF));
            mConnectionButton.setText(R.string.connect);
        }

    }


    private void changeSettings()
    {

        Intent intent = new Intent(this, SettingsActivity.class);

        startActivity(intent);
    }




    public static class ErrorDialog extends DialogFragment {

        private static final String ARG_MESSAGE = "message";

        public static ErrorDialog newInstance(String message) {
            ErrorDialog dialog = new ErrorDialog();
            Bundle args = new Bundle();
            args.putString(ARG_MESSAGE, message);
            dialog.setArguments(args);
            return dialog;
        }

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {
            final Activity activity = getActivity();
            return new AlertDialog.Builder(activity)
                    .setMessage(getArguments().getString(ARG_MESSAGE))
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialogInterface, int i) {
                            getActivity().finish();
                        }
                    })
                    .create();
        }

    }

    public static class ConfirmationDialog extends DialogFragment {

        @Override
        public Dialog onCreateDialog(Bundle savedInstanceState) {

            return new AlertDialog.Builder(getActivity())
                    .setMessage(R.string.permission_request)
                    .setPositiveButton(android.R.string.ok, new DialogInterface.OnClickListener() {
                        @Override
                        public void onClick(DialogInterface dialog, int which) {
                            ActivityCompat.requestPermissions(getActivity(), REQUESTED_PERMISSIONS,
                                     REQUEST_CODE);
                        }
                    })
                    .setNegativeButton(android.R.string.cancel,
                            new DialogInterface.OnClickListener() {
                                @Override
                                public void onClick(DialogInterface dialog, int which) {
                                   getActivity().finish();
                                }
                            })
                    .create();
        }

    }

  public static class ConnectionDialog extends DialogFragment
  {
      @NonNull
      @Override
      public Dialog onCreateDialog(@Nullable Bundle savedInstanceState) {

          return new AlertDialog.Builder(getActivity()).setMessage(R.string.not_paired)
                  .setPositiveButton(android.R.string.ok,null )
                  .create();
      }
  }


}






