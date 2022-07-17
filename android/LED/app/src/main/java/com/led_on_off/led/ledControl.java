package com.led_on_off.led;

import android.hardware.SensorEventListener;
import android.support.v7.app.ActionBarActivity;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuItem;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorManager;
import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.view.View;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.Toast;
import android.app.ProgressDialog;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.os.AsyncTask;

import java.io.IOException;
import java.util.UUID;


public class ledControl extends ActionBarActivity implements SensorEventListener {

    // Button btnOn, btnOff, btnDis;
    ImageButton On, Off, Discnt, Abt;
    Button b_nuly, b_calibrate;
    SeekBar s_plyn;
    String address = null;
    private ProgressDialog progress;
    BluetoothAdapter myBluetooth = null;
    BluetoothSocket btSocket = null;
    public static String string2=null;
    private boolean isBtConnected = false;
    //SPP UUID. Look for it
    static final UUID myUUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");
    private TextView T1, T2, T3, T4;
    String string;
    boolean nuly=false, calibrate=false;
    private SensorManager SM;

    public static boolean sending;

    float Rot[]=null; //for gravity rotational data
    //don't use R because android uses that for other stuff
    float I[]=null; //for magnetic rotational data
    float accels[]=new float[3];
    float mags[]=new float[3];
    float[] values = new float[3];
int plyn;
    public static float azimuth,pitch,roll;
    Thread t;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {
        super.onCreate(savedInstanceState);

        Intent newint = getIntent();
        address = newint.getStringExtra(DeviceList.EXTRA_ADDRESS); //receive the address of the bluetooth device

        //view of the ledControl
        setContentView(R.layout.activity_led_control);
        SM = (SensorManager)getSystemService(SENSOR_SERVICE);
        //mySensor = SM.getDefaultSensor(Sensor.TYPE_ORIENTATION);
        //SM.registerListener((SensorEventListener) this, mySensor, 1);


        T1 = (TextView) findViewById(R.id.text1);
        T2 = (TextView) findViewById(R.id.text2);
        T3 = (TextView) findViewById(R.id.text3);
        T4 = (TextView) findViewById(R.id.text4);
        //call the widgets

        On = (ImageButton)findViewById(R.id.on);
        Off = (ImageButton)findViewById(R.id.off);
        Discnt = (ImageButton)findViewById(R.id.discnt);
        Abt = (ImageButton)findViewById(R.id.abt);

        b_nuly = (Button) findViewById(R.id.nuly);
        b_calibrate = (Button) findViewById(R.id.calibrate);
        s_plyn = (SeekBar) findViewById(R.id.plyn);
        s_plyn.setOnSeekBarChangeListener(seekBarChangeListener);
        uppdateNow();

        sending = true;
        new ConnectBT().execute(); //Call the class to connect

        t=new Thread(){
            @Override
            public void run(){
                while(!isInterrupted()){
                    try {
                        Thread.sleep(50);
                        runOnUiThread(new Runnable() {
                            @Override
                            public void run() {
                                if (btSocket!=null)
                                {
                                    try
                                    {
                                       if (sending) {
                                           if (string2 != null) {
                                               btSocket.getOutputStream().write(string2.getBytes());
                                               string2 = null;
                                           }
                                           if (calibrate)
                                           {
                                               btSocket.getOutputStream().write(("cal=" + 1 + "\n").getBytes());
                                               calibrate=false;
                                           }
                                           if (!nuly){
                                               btSocket.getOutputStream().write(string.getBytes());
                                                T1.setText("Azimuth: " + azimuth);
                                                T2.setText("Pitch: " + pitch);
                                                T3.setText("Roll: " + roll);
                                                }
                                           if (nuly){
                                                btSocket.getOutputStream().write(("yaw=0" + "\n" + "pit=0" + "\n" + "rol=0" + "\n").getBytes());
                                                T1.setText("Azimuth: " + 0);
                                                T2.setText("Pitch: " + 0);
                                                T3.setText("Roll: " + 0);
                                                }
                                           btSocket.getOutputStream().write(("thr=" + plyn + "\n").getBytes());
                                           T4.setText("Plyn: " + plyn);
                                       }

                                    }
                                    catch (IOException e)
                                    {
                                    }
                                }
                            }
                        });
                    } catch (InterruptedException e) {
                        e.printStackTrace();
                    }
                }
            }
        };


        //commands to be sent to bluetooth

        b_nuly.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                nuly= !nuly;
                // Code here executes on main thread after user presses button
            }
        });

        b_calibrate.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                calibrate=true;
                // Code here executes on main thread after user presses button
            }
        });

        On.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                turnOnLed();      //method to turn on
            }
        });

        Off.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v)
            {
                turnOffLed();   //method to turn off
            }
        });

        Discnt.setOnClickListener(new View.OnClickListener()
        {
            @Override
            public void onClick(View v)
            {
                Disconnect(); //close connection
            }
        });

//        Handler handler = new Handler();
//        handler.postDelayed(new Runnable() {
//            @Override
//            public void run() {
//
//                T1.setText("X: " + x);
//                T2.setText("Y: " + y);
//                //T3.setText("Z: " + z);
//
//            }
//        },1000);
    }
    private SeekBar.OnSeekBarChangeListener seekBarChangeListener = new SeekBar.OnSeekBarChangeListener() {
        @Override
        public void onProgressChanged(SeekBar seekBar, int i, boolean b) {
            uppdateNow();
        }

        @Override
        public void onStartTrackingTouch(SeekBar seekBar) {

        }

        @Override
        public void onStopTrackingTouch(SeekBar seekBar) {

        }
    };
    private void uppdateNow() {plyn = s_plyn.getProgress();}

    @Override
    protected void onResume()
    {
        super.onResume();
    /*register the sensor listener to listen to the gyroscope sensor, use the
    callbacks defined in this class, and gather the sensor information as quick
    as possible*/

        SM.registerListener(this, SM.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),5000);
        SM.registerListener(this, SM.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),5000);
    }
    public void Disconnect()
    {
        if (btSocket!=null) //If the btSocket is busy
        {
            try
            {
                btSocket.close(); //close connection
            }
            catch (IOException e)
            {                                        msg("Error");
            }
        }
        finish(); //return to the first layout

    }

    private void turnOffLed()
    {
        sending = false;

    }

    private void turnOnLed()
    {
        sending = true;
        t.start();

//        if (btSocket!=null)
//        {
//            try
//            {
//                btSocket.getOutputStream().write(string.getBytes());
//            }
//            catch (IOException e)
//            {
//                msg("Error");
//            }
//        }
    }

    // fast way to call Toast
    private void msg(String s)
    {
        Toast.makeText(getApplicationContext(),s,Toast.LENGTH_LONG).show();
    }

    public  void about(View v)
    {
        if(v.getId() == R.id.abt)
        {
            Intent i = new Intent(this, AboutActivity.class);
            startActivity(i);
        }
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        // Inflate the menu; this adds items to the action bar if it is present.
        getMenuInflater().inflate(R.menu.menu_led_control, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        // Handle action bar item clicks here. The action bar will
        // automatically handle clicks on the Home/Up button, so long
        // as you specify a parent activity in AndroidManifest.xml.
        int id = item.getItemId();

        //noinspection SimplifiableIfStatement
        if (id == R.id.action_settings) {
            return true;
        }

        return super.onOptionsItemSelected(item);
    }



    private class ConnectBT extends AsyncTask<Void, Void, Void>  // UI thread
    {
        private boolean ConnectSuccess = true; //if it's here, it's almost connected

        @Override
        protected void onPreExecute()
        {
            progress = ProgressDialog.show(ledControl.this, "Connecting...", "Please wait!!!");  //show a progress dialog
        }

        @Override
        protected Void doInBackground(Void... devices) //while the progress dialog is shown, the connection is done in background
        {
            try
            {
                if (btSocket == null || !isBtConnected)
                {
                    myBluetooth = BluetoothAdapter.getDefaultAdapter();//get the mobile bluetooth device
                    BluetoothDevice dispositivo = myBluetooth.getRemoteDevice(address);//connects to the device's address and checks if it's available
                    btSocket = dispositivo.createInsecureRfcommSocketToServiceRecord(myUUID);//create a RFCOMM (SPP) connection
                    BluetoothAdapter.getDefaultAdapter().cancelDiscovery();
                    btSocket.connect();//start connection
                }
            }
            catch (IOException e)
            {
                ConnectSuccess = false;//if the try failed, you can check the exception here
            }
            return null;
        }
        @Override
        protected void onPostExecute(Void result) //after the doInBackground, it checks if everything went fine
        {
            super.onPostExecute(result);

            if (!ConnectSuccess)
            {
                msg("Connection Failed. Is it a SPP Bluetooth? Try again.");
                finish();
            }
            else
            {
                msg("Connected.");
                isBtConnected = true;
                t.start();

            }
            progress.dismiss();
        }
    }
    //@TargetApi(Build.VERSION_CODES.CUPCAKE)
    @Override
    public void onSensorChanged(SensorEvent event)
    {
        //below commented code - junk - unreliable is never populated
        //if sensor is unreliable, return void
        //if (event.accuracy == SensorManager.SENSOR_STATUS_UNRELIABLE)
        //{
        //    return;
        //}
        if(sending) {
            switch (event.sensor.getType()) {
                case Sensor.TYPE_MAGNETIC_FIELD:
                    mags = event.values.clone();
                    break;
                case Sensor.TYPE_ACCELEROMETER:
                    accels = event.values.clone();
                    break;
            }

            if (mags != null && accels != null) {
                Rot = new float[9];
                I = new float[9];
                SensorManager.getRotationMatrix(Rot, I, accels, mags);
                // Correct if screen is in Landscape

                float[] outR = new float[9];
                SensorManager.remapCoordinateSystem(Rot, SensorManager.AXIS_X, SensorManager.AXIS_Y, outR);
                SensorManager.getOrientation(outR, values);
                ;

                azimuth = values[0] * 57.2957795f; //looks like we don't need this one
                pitch = (values[1] * 57.2957795f);
                roll = (values[2] * 57.2957795f);

                string =("yaw=" + azimuth + "\n" + "pit=" + pitch + "\n" +"rol=" +roll + "\n");
//                if (btSocket!=null)
//                {
//                    try
//                    {
//                        btSocket.getOutputStream().write(string.getBytes());
//                    }
//                    catch (IOException e)
//                    {
//                        msg("Error");
//                    }
//                }

                mags = null; //retrigger the loop when things are repopulated
                accels = null; ////retrigger the loop when things are repopulated
            }
        }

    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int i) {

    }
}
