package com.led_on_off.led;

import android.bluetooth.BluetoothSocket;
import android.content.Intent;
import android.hardware.SensorEventListener;
import android.net.Uri;
import android.os.Bundle;
import android.support.v7.app.ActionBarActivity;
import android.view.View;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.TextView;

public class AboutActivity extends ActionBarActivity
{

    private TextView text;
    private CheckBox yaw,pitch, roll;
    BluetoothSocket btSocket = null;

    @Override
    protected void onCreate(Bundle savedInstanceState)
    {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_about);
        text = (TextView) findViewById(R.id.text);
        yaw = (CheckBox) findViewById(R.id.Yaw);
        pitch = (CheckBox) findViewById(R.id.Pitch);
        roll = (CheckBox) findViewById(R.id.Roll);

    }

    public  void control(View v)
    {

        Intent i = new Intent(this, ledControl.class);
        startActivity(i);

    }

    public  void send(View v)
    {
        StringBuffer stringBuffer = new StringBuffer();

        String p= ((EditText) findViewById(R.id.number1)).getText().toString();
        String i= ((EditText) findViewById(R.id.number2)).getText().toString();
        String d= ((EditText) findViewById(R.id.number3)).getText().toString();

        if (yaw.isChecked()){
            stringBuffer.append("Kpy=" + p + "\n" + "Kiy=" + i + "\n" +"Kdy=" + d + "\n");
        }
        if (pitch.isChecked()){
            stringBuffer.append("Kpp=" + p + "\n" + "Kip=" + i + "\n" +"Kdp=" + d + "\n");
        }
        if (roll.isChecked()){
            stringBuffer.append("Kpr=" + p + "\n" + "Kir=" + i + "\n" +"Kdr=" + d + "\n");
        }
        text.setText(stringBuffer);
        ledControl.string2=stringBuffer.toString();
        //btSocket.getOutputStream().write(stringBuffer.toString().getBytes());
        stringBuffer.delete(0, stringBuffer.length());
    }
}













