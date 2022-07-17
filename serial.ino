
String commandbuffer;
String valuebuffer;
bool valueflag;

String commands[17] =  
{
  "Kpy",
  "Kiy",
  "Kdy",
  "Kpp",
  "Kip",
  "Kdp",
  "Kpr",
  "Kir",
  "Kdr",
  "Kd2y",
  "Kd2p",
  "Kd2r",
  "cal",
  "yaw",
  "pit",
  "rol",
  "thr"
};

void nacitaj()
{
  while (Serial3.available())
  {
    char c = Serial3.read();
    if (c == '\n')
    {
      spracovanie();
      valuebuffer = "";
      commandbuffer = "";
      valueflag = 0;
    }
    else if (c == '=')
      valueflag = 1;

    else if (valueflag == 1)
      valuebuffer += c;

    else
      commandbuffer += c;
  }
}

void spracovanie()
{
  int command_num;

  for (command_num = 16; (commands[command_num] != commandbuffer) && (command_num != -1); command_num--)
    ;
  if (command_num >= 0)
  {
    lc_time=millis();
    
    if (valuebuffer.length() == 0 && command_num < 12)
    {
      // vypis
      Serial.print("(len zobrazenie) " + commands[command_num] + "=");
      Serial.println(value[command_num]); //vypíše hodnotu kpr
    }
    else
    {
      // zmena
      float pom = valuebuffer.toFloat();
      if (command_num >= 12){
        value[command_num] = pom;
//  Serial.print("zapis");
//  Serial.println(THROTTLE);  
}
      else
      {
        if (pom == 0)
        {
          Serial.println("chyba");
        }
        else
        {
          value[command_num] = pom;
          EEPROM.put((command_num * 4), pom);
          Serial.print("priradenie " +commands[command_num] + "=");
          Serial.println(pom);
        }
      }
    }
  }
  else 
  {
    Serial.print("Nesprávny príkaz:"); 
    Serial.println(commandbuffer);
  }
}
