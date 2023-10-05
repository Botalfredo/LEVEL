//=========================================================
//  _______        _   __ 
// |__   __|      | | /_ |
//    | | __ _ ___| | _| |
//    | |/ _` / __| |/ / |
//    | | (_| \__ \   <| |
//    |_|\__,_|___/_|\_\_|
//                                              
//=========================================================

void Task1code( void * pvParameters ){  
  Serial.println("Task1");
  
  vTaskDelay(3000);
  
  task1timer = millis();
  long task1timer1 = millis();
    while(1){
      //Serial.println("Task1");
      task1frec++;
      
      //ReadPPM();
      //ReadIBUS();
      RCState = 1;
      
      if (millis() - task1timer   >= 200){
        task1timer = millis();
        RightMessage();
      }
      else if (millis() - task1timer1   >= 800){
        task1timer1 = millis();
        while (ss.available() > 0)
          gps.encode(ss.read());
            displayInfo();

      //ReadGPS();
      }
      
      
      //ReadPPM();
      
      DefaultDetection();
      CalculFrec();

      vTaskDelay(1);     
    } 
}
