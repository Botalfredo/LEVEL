//=========================================================
//  _______        _    ___  
// |__   __|      | |  |__ \ 
//    | | __ _ ___| | __  ) |
//    | |/ _` / __| |/ / / / 
//    | | (_| \__ \   < / /_ 
//    |_|\__,_|___/_|\_\____|
//  
//=========================================================

void Task2code( void * pvParameters ){
  Serial.println("Task2");
  vTaskDelay(3000);
  //AfficherOLED();
  task2timer = millis();
  long task2timer1 = millis();
  while(1){
    //Serial.println("Task2");
    
    task2frec++;
    
    if(mpuInterrupt==true)
      ReadMPU();

    if (millis()-task2timer >= 300){
      task2timer = millis();
      ReadBMP();
    }
    if (millis()-task2timer1 >= 1000){
      AfficherOLED();
      
      task2timer1 = millis();
    }
    
    if(error == true){
        AfficherOLED();
        error = false;
    }
    RightServo();
    vTaskDelay(1);
  } 
}
