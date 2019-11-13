int afstand = 0;

//-------The Setup routine-------
//-------------------------------
void setup() {
  configController();
}

//Motor mode variable
// motorModeControlA(mode, input)
// Stop motoren
// Mode 0: Stop
// ##
// Bræmser motoren
// Mode 1: Break
// ##
// Position i grader i forhold til nuværende position.
// Fortegn angiver regning
// Mode 2: Position
// ##
// Hastighed 1-8. Fortegn angiver regning
// Mode 3: Speed
// ##
// Hastighed 1-8, Position i grader. Fortegn angiver regning
// Mode 4: Gå til position og hastighed


void loop() {

  
    motorModeControlB(4, 1660, 4); //Motorent drejer 1660 grader med hastighed 4
    motorModeControlA(4, 660, -4); //Motorent drejer 660 grader med hastighed 4
    while (mode_B_Done == false);
    while (mode_A_Done == false); //Programmet venter her til positionen er nået
    delay(300);

    motorModeControlB(4, 1660, -4); //Motorent drejer 1660 grader med hastighed 4
    motorModeControlA(4, 660, 4); //Motorent drejer 660 grader med hastighed 4
    while (mode_B_Done == false);
    while (mode_A_Done == false); //Programmet venter her til positionen er nået
    delay(300);


    /*
      motorModeControlB(2,660); //Motorent drejer 1660 grader
      motorModeControlA(2,-360); //Motorent drejer 660 grader
      while(mode_B_Done == false);
      while(mode_A_Done == false); //Programmet venter her til positionen er nået
      delay(300);

      motorModeControlB(2,-360); //Motorent drejer 1660 grader
      motorModeControlA(2,360); //Motorent drejer 660 grader
      while(mode_B_Done == false);
      while(mode_A_Done == false); //Programmet venter her til positionen er nået
      delay(300);
  */

  /*
    motorModeControlA(3,3); //Køre motor i en retning
    motorModeControlB(3,3); //Køre motor i en retning
    delay(800);

    motorModeControlA(3,-3); //Køre motor i anden retning (direkte skift)
    motorModeControlB(3,-3); //Køre motor i anden retning (direkte skift)
    delay(800);
  */


  /*
    afstand = hentAfstand(); //Henter afstanden og gemmer den i variablen "afstand"
    Serial.print(afstand);
    Serial.println(" mm");
  */

}

