#include "Controlador_MR1.h"
MR1 robo;
void setup()

{

  robo.inicializar_Prog();

  robo.plot = true;

  robo.plotTipo = VEL;

  robo.plotEspaco = JUNTAS;

}

void loop()

{

float P0[4] = {X0, Y0, Z0, R0};

float P1[4] = {164, 226, 68, -64}; 

float P2[4] = {192, -210, 172, -2}; 

int VEL1 = 40;

int VEL2 = 50;

robo.PTP(P1, VEL1);

delay(500);

robo.PTP_C(P2, VEL2);

delay(500);

robo.PTP(P0, VEL1);

robo.checarTemp();

delay(3000);

}