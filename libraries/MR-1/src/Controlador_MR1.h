#include "Arduino.h"
#include <Wire.h>
#include "LittleFS.h"

// Coordenadas da posição de trabalho
#define X0 250.0
#define Y0 4.6
#define Z0 152.0
#define R0 -45.0

// Macros de plotagem
#define JUNTAS 'J'
#define CARTESIANO 'C'
#define POS 'P'
#define VEL 'V'
#define ACC 'A'

// Pino do buzzer utilizado para emitir sons
#define PINO_BUZZER 15

class BusServo
{
private:
  HardwareSerial *SerialX;

public:
  BusServo();
  BusServo(HardwareSerial &SerialPort);
  void fcn1(uint8_t id);
  void fcn2(uint8_t id);
  void fcn3(uint8_t id, uint16_t p, uint16_t t);
  void fcn4(uint8_t id);
  void fcn5(uint8_t id, int8_t a);
  void fcn6(uint8_t id);
  int fcn7(uint8_t id);
  int fcn8(uint8_t id);
  int fcn9(uint8_t id);
  int fcn10(uint8_t id);
  int fcn11(uint8_t id);
  int fcn12(uint8_t *ret, bool real);
};

class MR1
{
private:
  BusServo Servos;
  int state;
  const float h = 110.0;
  const float a1 = 48.5;
  const float d2 = 4.6;
  const float a2 = 140;
  const float a3 = 105;
  const float at = 138;
  float _jointAngles[4];
  float _jointVels[4] = { 0, 0, 0, 0 };
  float _jointAccs[4] = { 0, 0, 0, 0 };
  float _toolPose[4];
  float _toolVel[4] = { 0, 0, 0, 0 };
  float _toolAcc[4] = { 0, 0, 0, 0 };
  int _motorPos[4];
  int _motorCmd[4];
  bool MOTORS_ENABLED;
  int GRIPPER_STATE;
  int _offsetsRaw[4];
  float _offsets[4];
  int _temps[5];
  float _voltages[5];
  struct
  {
    int nWaypoints;
    float waypoints[250][4];
    bool gripperState[250];
    char segmentType[250];
    int velocity[250];
  } trajectory;
  int _memory;
  const int maxMemory = 95;
  bool readMemory();
  void sendMemory();
  void memoryScreen();
  void formatFS();
  const int jointsLowerLim[4] = { -120, 5, -150, -120 };
  const int jointsUpperLim[4] = { 120, 120, 70, 120 };
  const int mapLowerLimDeg[4] = { -120, 0, -90, -120 };
  const int mapUpperLimDeg[4] = { 120, 90, 0, 120 };
  const int mapLowerLimMotor[4] = { 0, 125, 700, 0 };
  const int mapUpperLimMotor[4] = { 1000, 500, 325, 1000 };
  float HOME_POS[4] = { 250.0, 4.6, 152.0, -45.0 };
  float HOME_POS_DEG[4] = { 0.0, 90.0, -90.0, -45.0 };
  float SLEEP_POS_DEG[4] = { 0.0, 120.0, -150.0, -30.0 };
  struct DesiredPose
  {
    bool reachable;
    float jointAngles[4];
  };
  bool isHome;
  bool isSleeping;
  char movementType;
  bool manualTeaching;
  int jogAngVel_Pe;
  int jogAngVel;
  int jogAngAcc;
  int jogLinVel_Pe;
  int jogLinVel;
  int jogLinAcc;
  const int MIN_ANG_VEL_JOG = 5;
  const int MAX_ANG_VEL_JOG = 100;
  const int MIN_ANG_ACC_JOG = 300;
  const int MAX_ANG_ACC_JOG = 800;
  const int MIN_LIN_VEL_JOG = 20;
  const int MAX_LIN_VEL_JOG = 350;
  const int MIN_LIN_ACC_JOG = 300;
  const int MAX_LIN_ACC_JOG = 1200;
  const int MIN_ANG_VEL = 50;
  const int MAX_ANG_VEL = 150;
  const int MIN_ANG_ACC = 300;
  const int MAX_ANG_ACC = 2200;
  const int MIN_ANG_VEL_C = MIN_ANG_VEL * 1.30;
  const int MAX_ANG_VEL_C = MAX_ANG_VEL * 1.30;
  const int MIN_LIN_VEL = 100;
  const int MAX_LIN_VEL = 350;
  const int MIN_LIN_ACC = 600;
  const int MAX_LIN_ACC = 2600;
  const int sampleRate = 10;
  const float sampleRateSec = float(sampleRate) / 1000.0;
  const int displayRate = 80;
  const int PLOT_LIMIT_JOINT_POS = 190;
  const int PLOT_LIMIT_JOINT_VEL = 190;
  const int PLOT_LIMIT_JOINT_ACC = 2350;
  const int PLOT_LIMIT_CART_POS = 475;
  const int PLOT_LIMIT_CART_VEL = 570;
  const int PLOT_LIMIT_CART_ACC = 7600;
  int nTraj;
  int tempReadingError = 0;
  bool fileSystem;  
  int mapFloat(int joint);
  void motorCmd(int joint);
  bool hasBegun;
  void begin();
  char progMode;
  void move();
  void moveRobot(int buttonCode);
  void moveJoint(float t, int joint, int dir, bool blendOut);
  void moveCoordinate(float t, int var, int dir, bool blendOut);
  void adjustOffset();
  void forwardKinematics();
  DesiredPose inverseKinematics(float desiredPose[4]);
  void computeCartesianVals();
  void PTP_RAW(float jFinal[4], int velocity_pe);
  void PTP_C_RAW(float jFinal[4], int velocity_pe);
  void LIN_RAW(float finalPose[4], int velocity_pe);
  void newTrajectory();
  void playTrajectory();
  void printTrajSegment();
  void printNewTrajectory();
  void initHoming();
  bool checkProximity_TP();
  bool checkProximity(float initialPose[4], float finalPose[4]);
  bool linMoveCheck(float initialPose[4], float finalPose[4], int velocity);
  bool linSegmentCheck(bool lastWaypoint);
  bool readAngles();
  bool readTemps();
  bool readOffsets();
  bool readVoltages();
  void enableMotors_TP();
  bool readMotorPos();
  bool readMoveCmd();
  void matchPositions();
  void changeVelocity(int amount);
  void insertClaw_TP();
  void removeClaw_TP();
  void emergencyStop();
  void emergencyShutDown();
  static void emergencyStopButton_ISR();
  int getTrajectories();
  bool saveTrajectory();
  bool loadTrajectory();
  void deleteTrajectory();
  int getTrajectoryIndex();
  void delAllTrajs();
  bool readTrajectory(int trajIndex);
  bool createStdTrajs();
  int requestCode();
  void sendMsg(uint8_t msg);
  void sendState(uint8_t code);
  void sendTempStop(uint8_t motor_index);
  void sendTrajVars(bool sleeping, bool tooClose, bool unreachable, bool unreachableLastWP);
  void sendAllPositions();
  void sendPosition(int index);
  void sendVelocity();
  void sendTrajectories();
  void sendOffsets();
  void sendOffset(int index);
  void sendTemps();
  void updateTemp(int index);
  void sendVoltages();
  void updateVoltage(int index);
  void sendBarProgress(uint8_t prog);
  void plotVals(float values[4], int plotLimit);
  void checkPlot();
  void enablePlot();
  void disablePlot();
  void plotJoints();
  void plotCartesian();
  void plotPositions();
  void plotVelocities();
  void plotAccelerations();
  void switchPlot();
  void zeroVals();

public:
  MR1();

  // Funções de inicialização
  void inicializar_TP();
  void inicializar_Prog();
  void exec();

  // Funções de geração de trajetória
  void PTP(float posFinal[4], int velocidade);    // Trajetória PTP trapezoidal
  void PTP_C(float posFinal[4], int velocidade);  // Trajetória PTP polinomial cúbica
  void LIN(float posFinal[4], int velocidade);    // Trajetória LIN cartesiana

  // Posições padrão
  void inicio();
  void descanso();

  // Funções da garra
  void abrirGarra();
  void fecharGarra();
  void inserirGarra();
  void removerGarra();

  // Funções dos motores
  bool ligarMotores();
  void desligarMotores();
  void pararMotores();
  void mudarDesvio(int eixo, float desvio);

  // Checagem de temperatura dos motores para evitar superaquecimento
  void checarTemp();

  // Funções de leitura que retornam os respectivos valores nas variáveis passadas como argumento
  bool lerAngulos(float angulos[4]);
  bool lerPosicao(float pos[4]);
  bool lerTemps(int temps[5]);
  bool lerVoltagens(float voltagens[5]);
  bool lerDesvios(float desvios[4]);
  void lerAngulosComp(float angulos[4]);
  void lerPosicaoComp(float pos[4]);

  // Funções de leitura que apenas plotam os respectivos valores
  void plotarAngulos();
  void plotarPosicao();
  void plotarTemps();
  void plotarVoltagens();
  void plotarDesvios();
  void plotarAngulosComp();
  void plotarPosicaoComp();

  // Variáveis e funções de plotagem
  bool plot;        // Determina se a plotagem está ativada (true) ou desativada (false). Por padrão, a plotagem inicia desativada.
  char plotEspaco;  // Determina o espaço da plotagem (JUNTAS ou CARTESIANO). O espaço padrão é o de juntas.
  char plotTipo;    // Determina o tipo de plotagem (POS = posição, VEL = velocidade, ou ACC = aceleração). O tipo padrão é o de posição.
  void resetarPlot();

  // Função de beep
  void beepErro();
};