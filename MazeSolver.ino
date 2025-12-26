#include <Arduino.h>

/* ================= CONFIG ================= */

#define MAZE_SIZE 6
#define INF 255

// Sensor pins
#define IR_FRONT A0
#define IR_LEFT  A1
#define IR_RIGHT A2

// Motor pins
#define LM_F 5
#define LM_B 6
#define RM_F 9
#define RM_B 10

/* =============== DATA STRUCTURES =============== */

struct Cell {
  uint8_t r;
  uint8_t c;
};

// Horizontal & Vertical walls
uint8_t H[MAZE_SIZE + 1][MAZE_SIZE];
uint8_t V[MAZE_SIZE][MAZE_SIZE + 1];

// Distance grid
uint8_t dist[MAZE_SIZE][MAZE_SIZE];

// Robot state
Cell robot = {2, 3};   // MATLAB [3,4] → zero indexed
Cell goal  = {5, 5};

/* =============== QUEUE FOR BFS =============== */

Cell queueCells[MAZE_SIZE * MAZE_SIZE];
uint8_t qHead, qTail;

/* =============== SETUP ================= */

void setup() {
  Serial.begin(9600);

  pinMode(IR_FRONT, INPUT);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);

  pinMode(LM_F, OUTPUT);
  pinMode(LM_B, OUTPUT);
  pinMode(RM_F, OUTPUT);
  pinMode(RM_B, OUTPUT);

  initWalls();
  Serial.println("Robot Started");
}

/* =============== MAIN LOOP ================= */

void loop() {
  if (robot.r == goal.r && robot.c == goal.c) {
    stopMotors();
    Serial.println("GOAL REACHED");
    while (1);
  }

  senseWalls();
  floodFill();
  Cell next = decideMove();
  moveToNextCell(next);
  robot = next;
}

/* =============== WALL INITIALIZATION ================= */

void initWalls() {
  memset(H, 0, sizeof(H));
  memset(V, 0, sizeof(V));

  for (int i = 0; i < MAZE_SIZE; i++) {
    H[0][i] = 1;
    H[MAZE_SIZE][i] = 1;
    V[i][0] = 1;
    V[i][MAZE_SIZE] = 1;
  }
}

/* =============== SENSING ================= */

bool wallFront() { return analogRead(IR_FRONT) > 300; }
bool wallLeft()  { return analogRead(IR_LEFT)  > 300; }
bool wallRight() { return analogRead(IR_RIGHT) > 300; }

void senseWalls() {
  int r = robot.r;
  int c = robot.c;

  // Assuming robot faces NORTH (you can extend this later)
  if (wallFront()) H[r][c] = 1;
  if (wallLeft())  V[r][c] = 1;
  if (wallRight()) V[r][c + 1] = 1;
}

/* =============== FLOOD FILL ================= */

void floodFill() {
  for (int r = 0; r < MAZE_SIZE; r++)
    for (int c = 0; c < MAZE_SIZE; c++)
      dist[r][c] = INF;

  qHead = qTail = 0;
  dist[goal.r][goal.c] = 0;
  queueCells[qTail++] = goal;

  int dr[4] = {-1, 1, 0, 0};
  int dc[4] = {0, 0, -1, 1};

  while (qHead != qTail) {
    Cell cur = queueCells[qHead++];
    int d = dist[cur.r][cur.c];

    for (int i = 0; i < 4; i++) {
      int nr = cur.r + dr[i];
      int nc = cur.c + dc[i];

      if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

      bool blocked = false;
      if (i == 0 && H[cur.r][cur.c]) blocked = true;
      if (i == 1 && H[cur.r + 1][cur.c]) blocked = true;
      if (i == 2 && V[cur.r][cur.c]) blocked = true;
      if (i == 3 && V[cur.r][cur.c + 1]) blocked = true;

      if (!blocked && dist[nr][nc] == INF) {
        dist[nr][nc] = d + 1;
        queueCells[qTail++] = {nr, nc};
      }
    }
  }
}

/* =============== DECISION ================= */

Cell decideMove() {
  Cell best = robot;
  uint8_t minVal = INF;

  int dr[4] = {-1, 1, 0, 0};
  int dc[4] = {0, 0, -1, 1};

  for (int i = 0; i < 4; i++) {
    int nr = robot.r + dr[i];
    int nc = robot.c + dc[i];
    if (nr < 0 || nr >= MAZE_SIZE || nc < 0 || nc >= MAZE_SIZE) continue;

    bool blocked = false;
    if (i == 0 && H[robot.r][robot.c]) blocked = true;
    if (i == 1 && H[robot.r + 1][robot.c]) blocked = true;
    if (i == 2 && V[robot.r][robot.c]) blocked = true;
    if (i == 3 && V[robot.r][robot.c + 1]) blocked = true;

    if (!blocked && dist[nr][nc] < minVal) {
      minVal = dist[nr][nc];
      best = {nr, nc};
    }
  }
  return best;
}

/* =============== MOVEMENT ================= */

void moveToNextCell(Cell next) {
  if (next.r < robot.r) moveNorth();
  else if (next.r > robot.r) moveSouth();
  else if (next.c > robot.c) moveEast();
  else if (next.c < robot.c) moveWest();
}

void moveNorth() { moveForward(); }
void moveSouth() { turn180(); moveForward(); }
void moveEast()  { turnRight(); moveForward(); }
void moveWest()  { turnLeft(); moveForward(); }

/* =============== MOTOR CONTROL ================= */

void moveForward() {
  digitalWrite(LM_F, HIGH);
  digitalWrite(RM_F, HIGH);
  delay(500);   // replace with encoder logic
  stopMotors();
}

void turnLeft() {
  digitalWrite(LM_B, HIGH);
  digitalWrite(RM_F, HIGH);
  delay(300);
  stopMotors();
}

void turnRight() {
  digitalWrite(LM_F, HIGH);
  digitalWrite(RM_B, HIGH);
  delay(300);
  stopMotors();
}

void turn180() {
  turnRight();
  turnRight();
}

void stopMotors() {
  digitalWrite(LM_F, LOW);
  digitalWrite(LM_B, LOW);
  digitalWrite(RM_F, LOW);
  digitalWrite(RM_B, LOW);
}
