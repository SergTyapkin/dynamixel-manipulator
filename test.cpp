#include <iostream>
using namespace std;
#include <stdlib.h>
#include <math.h>
#include "./Kinematics/Kinematics.h"


#define OBSTACLES_COUNT 1
const float obstacles[OBSTACLES_COUNT][2][3] = {{{-25, -25, -20}, {25, 25, 0}}};

const float startCoords[] = {30, 30, -5};
const float endCoords[] = {-30, -30, -5};


bool isPointInBlock(float x, float y, float z, float blockXMin, float blockYMin, float blockZMin, float blockXMax, float blockYMax, float blockZMax) {
  return
      (blockXMin < x && x < blockXMax) &&
      (blockYMin < y && y < blockYMax) &&
      (blockZMin < z && z < blockZMax);
}

// Возвращаемое значение - содержатся ли оба конца линии внутри параллелепипеда
bool findBlockAndLineIntersections(const float* lineStart, const float* lineEnd, const float* blockStart, const float* blockEnd, float* exportPoint1, float* exportPoint2, bool &isPoint1Inside, bool &isPoint2Inside) {
  // Сортировка координат параллелепипеда
  const float blockXMin = min(blockStart[0], blockEnd[0]);
  const float blockYMin = min(blockStart[1], blockEnd[1]);
  const float blockZMin = min(blockStart[2], blockEnd[2]);
  const float blockXMax = max(blockStart[0], blockEnd[0]);
  const float blockYMax = max(blockStart[1], blockEnd[1]);
  const float blockZMax = max(blockStart[2], blockEnd[2]);

  const float deltaLength = 0.1;
  // Получение вектора из начала в конец линии
  float dx = (lineEnd[0] - lineStart[0]);
  float dy = (lineEnd[1] - lineStart[1]);
  float dz = (lineEnd[2] - lineStart[2]);
  // Нормирование вектора, чтобы его длина была равна deltaLength
  const float lineLength = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
  const float coeff = lineLength / deltaLength;
  dx /= coeff;
  dy /= coeff;
  dz /= coeff;

  // Теперь просто идём по чуть-чуть из начала в конец линии, пока не найдем точку внутри прямоугольника. Это вход. Потом найдём точку вне прямоугольника. Это выходная точка.
  const unsigned iterationsCount = lineLength / deltaLength;
  bool isPoint1Found = false;
  bool isPoint2Found = false;
  isPoint1Inside = isPointInBlock(lineStart[0], lineStart[1], lineStart[2], blockXMin, blockYMin, blockZMin, blockXMax, blockYMax, blockZMax);
  isPoint2Inside = isPointInBlock(lineEnd[0], lineEnd[1], lineEnd[2], blockXMin, blockYMin, blockZMin, blockXMax, blockYMax, blockZMax);;
  bool isPrevPointIntersect = isPrevPointIntersect;
  for (unsigned i = 0; i < iterationsCount; i++) {
    float x = lineStart[0] + dx * i;
    float y = lineStart[1] + dy * i;
    float z = lineStart[2] + dz * i;
    bool isCurPointIntersect = isPointInBlock(x, y, z, blockXMin, blockYMin, blockZMin, blockXMax, blockYMax, blockZMax);
    // если точка внутри параллелепипеда
    if (
        isCurPointIntersect &&
        !isPrevPointIntersect &&
        !isPoint1Found
        ) {
      // сохраняем точку между текущей и предыдущей, как входную
      exportPoint1[0] = x - dx / 2;
      exportPoint1[1] = y - dy / 2;
      exportPoint1[2] = z - dz / 2;
      isPoint1Found = true;
    }
    if (
        !isCurPointIntersect &&
        isPrevPointIntersect &&
        !isPoint2Found
        ) {
      // сохраняем точку между текущей и предыдущей, как выходную
      exportPoint2[0] = x - dx / 2;
      exportPoint2[1] = y - dy / 2;
      exportPoint2[2] = z - dz / 2;
      isPoint2Found = true;
    }

    isPrevPointIntersect = isCurPointIntersect;
  }

  return isPoint1Found || isPoint2Found;
}


float** _allocateTrajectoryArray(size_t size) {
  float** arr = (float**)malloc(size * sizeof(float*));
  for (unsigned i = 0; i < size; i++) {
    arr[i] = (float*)malloc(3 * sizeof(float));
  }
  return arr;
}

float** generateWay(const float* start, const float* end, float** &way, unsigned &waypointsCount) {
  way = NULL;
  waypointsCount = 0;

  for (unsigned obstacleIdx = 0; obstacleIdx < OBSTACLES_COUNT; obstacleIdx++) {
    float point1[3];
    float point2[3];
    bool isPoint1Inside;
    bool isPoint2Inside;
    bool isIntersect = findBlockAndLineIntersections(start, end, obstacles[obstacleIdx][0], obstacles[obstacleIdx][1], point1, point2, isPoint1Inside, isPoint2Inside);

    if (isPoint1Inside && isPoint2Inside) { // Если обе точки внутри
      waypointsCount = 0;
      way = _allocateTrajectoryArray(waypointsCount);
      return way;
    } else if (isPoint2Inside) {  // если конечная точка внутри препятствия
      waypointsCount = 2;
      way = _allocateTrajectoryArray(waypointsCount);
      way[0][0] = start[0];
      way[0][1] = start[1];
      way[0][2] = start[2];

      way[1][0] = point2[0];
      way[1][1] = point2[1];
      way[1][2] = point2[2];
      return way;
    } else if (isPoint1Inside) {  // если начальная точка внутри препятствия
      waypointsCount = 2;
      way = _allocateTrajectoryArray(waypointsCount);
      way[0][0] = point1[0];
      way[0][1] = point1[1];
      way[0][2] = point1[2];

      way[1][0] = end[0];
      way[1][1] = end[1];
      way[1][2] = end[2];
      return way;
    } else { // Обе точки вне блока
      // пересечений нет
      if (!isIntersect) {
        waypointsCount = 2;
        way = _allocateTrajectoryArray(waypointsCount);
        way[0][0] = start[0];
        way[0][1] = start[1];
        way[0][2] = start[2];

        way[1][0] = end[0];
        way[1][1] = end[1];
        way[1][2] = end[2];
        return way;
      }
    }
    // Пересечения два

    // Если пересекается, нужно сместить эти точки в ближайшие к ним вершины параллелепипеда
    // Сортировка координат параллелепипеда
    const float xMin = min(obstacles[obstacleIdx][0][0], obstacles[obstacleIdx][1][0]);
    const float yMin = min(obstacles[obstacleIdx][0][1], obstacles[obstacleIdx][1][1]);
    const float zMin = min(obstacles[obstacleIdx][0][2], obstacles[obstacleIdx][1][2]);
    const float xMax = max(obstacles[obstacleIdx][0][0], obstacles[obstacleIdx][1][0]);
    const float yMax = max(obstacles[obstacleIdx][0][1], obstacles[obstacleIdx][1][1]);
    const float zMax = max(obstacles[obstacleIdx][0][2], obstacles[obstacleIdx][1][2]);

    const float vertices[8][3] = {
        {xMin, yMin, zMin}, {xMax, yMin, zMin}, {xMin, yMax, zMin}, {xMax, yMax, zMin},
        {xMin, yMin, zMax}, {xMax, yMin, zMax}, {xMin, yMax, zMax}, {xMax, yMax, zMax}
    };
    // Ищем индексы ближайших вершин к каждой из точек
    float minDist1 = 999999999;
    float minDist2 = 999999999;
    unsigned minVertexIdx1;
    unsigned minVertexIdx2;
    for (unsigned i = 0; i < 8; i++) {
      const float x = vertices[i][0];
      const float y = vertices[i][1];
      const float z = vertices[i][2];
      const float dist1 = sqrt(pow(x - point1[0], 2) + pow(y - point1[1], 2) + pow(z - point1[2], 2));
      const float dist2 = sqrt(pow(x - point2[0], 2) + pow(y - point2[1], 2) + pow(z - point2[2], 2));

      if (dist1 < minDist1) {
        minDist1 = dist1;
        minVertexIdx1 = i;
      }
      if (dist2 < minDist2) {
        minDist2 = dist2;
        minVertexIdx2 = i;
      }
    }

    #define i1 minVertexIdx1
    #define i2 minVertexIdx2
    const float* i1Coord = vertices[minVertexIdx1];
    const float* i2Coord = vertices[minVertexIdx2];
    #define indexesIs(idx1, idx2) (min(i1, i2) == idx1 && max(i1, i2) == idx2)
    // Есть три варианта.
    // 1. Это одна и та же точка. i1 = i2
    // 2. Вершины соединены ребром. i1 = i2 +- 1 || (0, 2) || (4, 6) || i1 = i2 +- 4
    // 3. Вершины находятся на одной грани. (0, 5) (1, 4) (1, 7) (3, 5) (3, 6) (2, 7) (2, 4) (0, 6) (0, 3) (1, 2) (4, 7) (5, 6)
    // 4. Вершины противоположны, на диагонали параллелепипеда (0, 7) (1, 6) (3, 4) (2, 5)
    if (i1 == i2) {  // 1-ый случай
      waypointsCount = 3;
      way = _allocateTrajectoryArray(waypointsCount);
      way[1][0] = i1Coord[0];
      way[1][1] = i1Coord[1];
      way[1][2] = i1Coord[2];
    } else if (indexesIs(0, 7) || indexesIs(1, 6) || indexesIs(3, 4) || indexesIs(2, 5)) {  // 4-ый случай
      waypointsCount = 5;
      way = _allocateTrajectoryArray(waypointsCount);
      way[1][0] = i1Coord[0];
      way[1][1] = i1Coord[1];
      way[1][2] = i1Coord[2];

      unsigned centralVertexIdx;
      if (indexesIs(0, 7))
        centralVertexIdx = 4;
      else if (indexesIs(1, 6))
        centralVertexIdx = 5;
      else if (indexesIs(3, 4))
        centralVertexIdx = 7;
      else // (2, 5)
        centralVertexIdx = 6;
      way[2][0] = vertices[centralVertexIdx][0];
      way[2][1] = vertices[centralVertexIdx][1];
      way[2][2] = vertices[centralVertexIdx][2];

      way[3][0] = i2Coord[0];
      way[3][1] = i2Coord[1];
      way[3][2] = i2Coord[2];
    } else { // 2-ой случай или 3-ий случай
      waypointsCount = 4;
      way = _allocateTrajectoryArray(waypointsCount);
      way[1][0] = i1Coord[0];
      way[1][1] = i1Coord[1];
      way[1][2] = i1Coord[2];

      way[2][0] = i2Coord[0];
      way[2][1] = i2Coord[1];
      way[2][2] = i2Coord[2];
    }

    // первая точка
    way[0][0] = start[0];
    way[0][1] = start[1];
    way[0][2] = start[2];
    // последняя точка
    way[waypointsCount-1][0] = end[0];
    way[waypointsCount-1][1] = end[1];
    way[waypointsCount-1][2] = end[2];
    return way;
  }

  // пересечений с объектами не найдено. Просто копируем начало и конец
  waypointsCount = 2;
  way = _allocateTrajectoryArray(waypointsCount);
  way[0][0] = start[0];
  way[0][1] = start[1];
  way[0][2] = start[2];

  way[1][0] = end[0];
  way[1][1] = end[1];
  way[1][2] = end[2];
  return way;
}


void freeWayMemory(float** &way, unsigned &waypointsCount) {
  for (unsigned i = 0; i < waypointsCount; i++) {
    free(way[i]);
  }
  free(way);
  way = NULL;
  waypointsCount = 0;
}



pos startPositions[JOINTS_COUNT] = {};
pos endPositions[JOINTS_COUNT] = {};

float** trajectory;
unsigned trajectoryKeypointsCount;
pos** anglesTrajectory;

int main() {
  generateWay(startCoords, endCoords,  trajectory, trajectoryKeypointsCount);
  anglesTrajectory = (pos**)malloc(trajectoryKeypointsCount * sizeof(pos*));
  for (unsigned i = 0; i < trajectoryKeypointsCount; i++) {
    anglesTrajectory[i] = (pos*)calloc(JOINTS_COUNT, sizeof(pos));
    getAnglesByTargetPoint(trajectory[i][0], trajectory[i][1], trajectory[i][2], anglesTrajectory[i], anglesTrajectory[i]);
  }

  for (unsigned i = 0; i < trajectoryKeypointsCount; i++) {
    cout << "[[" << i << "]]" << endl;
    cout << "Point: " << trajectory[i][0] << ", " << trajectory[i][1] << ", " << trajectory[i][2] << endl;
    cout << "Angles: ";
    for (unsigned c = 0; c < JOINTS_COUNT; c++)
      cout << anglesTrajectory[i][c] << " ";
    cout << endl;
  }
  return 0;
}
