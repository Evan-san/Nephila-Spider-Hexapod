#ifndef IK_H
#define IK_H

#include <math.h>

// ============================================================
// PANJANG LINK (mm) — sesuaikan kalau beda dari ini
// ============================================================
const float L_COXA  = 25.0;
const float L_FEMUR = 45.0;
const float L_TIBIA = 85.0;

// ============================================================
// SUDUT MOUNTING TIAP KAKI (derajat), diukur dari sumbu X body
// (arah "depan" robot), berlawanan arah jarum jam.
// Index 0 = leg1, ..., index 5 = leg6
// ============================================================
const float legMountAngle[6] = {45, 90, 135, 225, 270, 315};

struct LegAngles {
  float coxa;    // derajat, relatif terhadap arah "keluar" kaki itu sendiri
  float femur;   // derajat, 0 = femur segaris horizontal dengan coxa
  float tibia;   // derajat, sudut interior antara femur dan tibia
  bool  valid;   // false = target di luar jangkauan (workspace) kaki
};

inline float degToRad(float d) { return d * PI / 180.0; }
inline float radToDeg(float r) { return r * 180.0 / PI; }

// ------------------------------------------------------------
// legIK()
// legIndex : 0..5  (leg1..leg6)
// x, y, z  : posisi target ujung kaki (foot tip) dalam FRAME BODY
//            (mm), relatif terhadap pusat body.
//            x = depan, y = kiri, z = atas
//
// Mengembalikan sudut coxa/femur/tibia dalam "sudut kinematik murni"
// (belum dikonversi ke sudut servo — lihat catatan kalibrasi di bawah).
// ------------------------------------------------------------
LegAngles legIK(int legIndex, float x, float y, float z) {
  LegAngles result;
  result.valid = true;

  // 1) Putar target dari frame body ke frame lokal kaki, supaya
  //    "arah depan" versi kaki ini sejajar sumbu-x lokalnya.
  float mount = degToRad(legMountAngle[legIndex]);
  float lx =  x * cos(mount) + y * sin(mount);
  float ly = -x * sin(mount) + y * cos(mount);
  float lz =  z;

  // 2) Coxa: sudut yaw supaya coxa mengarah ke target
  float coxaAngle = atan2(ly, lx);

  // 3) Jarak horizontal dari sumbu coxa ke target, dikurangi
  //    panjang link coxa itu sendiri
  float horizDist = sqrt(lx * lx + ly * ly) - L_COXA;

  // 4) Jarak garis lurus dari sendi femur ke target
  float dist = sqrt(horizDist * horizDist + lz * lz);

  // 5) Cek keterjangkauan (workspace check)
  float maxReach = L_FEMUR + L_TIBIA;
  float minReach = fabs(L_FEMUR - L_TIBIA);
  if (dist > maxReach || dist < minReach) {
    result.valid = false;
    dist = constrain(dist, minReach + 0.01, maxReach - 0.01);
  }

  // 6) Femur = sudut elevasi ke target + sudut dalam segitiga femur-tibia-dist
  float elevation = atan2(lz, horizDist);
  float cosBeta = (L_FEMUR * L_FEMUR + dist * dist - L_TIBIA * L_TIBIA) / (2 * L_FEMUR * dist);
  cosBeta = constrain(cosBeta, -1.0, 1.0);
  float beta = acos(cosBeta);
  float femurAngle = elevation + beta;

  // 7) Tibia = sudut dalam segitiga femur-tibia-dist (hukum cosinus)
  float cosGamma = (L_FEMUR * L_FEMUR + L_TIBIA * L_TIBIA - dist * dist) / (2 * L_FEMUR * L_TIBIA);
  cosGamma = constrain(cosGamma, -1.0, 1.0);
  float tibiaAngle = acos(cosGamma);

  result.coxa  = radToDeg(coxaAngle);
  result.femur = radToDeg(femurAngle);
  result.tibia = radToDeg(tibiaAngle);
  return result;
}

#endif