// ═══════════════════════════════════════════════════════════════════════════
//  SMART AUTONOMOUS CAR — Full Arduino Nano Code
//  Features: Line Following | Obstacle Avoidance | GPS Navigation | EKF
//            nRF24L01 RF Telemetry → MATLAB Ground Station
//
//  Supervised by: Dr. Ahmed
//  Hardware: Arduino Nano + MPU-6050 + NEO-6M + HC-SR04 + 3x IR + L298N
// ═══════════════════════════════════════════════════════════════════════════

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <math.h>

// ───────────────────────────────────────────────────────────────────────────
//  PIN DEFINITIONS
// ───────────────────────────────────────────────────────────────────────────

// Ultrasonic (HC-SR04)
#define TRIG_PIN      2
#define ECHO_PIN      3

// IR Line Sensors (LOW = on black line)
#define IR_LEFT       4
#define IR_MID        5
#define IR_RIGHT      6

// nRF24L01
#define RF_CE         7
#define RF_CSN        8

// Motor Driver (L298N)
#define ENA           9   // PWM — Left motor speed
#define ENB           10  // PWM — Right motor speed
#define IN1           A0  // Left motor direction
#define IN2           A1  // Right motor direction
#define IN3           A2  // Right motor direction
#define IN4           A3  // Right motor direction

// GPS SoftwareSerial
#define GPS_RX        11  // GPS TX  → Nano D11
#define GPS_TX        12  // Nano D12 → GPS RX  (unused but declared)

// I2C: A4=SDA, A5=SCL (MPU-6050) — hardware, no define needed

// ───────────────────────────────────────────────────────────────────────────
//  MOTOR SPEED CONSTANTS
// ───────────────────────────────────────────────────────────────────────────
#define BASE_SPEED    160   // Normal forward speed (0–255)
#define TURN_SPEED    110   // Slower side during line-follow turn
#define GPS_SPEED     140   // Speed during GPS navigation
#define REVERSE_SPEED 150   // Speed during obstacle reverse

// ───────────────────────────────────────────────────────────────────────────
//  OBSTACLE AVOIDANCE THRESHOLDS
// ───────────────────────────────────────────────────────────────────────────
#define OBSTACLE_CM   20    // Stop and avoid if object closer than this
#define CLEAR_CM      35    // Resume if path is clear beyond this

// ───────────────────────────────────────────────────────────────────────────
//  GPS WAYPOINT TARGET  ← change to your target coordinates
// ───────────────────────────────────────────────────────────────────────────
#define TARGET_LAT    30.044500
#define TARGET_LON    31.235800
#define ARRIVE_DIST_M 1.5    // Consider arrived within 1.5 m

// ───────────────────────────────────────────────────────────────────────────
//  RF TELEMETRY RATE
// ───────────────────────────────────────────────────────────────────────────
#define TX_INTERVAL_MS  100   // Transmit every 100 ms (10 Hz)

// ═══════════════════════════════════════════════════════════════════════════
//  OBJECTS
// ═══════════════════════════════════════════════════════════════════════════

RF24           radio(RF_CE, RF_CSN);
const byte     RF_ADDR[6] = "CAR01";   // Must match receiver sketch

MPU6050        imu;
TinyGPSPlus    gps;
SoftwareSerial gpsSerial(GPS_RX, GPS_TX);

// ═══════════════════════════════════════════════════════════════════════════
//  RF DATA PACKET  (7 floats = 28 bytes — fits nRF24L01 32-byte payload)
// ═══════════════════════════════════════════════════════════════════════════
struct DataPacket {
    float lat;      // GPS latitude  (degrees)
    float lon;      // GPS longitude (degrees)
    float alt;      // GPS altitude  (meters)
    float ax;       // IMU accel X   (m/s²)
    float ay;       // IMU accel Y   (m/s²)
    float gz;       // IMU gyro Z    (deg/s)
    float heading;  // EKF heading   (degrees)
} pkt;

// ═══════════════════════════════════════════════════════════════════════════
//  CAR STATE MACHINE
// ═══════════════════════════════════════════════════════════════════════════
enum CarMode {
    MODE_LINE_FOLLOW,
    MODE_GPS_NAV,
    MODE_AVOID,
    MODE_ARRIVED
};
CarMode carMode = MODE_LINE_FOLLOW;

// ═══════════════════════════════════════════════════════════════════════════
//  EKF — 5-STATE: [x(m), y(m), heading(rad), vx(m/s), vy(m/s)]
//  Origin is set on first valid GPS fix.
//  Runs at ~50 Hz from IMU; GPS corrects at 1 Hz.
// ═══════════════════════════════════════════════════════════════════════════

// EKF state
float ekf_x[5]      = {0, 0, 0, 0, 0};  // [x, y, hdg, vx, vy]
float ekf_P[5][5];                        // covariance (5x5)
bool  ekf_originSet = false;
double originLat    = 0;
double originLon    = 0;

// Calibrated IMU bias (measured during T8 static test — update these values!)
float ax_bias = 0.0f;   // m/s² — replace after running IMU bias test
float ay_bias = 0.0f;
float gz_bias = 0.0f;   // deg/s

// Timing
unsigned long lastEKF   = 0;
unsigned long lastGPSup = 0;
unsigned long lastTx    = 0;

// Raw IMU readings (filled in readIMU)
float raw_ax = 0, raw_ay = 0, raw_gz = 0;

// ───────────────────────────────────────────────────────────────────────────
//  EKF helper: multiply 5x5 matrix A by 5x1 vector v → result r
// ───────────────────────────────────────────────────────────────────────────
void mat5x5_vec5(float A[5][5], float v[5], float r[5]) {
    for (int i = 0; i < 5; i++) {
        r[i] = 0;
        for (int j = 0; j < 5; j++) r[i] += A[i][j] * v[j];
    }
}

// ───────────────────────────────────────────────────────────────────────────
//  EKF helper: A = B * C  (5x5 * 5x5)
// ───────────────────────────────────────────────────────────────────────────
void mat5_mul(float A[5][5], float B[5][5], float C[5][5]) {
    float tmp[5][5] = {};
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            for (int k = 0; k < 5; k++)
                tmp[i][j] += B[i][k] * C[k][j];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            A[i][j] = tmp[i][j];
}

// ───────────────────────────────────────────────────────────────────────────
//  EKF helper: transpose 5x5
// ───────────────────────────────────────────────────────────────────────────
void mat5_transpose(float At[5][5], float A[5][5]) {
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            At[i][j] = A[j][i];
}

// ───────────────────────────────────────────────────────────────────────────
//  EKF PREDICT — called every ~20 ms (50 Hz) from IMU
// ───────────────────────────────────────────────────────────────────────────
void ekfPredict(float ax_body, float ay_body, float gz_rads, float dt) {
    float hdg = ekf_x[2];
    float ch  = cosf(hdg);
    float sh  = sinf(hdg);

    // World-frame accelerations
    float ax_w = ch * ax_body - sh * ay_body;
    float ay_w = sh * ax_body + ch * ay_body;

    // State transition (nonlinear)
    float x_new[5];
    x_new[0] = ekf_x[0] + ekf_x[3] * dt;
    x_new[1] = ekf_x[1] + ekf_x[4] * dt;
    x_new[2] = ekf_x[2] + gz_rads * dt;
    x_new[3] = ekf_x[3] + ax_w * dt;
    x_new[4] = ekf_x[4] + ay_w * dt;

    // Wrap heading to [-pi, pi]
    while (x_new[2] >  M_PI) x_new[2] -= 2 * M_PI;
    while (x_new[2] < -M_PI) x_new[2] += 2 * M_PI;

    for (int i = 0; i < 5; i++) ekf_x[i] = x_new[i];

    // Jacobian F (linearised around current state)
    float F[5][5] = {
        {1, 0, (-sh*ekf_x[3] - ch*ekf_x[4])*dt, dt,  0},
        {0, 1, ( ch*ekf_x[3] - sh*ekf_x[4])*dt,  0, dt},
        {0, 0,  1,                                  0,  0},
        {0, 0,  0,                                  1,  0},
        {0, 0,  0,                                  0,  1}
    };

    // Process noise Q (tune to your environment)
    float Q[5][5] = {};
    Q[0][0] = 0.01f;    // x uncertainty
    Q[1][1] = 0.01f;    // y uncertainty
    Q[2][2] = 0.001f;   // heading uncertainty
    Q[3][3] = 0.1f;     // vx uncertainty
    Q[4][4] = 0.1f;     // vy uncertainty

    // P = F * P * F' + Q
    float Ft[5][5], FP[5][5], FPFt[5][5];
    mat5_transpose(Ft, F);
    mat5_mul(FP, F, ekf_P);
    mat5_mul(FPFt, FP, Ft);
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            ekf_P[i][j] = FPFt[i][j] + Q[i][j];
}

// ───────────────────────────────────────────────────────────────────────────
//  EKF UPDATE — called when GPS gives a new fix (~1 Hz)
//  Observation: [x_local, y_local]  (2-state GPS measurement)
// ───────────────────────────────────────────────────────────────────────────
void ekfUpdateGPS(float gps_x, float gps_y) {
    // H = [1 0 0 0 0;
    //      0 1 0 0 0]
    // Innovation
    float innov[2] = {
        gps_x - ekf_x[0],
        gps_y - ekf_x[1]
    };

    // GPS measurement noise
    float R_gps = 4.0f;   // ~2 m std → variance = 4 m²

    // S = H*P*H' + R  (2x2 for our 2-state observation)
    float S[2][2] = {
        {ekf_P[0][0] + R_gps, ekf_P[0][1]},
        {ekf_P[1][0],         ekf_P[1][1] + R_gps}
    };

    // Invert S (2x2 explicit inverse)
    float det = S[0][0]*S[1][1] - S[0][1]*S[1][0];
    if (fabsf(det) < 1e-9f) return;  // Singular — skip update
    float Si[2][2] = {
        { S[1][1]/det, -S[0][1]/det},
        {-S[1][0]/det,  S[0][0]/det}
    };

    // K = P * H' * Si  (5x2 Kalman gain)
    // H' columns are [1,0,0,0,0]' and [0,1,0,0,0]' so P*H' = P cols 0 and 1
    float K[5][2];
    for (int i = 0; i < 5; i++) {
        K[i][0] = ekf_P[i][0]*Si[0][0] + ekf_P[i][1]*Si[1][0];
        K[i][1] = ekf_P[i][0]*Si[0][1] + ekf_P[i][1]*Si[1][1];
    }

    // x = x + K * innov
    for (int i = 0; i < 5; i++)
        ekf_x[i] += K[i][0]*innov[0] + K[i][1]*innov[1];

    // Wrap heading
    while (ekf_x[2] >  M_PI) ekf_x[2] -= 2*M_PI;
    while (ekf_x[2] < -M_PI) ekf_x[2] += 2*M_PI;

    // P = (I - K*H)*P  (Joseph form for numerical stability)
    // KH is 5x5: KH[i][j] = K[i][0]*H[0][j] + K[i][1]*H[1][j]
    // H[0][0]=1 all others 0; H[1][1]=1 all others 0
    float IKH[5][5];
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++) {
            float KH_ij = (j==0) ? K[i][0] : (j==1) ? K[i][1] : 0.0f;
            IKH[i][j] = (i==j ? 1.0f : 0.0f) - KH_ij;
        }
    float newP[5][5];
    mat5_mul(newP, IKH, ekf_P);
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            ekf_P[i][j] = newP[i][j];
}

// ───────────────────────────────────────────────────────────────────────────
//  Convert GPS degrees → local ENU metres from origin
// ───────────────────────────────────────────────────────────────────────────
void gpsToLocal(double lat, double lon, float &lx, float &ly) {
    double dLat = (lat - originLat) * DEG_TO_RAD;
    double dLon = (lon - originLon) * DEG_TO_RAD;
    double R    = 6371000.0;
    ly = (float)(dLat * R);
    lx = (float)(dLon * R * cos(originLat * DEG_TO_RAD));
}

// ───────────────────────────────────────────────────────────────────────────
//  Bearing from current EKF position → target GPS waypoint (radians)
// ───────────────────────────────────────────────────────────────────────────
float bearingToTarget() {
    float tx, ty;
    gpsToLocal(TARGET_LAT, TARGET_LON, tx, ty);
    return atan2f(tx - ekf_x[0], ty - ekf_x[1]);  // NED bearing
}

// ───────────────────────────────────────────────────────────────────────────
//  Distance to target (metres) using Haversine
// ───────────────────────────────────────────────────────────────────────────
float distToTarget() {
    if (!gps.location.isValid()) return 9999.0f;
    double lat1 = gps.location.lat() * DEG_TO_RAD;
    double lon1 = gps.location.lng() * DEG_TO_RAD;
    double lat2 = TARGET_LAT * DEG_TO_RAD;
    double lon2 = TARGET_LON * DEG_TO_RAD;
    double dLat = lat2 - lat1;
    double dLon = lon2 - lon1;
    double a    = sin(dLat/2)*sin(dLat/2) +
                  cos(lat1)*cos(lat2)*sin(dLon/2)*sin(dLon/2);
    return (float)(6371000.0 * 2.0 * atan2(sqrt(a), sqrt(1-a)));
}

// ═══════════════════════════════════════════════════════════════════════════
//  MOTOR CONTROL
// ═══════════════════════════════════════════════════════════════════════════
void motorStop() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

void motorForward(int leftPWM, int rightPWM) {
    leftPWM  = constrain(leftPWM,  0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);   // Left forward
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);   // Right forward
    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);
}

void motorReverse(int leftPWM, int rightPWM) {
    leftPWM  = constrain(leftPWM,  0, 255);
    rightPWM = constrain(rightPWM, 0, 255);
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
    digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
    analogWrite(ENA, leftPWM);
    analogWrite(ENB, rightPWM);
}

void motorTurnRight(int spd) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
    digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH);
    analogWrite(ENA, spd);
    analogWrite(ENB, spd);
}

void motorTurnLeft(int spd) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
    digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
    analogWrite(ENA, spd);
    analogWrite(ENB, spd);
}

// ═══════════════════════════════════════════════════════════════════════════
//  SENSORS
// ═══════════════════════════════════════════════════════════════════════════
float getDistanceCM() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long dur = pulseIn(ECHO_PIN, HIGH, 25000);  // 25 ms timeout
    if (dur == 0) return 400.0f;                // No echo = clear
    return dur * 0.0343f / 2.0f;
}

void readIMU() {
    int16_t ax16, ay16, az16, gx16, gy16, gz16;
    imu.getMotion6(&ax16, &ay16, &az16, &gx16, &gy16, &gz16);
    // Convert to SI units and subtract bias
    raw_ax = (ax16 / 16384.0f) * 9.81f - ax_bias;
    raw_ay = (ay16 / 16384.0f) * 9.81f - ay_bias;
    raw_gz = (gz16 / 131.0f) - gz_bias;
}

// ═══════════════════════════════════════════════════════════════════════════
//  CAR MODE HANDLERS
// ═══════════════════════════════════════════════════════════════════════════

// ── MODE 2: OBSTACLE AVOIDANCE ──────────────────────────────────────────
void handleAvoid() {
    motorStop();       delay(150);
    motorReverse(REVERSE_SPEED, REVERSE_SPEED);  delay(350);
    motorStop();       delay(50);
    motorTurnRight(BASE_SPEED);                  delay(450);
    motorStop();       delay(50);
    // Return to previous mode
    carMode = MODE_LINE_FOLLOW;
}

// ── MODE 1: LINE FOLLOWING ───────────────────────────────────────────────
void handleLineFollow() {
    bool L = !digitalRead(IR_LEFT);   // Invert: LOW = on line
    bool M = !digitalRead(IR_MID);
    bool R = !digitalRead(IR_RIGHT);

    if (M && !L && !R) {
        // Straight
        motorForward(BASE_SPEED, BASE_SPEED);
    } else if (L && !R) {
        // Drifted left — slow left motor
        motorForward(TURN_SPEED, BASE_SPEED);
    } else if (R && !L) {
        // Drifted right — slow right motor
        motorForward(BASE_SPEED, TURN_SPEED);
    } else if (L && M && R) {
        // Intersection — keep straight
        motorForward(BASE_SPEED, BASE_SPEED);
    } else {
        // Line lost — switch to GPS nav if fix available
        motorStop();
        if (gps.location.isValid() && ekf_originSet) {
            carMode = MODE_GPS_NAV;
        }
    }
}

// ── MODE 3: GPS NAVIGATION ───────────────────────────────────────────────
void handleGPSNav() {
    // Check if arrived
    if (distToTarget() < ARRIVE_DIST_M) {
        carMode = MODE_ARRIVED;
        return;
    }

    // If any IR sensor sees a line, return to line-follow mode
    if (!digitalRead(IR_LEFT) || !digitalRead(IR_MID) || !digitalRead(IR_RIGHT)) {
        carMode = MODE_LINE_FOLLOW;
        return;
    }

    // Proportional heading controller
    float target_bearing = bearingToTarget();
    float hdg_error      = target_bearing - ekf_x[2];

    // Normalise error to [-pi, pi]
    while (hdg_error >  M_PI) hdg_error -= 2*M_PI;
    while (hdg_error < -M_PI) hdg_error += 2*M_PI;

    // P-gain — scales how aggressively to steer
    float Kp      = 60.0f;
    float steer   = constrain(Kp * hdg_error, -80, 80);
    int   leftPWM = (int)(GPS_SPEED + steer);
    int   rightPWM= (int)(GPS_SPEED - steer);
    motorForward(constrain(leftPWM, 60, 255),
                 constrain(rightPWM, 60, 255));
}

// ── MODE 4: ARRIVED ──────────────────────────────────────────────────────
void handleArrived() {
    motorStop();
    // Stay stopped — latch until power cycle or manual reset
}

// ═══════════════════════════════════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════════════════════════════════
void setup() {
    Serial.begin(115200);
    gpsSerial.begin(9600);

    // Pin modes
    pinMode(TRIG_PIN,  OUTPUT);
    pinMode(ECHO_PIN,  INPUT);
    pinMode(IR_LEFT,   INPUT);
    pinMode(IR_MID,    INPUT);
    pinMode(IR_RIGHT,  INPUT);
    pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
    pinMode(ENA, OUTPUT); pinMode(ENB, OUTPUT);

    motorStop();

    // ── IMU Init ──
    Wire.begin();
    imu.initialize();
    if (!imu.testConnection()) {
        Serial.println("MPU-6050 FAIL — check wiring");
    }
    imu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
    imu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    imu.setDLPFMode(MPU6050_DLPF_BW_42);   // 42 Hz low-pass filter

    // ── RF Init ──
    if (!radio.begin()) {
        Serial.println("nRF24L01 FAIL — check wiring and 3.3V supply");
    }
    radio.setPALevel(RF24_PA_HIGH);
    radio.setDataRate(RF24_250KBPS);
    radio.setChannel(108);               // Channel 108 avoids common WiFi
    radio.openWritingPipe(RF_ADDR);
    radio.stopListening();               // Transmit mode

    // ── EKF Covariance Init ──
    for (int i = 0; i < 5; i++)
        for (int j = 0; j < 5; j++)
            ekf_P[i][j] = (i == j) ? 50.0f : 0.0f;

    Serial.println("Smart Car Ready.");
    delay(500);
}

// ═══════════════════════════════════════════════════════════════════════════
//  MAIN LOOP
// ═══════════════════════════════════════════════════════════════════════════
void loop() {
    unsigned long now = millis();

    // ── Feed GPS parser ──────────────────────────────────────────────────
    while (gpsSerial.available()) gps.encode(gpsSerial.read());

    // ── Read IMU ─────────────────────────────────────────────────────────
    readIMU();

    // ── EKF Predict at 50 Hz ─────────────────────────────────────────────
    if (now - lastEKF >= 20) {
        float dt = (now - lastEKF) / 1000.0f;
        lastEKF = now;

        // Set origin on first GPS fix
        if (gps.location.isValid() && !ekf_originSet) {
            originLat    = gps.location.lat();
            originLon    = gps.location.lng();
            ekf_originSet = true;
            Serial.print("EKF origin set: ");
            Serial.print(originLat, 6); Serial.print(", ");
            Serial.println(originLon, 6);
        }

        ekfPredict(raw_ax, raw_ay, raw_gz * DEG_TO_RAD, dt);
    }

    // ── EKF GPS Update at 1 Hz ───────────────────────────────────────────
    if (gps.location.isValid() && ekf_originSet &&
        gps.location.isUpdated() && now - lastGPSup >= 900) {
        lastGPSup = now;
        float lx, ly;
        gpsToLocal(gps.location.lat(), gps.location.lng(), lx, ly);
        ekfUpdateGPS(lx, ly);
    }

    // ── Obstacle check (always highest priority) ──────────────────────────
    float dist_cm = getDistanceCM();
    if (dist_cm < OBSTACLE_CM && carMode != MODE_ARRIVED) {
        carMode = MODE_AVOID;
    }

    // ── State machine ────────────────────────────────────────────────────
    switch (carMode) {
        case MODE_AVOID:       handleAvoid();       break;
        case MODE_LINE_FOLLOW: handleLineFollow();  break;
        case MODE_GPS_NAV:     handleGPSNav();      break;
        case MODE_ARRIVED:     handleArrived();     break;
    }

    // ── RF Transmit at 10 Hz ─────────────────────────────────────────────
    if (now - lastTx >= TX_INTERVAL_MS) {
        lastTx = now;

        pkt.ax      = raw_ax;
        pkt.ay      = raw_ay;
        pkt.gz      = raw_gz;
        pkt.lat     = gps.location.isValid() ? (float)gps.location.lat() : 0.0f;
        pkt.lon     = gps.location.isValid() ? (float)gps.location.lng() : 0.0f;
        pkt.alt     = gps.altitude.isValid()  ? (float)gps.altitude.meters() : 0.0f;
        pkt.heading = ekf_x[2] * RAD_TO_DEG;   // Send heading in degrees

        bool ok = radio.write(&pkt, sizeof(pkt));

        // Debug to Serial (optional — comment out in final build)
        Serial.print(ok ? "TX OK  " : "TX FAIL  ");
        Serial.print("Mode:"); Serial.print(carMode);
        Serial.print("  Dist:"); Serial.print(dist_cm, 1);
        Serial.print("  Hdg:"); Serial.print(pkt.heading, 1);
        Serial.print("  Lat:"); Serial.print(pkt.lat, 5);
        Serial.print("  Lon:"); Serial.println(pkt.lon, 5);
    }
}

// ═══════════════════════════════════════════════════════════════════════════
//  NOTES
// ─────────────────────────────────────────────────────────────────────────
//  CALIBRATION STEPS (do these before first real run):
//
//  1. IMU BIAS (Test T8):
//     Place car still on flat surface for 60 seconds.
//     Average the ax, ay, gz readings from Serial Monitor.
//     Enter those averages as ax_bias, ay_bias, gz_bias above.
//
//  2. GPS WAYPOINT:
//     Change TARGET_LAT / TARGET_LON at the top of this file
//     to your desired destination coordinates.
//
//  3. MOTOR DIRECTION:
//     Upload, run motorForward(160,160) briefly.
//     If car goes backward, swap IN1↔IN2 (or IN3↔IN4) wires.
//
//  4. IR POLARITY:
//     Hold sensor 5–8 mm over black line.
//     Confirm digitalRead() returns LOW.
//     If reversed, change !digitalRead() to digitalRead() in handleLineFollow().
//
//  5. RF CHANNEL:
//     If packet loss > 10%, try channel 76 (uncommon WiFi overlap).
//     Both car and ground receiver must use the same channel.
// ═══════════════════════════════════════════════════════════════════════════
