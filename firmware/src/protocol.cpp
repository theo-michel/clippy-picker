#include "protocol.h"
#include "motors.h"
#include "gripper.h"
#include "config.h"
#include <string.h>

Protocol protocol;

// ── Init / read ─────────────────────────────────────────────────────────────

void Protocol::init() {
    Serial.begin(SERIAL_BAUD);
    _pos = 0;
}

bool Protocol::readCommand() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n' || c == '\r') {
            if (_pos > 0) {
                _buf[_pos] = '\0';
                _pos = 0;
                return true;
            }
        } else if (_pos < CMD_BUF_SIZE - 1) {
            _buf[_pos++] = c;
        }
    }
    return false;
}

// ── Responses ───────────────────────────────────────────────────────────────

void Protocol::ok()                    { Serial.println("OK"); }
void Protocol::done()                  { Serial.println("DONE"); }
void Protocol::error(const char* msg)  { Serial.print("ERR:"); Serial.println(msg); }

// ── Dispatch ────────────────────────────────────────────────────────────────

void Protocol::dispatch() {
    char work[CMD_BUF_SIZE];
    strncpy(work, _buf, CMD_BUF_SIZE);

    char* tok = strtok(work, " ");
    if (!tok) return;

    if      (strcmp(tok, "M")      == 0) _cmdMove();
    else if (strcmp(tok, "G")      == 0) _cmdGantry();
    else if (strcmp(tok, "MG")     == 0) _cmdMoveGantry();
    else if (strcmp(tok, "GRIP")   == 0) _cmdGrip();
    else if (strcmp(tok, "SPD")    == 0) _cmdSpeed();
    else if (strcmp(tok, "ACC")    == 0) _cmdAccel();
    else if (strcmp(tok, "GSPD")   == 0) _cmdGantrySpeed();
    else if (strcmp(tok, "GACC")   == 0) _cmdGantryAccel();
    else if (strcmp(tok, "HOME")   == 0) _cmdHome();
    else if (strcmp(tok, "GANTRY_HOME") == 0) _cmdGantryHome();
    else if (strcmp(tok, "DELTA_ENABLE")  == 0) _cmdDeltaEnable();
    else if (strcmp(tok, "DELTA_DISABLE") == 0) _cmdDeltaDisable();
    else if (strcmp(tok, "STOP")   == 0) _cmdStop();
    else if (strcmp(tok, "ESTOP")  == 0) _cmdEstop();
    else if (strcmp(tok, "POS")    == 0) _cmdPos();
    else if (strcmp(tok, "STATUS") == 0) _cmdStatus();
    else if (strcmp(tok, "TELEM")  == 0) _cmdTelem();
    else if (strcmp(tok, "ZERO")   == 0) _cmdZero();
    else if (strcmp(tok, "PING")   == 0) _cmdPing();
    else                                 error("UNKNOWN_CMD");
}

// ── Command implementations ─────────────────────────────────────────────────

void Protocol::_cmdMove() {
    char *s1 = strtok(NULL, " "), *s2 = strtok(NULL, " "), *s3 = strtok(NULL, " ");
    if (!s1 || !s2 || !s3) { error("M needs 3 angles"); return; }

    if (!motors.setDeltaTarget(atof(s1), atof(s2), atof(s3))) {
        error("ANGLE_LIMIT");
        return;
    }
    ok();
}

void Protocol::_cmdGantry() {
    char* sx = strtok(NULL, " ");
    if (!sx) { error("G needs 1 position (mm)"); return; }

    if (!motors.setGantryTarget(atof(sx))) {
        error("GANTRY_LIMIT");
        return;
    }
    ok();
}

void Protocol::_cmdMoveGantry() {
    char *s1 = strtok(NULL, " "), *s2 = strtok(NULL, " ");
    char *s3 = strtok(NULL, " "), *sx = strtok(NULL, " ");
    if (!s1 || !s2 || !s3 || !sx) { error("MG needs 3 angles + 1 position"); return; }

    if (!motors.setDeltaTarget(atof(s1), atof(s2), atof(s3))) { error("ANGLE_LIMIT"); return; }
    if (!motors.setGantryTarget(atof(sx)))                     { error("GANTRY_LIMIT"); return; }
    ok();
}

void Protocol::_cmdGrip() {
    char* arg = strtok(NULL, " ");
    if (!arg) { error("GRIP needs OPEN/CLOSE/<pos>"); return; }

    if      (strcmp(arg, "OPEN")  == 0) gripper.open();
    else if (strcmp(arg, "CLOSE") == 0) gripper.close();
    else                                gripper.setPosition(atoi(arg));
    ok();
}

void Protocol::_cmdSpeed() {
    char* v = strtok(NULL, " ");
    if (!v) { error("SPD needs value"); return; }
    motors.setDeltaSpeed(atof(v));
    ok();
}

void Protocol::_cmdAccel() {
    char* v = strtok(NULL, " ");
    if (!v) { error("ACC needs value"); return; }
    motors.setDeltaAccel(atof(v));
    ok();
}

void Protocol::_cmdGantrySpeed() {
    char* v = strtok(NULL, " ");
    if (!v) { error("GSPD needs value"); return; }
    motors.setGantrySpeed(atof(v));
    ok();
}

void Protocol::_cmdGantryAccel() {
    char* v = strtok(NULL, " ");
    if (!v) { error("GACC needs value"); return; }
    motors.setGantryAccel(atof(v));
    ok();
}

void Protocol::_cmdHome()         { motors.home();             ok(); }
void Protocol::_cmdGantryHome()   { motors.startGantryHoming(); ok(); }
void Protocol::_cmdDeltaEnable()  { motors.enableDelta();       ok(); }
void Protocol::_cmdDeltaDisable() { motors.disableDelta();      ok(); }
void Protocol::_cmdStop()       { motors.stop();             ok(); }
void Protocol::_cmdEstop() { motors.emergencyStop(); ok(); }
void Protocol::_cmdZero()  { motors.setZero();       ok(); }
void Protocol::_cmdPing()  { Serial.println("PONG"); }

void Protocol::_cmdPos() {
    char buf[80];
    snprintf(buf, sizeof(buf), "%.2f,%.2f,%.2f,%.2f",
             motors.getDeltaAngle(0), motors.getDeltaAngle(1),
             motors.getDeltaAngle(2), motors.getGantryMm());
    Serial.print("POS:");
    Serial.println(buf);
}

void Protocol::_cmdStatus() {
    Serial.print("STATUS:");
    Serial.println(motors.isMoving() ? "MOVING" : "IDLE");
}

void Protocol::_cmdTelem() {
    char buf[200];
    snprintf(buf, sizeof(buf),
        "d1=%.2f,d2=%.2f,d3=%.2f,gx=%.2f,"
        "moving=%d,"
        "dxl_pos=%ld,dxl_temp=%ld,dxl_load=%ld,dxl_ok=%d",
        motors.getDeltaAngle(0), motors.getDeltaAngle(1),
        motors.getDeltaAngle(2), motors.getGantryMm(),
        motors.isMoving() ? 1 : 0,
        (long)gripper.getPosition(),
        (long)gripper.getTemperature(),
        (long)gripper.getLoad(),
        gripper.isConnected() ? 1 : 0);
    Serial.print("TELEM:");
    Serial.println(buf);
}
