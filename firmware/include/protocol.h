#pragma once

#include <Arduino.h>

#define CMD_BUF_SIZE 128

/// Serial command protocol.
///
/// Reads newline-terminated text commands from the USB serial port,
/// tokenises them, validates arguments, and dispatches to the
/// motors / gripper subsystems.
///
/// Response conventions:
///   OK              — command accepted, motion started
///   DONE            — all motors have reached their targets
///   ERR:<message>   — something went wrong
///   POS:<csv>       — position report
///   STATUS:<state>  — IDLE / MOVING
///   TELEM:<kv>      — full telemetry (key=value pairs)
///   PONG            — heartbeat reply
class Protocol {
public:
    void init();

    /// Call every loop().  Returns true when a complete line is ready.
    bool readCommand();

    /// Parse and execute the buffered command.
    void dispatch();

    // Response helpers (static so other modules can use them too)
    static void ok();
    static void done();
    static void error(const char* msg);

private:
    char _buf[CMD_BUF_SIZE];
    int  _pos = 0;

    // Command handlers
    void _cmdMove();
    void _cmdGantry();
    void _cmdMoveGantry();
    void _cmdGrip();
    void _cmdSpeed();
    void _cmdAccel();
    void _cmdGantrySpeed();
    void _cmdGantryAccel();
    void _cmdHome();
    void _cmdStop();
    void _cmdEstop();
    void _cmdPos();
    void _cmdStatus();
    void _cmdTelem();
    void _cmdZero();
    void _cmdPing();
};

extern Protocol protocol;
