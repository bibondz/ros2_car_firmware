/*
 * Copyright 2026 Phuthiphong Wongchantib
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 *
 * Original author / contributor:
 * Phuthiphong Wongchantib
 */
/**
 * Fake HardwareSerial for the native GPS parser tests: a byte queue you can
 * push canned NMEA sentences into.
 */
#ifndef FAKE_SERIAL_H
#define FAKE_SERIAL_H

#include <cstdint>
#include <cstring>
#include <string>

#define SERIAL_8N1 0x800001c

class FakeSerial {
public:
    void begin(unsigned long baud, int = SERIAL_8N1, int = -1, int = -1) { baud_ = baud; }
    void end() {}
    void setRxBufferSize(int) {}

    int available() { return (int)(buffer_.size() - read_pos_); }
    int read() {
        if (read_pos_ >= buffer_.size()) return -1;
        return (unsigned char)buffer_[read_pos_++];
    }
    size_t read(uint8_t* buffer, size_t size) {
        size_t n = 0;
        while (n < size && read_pos_ < buffer_.size()) {
            buffer[n++] = (uint8_t)buffer_[read_pos_++];
        }
        return n;
    }

    /** Bulk read, the way Arduino's HardwareSerial provides it.
     *
     * The GPS parser reads in batches rather than a byte at a time, because on
     * the ESP32 every single-byte read takes a lock in the UART HAL - measured
     * at about 26 us per byte, which cost the control loop 3.1 ms per cycle
     * with a GPS fix. The mock has to offer the same call, or the harness
     * silently stops testing the code that actually ships.
     */
    size_t readBytes(unsigned char *out, size_t length) {
        size_t n = 0;
        while (n < length && read_pos_ < buffer_.size()) {
            out[n++] = (unsigned char)buffer_[read_pos_++];
        }
        return n;
    }

    /** Test helper: queue raw characters as if the GPS had sent them. */
    void push(const char *text) { buffer_ += text; }
    void clear() { buffer_.clear(); read_pos_ = 0; }
    unsigned long baud() const { return baud_; }

    // ---- output side, so a console's REPLIES can be asserted on ----------
    // Without this a line protocol can only be tested for "did not crash",
    // which is the half of it that never breaks.
    size_t print(const char *text) { out_ += text; return strlen(text); }
    size_t print(char c) { out_ += c; return 1; }
    size_t println(const char *text) { out_ += text; out_ += '\n'; return strlen(text) + 1; }
    size_t write(uint8_t c) { out_ += (char)c; return 1; }

    /** Everything written since the last clearOut(). */
    const std::string &out() const { return out_; }
    void clearOut() { out_.clear(); }
    /** Convenience for assertions: did the reply contain this? */
    bool said(const char *needle) const { return out_.find(needle) != std::string::npos; }

private:
    std::string buffer_;
    std::string out_;
    size_t read_pos_ = 0;
    unsigned long baud_ = 0;
};

#endif // FAKE_SERIAL_H
