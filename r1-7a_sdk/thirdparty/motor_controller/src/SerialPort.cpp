#include "SerialPort.h"
#include <stdexcept>
#include <iostream>
#include <chrono>
#include <cstring>

#ifndef _WIN32
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#ifdef __linux__
#include <linux/serial.h>
#if defined(TCGETS2) && !defined(BOTHER)
struct termios2 {
    tcflag_t c_iflag;
    tcflag_t c_oflag;
    tcflag_t c_cflag;
    tcflag_t c_lflag;
    cc_t c_line;
    cc_t c_cc[19];
    speed_t c_ispeed;
    speed_t c_ospeed;
};
#define BOTHER 0x00001000
#endif
#endif
#endif

SerialPort::SerialPort()
#ifdef _WIN32
    : m_handle(INVALID_HANDLE_VALUE)
#else
    : m_fd(-1)
#endif
    , m_isOpen(false)
    , m_running(false)
    , m_timeoutSec(0.1) {
}

SerialPort::~SerialPort() {
    close();
}

bool SerialPort::open(const std::string& port, int baudrate, double timeout) {
    if (m_isOpen) {
        close();
    }

    m_timeoutSec = timeout;

#ifdef _WIN32
    std::string portName = "\\\\.\\" + port;

    m_handle = CreateFileA(
        portName.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0,
        NULL,
        OPEN_EXISTING,
        FILE_ATTRIBUTE_NORMAL,
        NULL
    );

    if (m_handle == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        std::cerr << "Failed to open port " << port << ", error code: " << error << std::endl;
        return false;
    }
#else
    m_fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
    if (m_fd < 0) {
        std::cerr << "Failed to open port " << port << ": " << strerror(errno) << std::endl;
        return false;
    }
#endif

    try {
        configurePort(baudrate, timeout);
        m_isOpen = true;
        std::cout << "Port " << port << " opened, baudrate: " << baudrate << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Port configuration failed: " << e.what() << std::endl;
#ifdef _WIN32
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
#else
        ::close(m_fd);
        m_fd = -1;
#endif
        return false;
    }
}

void SerialPort::close() {
    stopAsyncRead();
#ifdef _WIN32
    if (m_handle != INVALID_HANDLE_VALUE) {
        CloseHandle(m_handle);
        m_handle = INVALID_HANDLE_VALUE;
        m_isOpen = false;
    }
#else
    if (m_fd >= 0) {
        ::close(m_fd);
        m_fd = -1;
        m_isOpen = false;
    }
#endif
    {
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        m_buffer.clear();
    }
}

bool SerialPort::isOpen() const {
    return m_isOpen;
}

void SerialPort::configurePort(int baudrate, double timeout) {
#ifdef _WIN32
    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);

    if (!GetCommState(m_handle, &dcb)) {
        throw std::runtime_error("Failed to get port state");
    }

    dcb.BaudRate = baudrate;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;
    dcb.fBinary = TRUE;
    dcb.fDtrControl = DTR_CONTROL_DISABLE;
    dcb.fRtsControl = RTS_CONTROL_DISABLE;
    dcb.fOutxCtsFlow = FALSE;
    dcb.fOutxDsrFlow = FALSE;

    if (!SetCommState(m_handle, &dcb)) {
        throw std::runtime_error("Failed to set port state");
    }

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = MAXDWORD;
    timeouts.ReadTotalTimeoutMultiplier = 0;
    timeouts.ReadTotalTimeoutConstant = static_cast<DWORD>(timeout * 1000);
    timeouts.WriteTotalTimeoutMultiplier = 0;
    timeouts.WriteTotalTimeoutConstant = 1000;

    if (!SetCommTimeouts(m_handle, &timeouts)) {
        throw std::runtime_error("Failed to set timeouts");
    }

    SetupComm(m_handle, 4096, 4096);
    PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
    (void)timeout;

    struct termios tty {};
    if (tcgetattr(m_fd, &tty) != 0) {
        throw std::runtime_error(std::string("tcgetattr failed: ") + strerror(errno));
    }

    bool non_standard_baud = false;
    speed_t baud;
    switch (baudrate) {
    case 9600:    baud = B9600;    break;
    case 19200:   baud = B19200;   break;
    case 38400:   baud = B38400;   break;
    case 57600:   baud = B57600;   break;
    case 115200:  baud = B115200;  break;
    case 230400:  baud = B230400;  break;
    case 460800:  baud = B460800;  break;
    case 500000:  baud = B500000;  break;
    case 576000:  baud = B576000;  break;
    case 921600:  baud = B921600;  break;
    case 1000000: baud = B1000000; break;
    case 1152000: baud = B1152000; break;
    case 1500000: baud = B1500000; break;
    case 2000000: baud = B2000000; break;
    case 2500000: baud = B2500000; break;
    case 3000000: baud = B3000000; break;
    case 3500000: baud = B3500000; break;
    case 4000000: baud = B4000000; break;
    default:
        baud = B38400;
        non_standard_baud = true;
        break;
    }

    cfsetispeed(&tty, baud);
    cfsetospeed(&tty, baud);

    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS | HUPCL);
    tty.c_cflag |= (CLOCAL | CREAD);

    tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL |
                     IXON | IXOFF | IXANY | IMAXBEL);
    tty.c_iflag |= IGNPAR;
    tty.c_oflag &= ~(OPOST | ONLCR | OCRNL | ONOCR | ONLRET);
    tty.c_lflag &= ~(ISIG | ICANON | ECHO | ECHOE | ECHOK | ECHONL | IEXTEN);

    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(m_fd, TCSANOW, &tty) != 0) {
        throw std::runtime_error(std::string("tcsetattr failed: ") + strerror(errno));
    }

#ifdef __linux__
    struct serial_struct sc {};
    if (ioctl(m_fd, TIOCGSERIAL, &sc) == 0) {
        sc.flags |= ASYNC_LOW_LATENCY;
        if (non_standard_baud && sc.baud_base >= static_cast<int>(baudrate) && baudrate > 0) {
            sc.flags &= ~ASYNC_SPD_MASK;
            sc.flags |= ASYNC_SPD_CUST;
            sc.custom_divisor = sc.baud_base / baudrate;
        }
        ioctl(m_fd, TIOCSSERIAL, &sc);
    }

#if defined(TCGETS2)
    if (non_standard_baud) {
        struct termios2 t2 {};
        if (ioctl(m_fd, TCGETS2, &t2) == 0) {
            t2.c_cflag &= ~CBAUD;
            t2.c_cflag |= BOTHER;
            t2.c_ispeed = baudrate;
            t2.c_ospeed = baudrate;
            if (ioctl(m_fd, TCSETS2, &t2) != 0) {
                throw std::runtime_error(
                    std::string("TCSETS2 failed for baudrate ") +
                    std::to_string(baudrate) + ": " + strerror(errno));
            }
        }
    }
#endif
#else
    if (non_standard_baud) {
        throw std::runtime_error(
            "Non-standard baudrate requires Linux termios2 support");
    }
#endif

    // Clear O_NONBLOCK for more predictable write/read; async path uses select.
    int flags = fcntl(m_fd, F_GETFL, 0);
    if (flags >= 0) {
        fcntl(m_fd, F_SETFL, flags & ~O_NONBLOCK);
    }

    tcflush(m_fd, TCIOFLUSH);
#endif
}

void SerialPort::write(const std::vector<uint8_t>& data) {
    if (!m_isOpen) {
        throw std::runtime_error("Port not open");
    }

#ifdef _WIN32
    DWORD bytesWritten;
    if (!WriteFile(m_handle, data.data(), static_cast<DWORD>(data.size()), &bytesWritten, NULL)) {
        throw std::runtime_error("Port write failed");
    }

    if (bytesWritten != data.size()) {
        throw std::runtime_error("Port write incomplete");
    }
#else
    size_t total = 0;
    while (total < data.size()) {
        ssize_t n = ::write(m_fd, data.data() + total, data.size() - total);
        if (n < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("Port write failed: ") + strerror(errno));
        }
        if (n == 0) {
            throw std::runtime_error("Port write incomplete");
        }
        total += static_cast<size_t>(n);
    }
#endif
}

std::vector<uint8_t> SerialPort::read(size_t size) {
    if (!m_isOpen) {
        throw std::runtime_error("Port not open");
    }

    std::vector<uint8_t> buffer(size);

#ifdef _WIN32
    DWORD totalRead = 0;

    while (totalRead < size) {
        DWORD bytesRead = 0;
        DWORD remaining = static_cast<DWORD>(size - totalRead);

        if (!ReadFile(m_handle, buffer.data() + totalRead, remaining, &bytesRead, NULL)) {
            DWORD error = GetLastError();
            if (error == ERROR_TIMEOUT || error == ERROR_IO_PENDING) {
                break;
            }
            throw std::runtime_error("Port read failed");
        }

        if (bytesRead == 0) {
            break;
        }

        totalRead += bytesRead;
    }

    buffer.resize(totalRead);
#else
    size_t totalRead = 0;
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(m_timeoutSec));

    while (totalRead < size) {
        auto now = std::chrono::steady_clock::now();
        if (now >= deadline) {
            break;
        }

        auto remain = std::chrono::duration_cast<std::chrono::microseconds>(deadline - now);
        timeval tv {};
        tv.tv_sec = static_cast<time_t>(remain.count() / 1000000);
        tv.tv_usec = static_cast<suseconds_t>(remain.count() % 1000000);

        fd_set rset;
        FD_ZERO(&rset);
        FD_SET(m_fd, &rset);

        int sel = select(m_fd + 1, &rset, nullptr, nullptr, &tv);
        if (sel < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw std::runtime_error(std::string("Port read select failed: ") + strerror(errno));
        }
        if (sel == 0) {
            break;
        }

        ssize_t n = ::read(m_fd, buffer.data() + totalRead, size - totalRead);
        if (n < 0) {
            if (errno == EINTR || errno == EAGAIN) {
                continue;
            }
            throw std::runtime_error(std::string("Port read failed: ") + strerror(errno));
        }
        if (n == 0) {
            break;
        }
        totalRead += static_cast<size_t>(n);
    }

    buffer.resize(totalRead);
#endif
    return buffer;
}

void SerialPort::flush() {
    if (m_isOpen) {
#ifdef _WIN32
        PurgeComm(m_handle, PURGE_RXCLEAR | PURGE_TXCLEAR);
#else
        tcflush(m_fd, TCIOFLUSH);
#endif
    }
    {
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        m_buffer.clear();
    }
}

void SerialPort::startAsyncRead(DataReceivedCallback callback) {
    if (!m_isOpen) {
        throw std::runtime_error("Port not open");
    }

    if (m_running) {
        stopAsyncRead();
    }

    m_callback = callback;
    m_running = true;
    {
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        m_buffer.clear();
    }
    m_readThread = std::thread(&SerialPort::readThreadFunc, this);
}

void SerialPort::stopAsyncRead() {
    m_running = false;
    if (m_readThread.joinable()) {
        m_readThread.join();
    }
    m_callback = nullptr;
    {
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        m_buffer.clear();
    }
}

void SerialPort::readThreadFunc() {
    while (m_running) {
        if (readAvailableData()) {
            processBuffer();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

bool SerialPort::readAvailableData() {
    if (!m_isOpen) {
        return false;
    }

#ifdef _WIN32
    COMSTAT comStat;
    DWORD errors;
    if (!ClearCommError(m_handle, &errors, &comStat)) {
        return false;
    }

    if (comStat.cbInQue == 0) {
        return false;
    }

    DWORD bytesToRead = comStat.cbInQue;
    std::vector<uint8_t> tempBuffer(bytesToRead);

    DWORD bytesRead = 0;
    if (!ReadFile(m_handle, tempBuffer.data(), bytesToRead, &bytesRead, NULL)) {
        return false;
    }

    if (bytesRead > 0) {
        tempBuffer.resize(bytesRead);
        std::lock_guard<std::mutex> lock(m_bufferMutex);
        m_buffer.insert(m_buffer.end(), tempBuffer.begin(), tempBuffer.end());
        return true;
    }

    return false;
#else
    fd_set rset;
    FD_ZERO(&rset);
    FD_SET(m_fd, &rset);
    timeval tv {};
    tv.tv_sec = 0;
    tv.tv_usec = 0;

    int sel = select(m_fd + 1, &rset, nullptr, nullptr, &tv);
    if (sel <= 0) {
        return false;
    }

    uint8_t temp[1024];
    ssize_t n = ::read(m_fd, temp, sizeof(temp));
    if (n <= 0) {
        return false;
    }

    std::lock_guard<std::mutex> lock(m_bufferMutex);
    m_buffer.insert(m_buffer.end(), temp, temp + n);
    return true;
#endif
}

void SerialPort::processBuffer() {
    std::lock_guard<std::mutex> lock(m_bufferMutex);

    const size_t PACKET_SIZE = 26;
    const uint8_t HEADER0 = 0xFC;
    const uint8_t HEADER1 = 0xEE;

    while (m_buffer.size() >= PACKET_SIZE) {
        size_t headerPos = 0;
        bool found = false;

        for (size_t i = 0; i <= m_buffer.size() - 2; ++i) {
            if (m_buffer[i] == HEADER0 && m_buffer[i + 1] == HEADER1) {
                headerPos = i;
                found = true;
                break;
            }
        }

        if (!found) {
            if (m_buffer.size() > 2) {
                m_buffer.erase(m_buffer.begin(), m_buffer.end() - 2);
            }
            else {
                m_buffer.clear();
            }
            break;
        }

        if (m_buffer.size() - headerPos < PACKET_SIZE) {
            break;
        }

        std::vector<uint8_t> packet(m_buffer.begin() + headerPos,
            m_buffer.begin() + headerPos + PACKET_SIZE);

        m_buffer.erase(m_buffer.begin(), m_buffer.begin() + headerPos + PACKET_SIZE);

        if (m_callback) {
            m_callback(packet);
        }
    }

    if (m_buffer.size() > 4096) {
        m_buffer.erase(m_buffer.begin(), m_buffer.end() - 2);
    }
}
