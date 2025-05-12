#include "rclcpp/rclcpp.hpp"
#include "oled_display_node/msg/display_output.hpp"
#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <sstream>
#include <iomanip> // For std::setprecision

// Linux I2C and System includes
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <string.h>
#include <cerrno> // For errno
#include <cstdio> // For popen, fgetc, pclose
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>

// Local includes
#include "oled_display_node/oled_display.h"
#include "oled_display_node/font8x8_basic.h" // Make sure this contains the actual font data!

using namespace std::chrono_literals;
using std::placeholders::_1;

// --- Forward Declarations for Driver Functions ---
static int i2c_write(rclcpp::Logger logger, const char *i2cDevFile, uint8_t i2c7bitAddr, uint8_t *pBuffer, int numBytes);
static int i2c_read(rclcpp::Logger logger, const char *i2cDevFile, uint8_t i2c7bitAddr, uint8_t *pBuffer, int numBytes, uint8_t chipRegAddr, bool chipRegAddrFlag);
int dispOled_detectDisplayType(rclcpp::Logger logger, std::string devName, uint8_t i2c7bitAddr, int *dispType);
int dispOled_initCtx(rclcpp::Logger logger, std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr);
int dispOled_init(rclcpp::Logger logger, std::string devName, dispCtx_t *dispCtx, int displayType, uint8_t i2cAddr);
int dispOled_setCursor(rclcpp::Logger logger, dispCtx_t *dispCtx, int column, int line);
int dispOled_clearDisplay(rclcpp::Logger logger, dispCtx_t *dispCtx);
int dispOled_writeText(rclcpp::Logger logger, dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, const char *textStr);
std::string getPopen(std::string input);
void getIpAddressses(rclcpp::Logger logger, std::string &ethAddress, std::string &wlanAddress, int logFindings);

// --- OLED Initialization Data ---
#define SSD1306_INIT_BYTE_COUNT         6
static uint8_t ssd1306_init_bytes[SSD1306_INIT_BYTE_COUNT] = {
                OLED_CONTROL_BYTE_CMD_STREAM,
                                OLED_CMD_SET_CHARGE_PUMP,       0x14,
                                OLED_CMD_SET_SEGMENT_REMAP,
                                OLED_CMD_SET_COM_SCAN_MODE,
                                OLED_CMD_DISPLAY_ON
};

#define SH1106_INIT_BYTE_COUNT  17
static uint8_t sh1106_init_bytes[SH1106_INIT_BYTE_COUNT] = {
                OLED_CONTROL_BYTE_CMD_STREAM,
                                0x30,                           // Charge pump default
                                0x40,                           // RAM display line of 0
                                OLED_CMD_DISPLAY_OFF,
                                OLED_CMD_SET_SEGMENT_REMAP,
                                OLED_CMD_SET_COM_SCAN_MODE,
                                OLED_CMD_SET_DISPLAY_OFFSET, 0, // Sets mapping of display start line
                                OLED_CMD_DC_DC_CTRL_MODE, 0x8B, // Must have DISPLAY_OFF and follow tih 0x8B
                                0x81, 0x80,         // Display contrast set to second byte
                                OLED_CMD_DISPLAY_RAM,
                                OLED_CMD_DISPLAY_NORMAL,
                                OLED_CMD_SET_MUX_RATIO, 0x3F,   // Init multiplex ration to standard value
                                OLED_CMD_DISPLAY_ON
};

// --- OledDisplayNode Class Definition ---
class OledDisplayNode : public rclcpp::Node
{
public:
    OledDisplayNode() : Node("oled_display_node"), display_initialized_(false)
    {
        // Declare parameters
        this->declare_parameter<std::string>("i2c_device", "/dev/i2c-1");
        this->declare_parameter<int>("i2c_address", SH1106_OLED_I2C_ADDRESS); // Default to SH1106 addr
        this->declare_parameter<int>("display_type", DISPLAY_TYPE_SH1106); // Default to SH1106
        this->declare_parameter<double>("update_rate_hz", 0.5); // Rate for periodic status updates
        this->declare_parameter<double>("init_delay_s", 1.0);
        this->declare_parameter<double>("update_delay_s", 0.25);

        // Get parameters
        i2c_device_ = this->get_parameter("i2c_device").as_string();
        int i2c_address = this->get_parameter("i2c_address").as_int();
        int display_type = this->get_parameter("display_type").as_int();
        double update_rate_hz = this->get_parameter("update_rate_hz").as_double();
        init_delay_s_ = this->get_parameter("init_delay_s").as_double();
        update_delay_s_ = this->get_parameter("update_delay_s").as_double();

        RCLCPP_INFO(this->get_logger(), "Using I2C device: %s, Address: %#X, Type: %d",
                    i2c_device_.c_str(), i2c_address, display_type);

        // Initialize OLED display driver
        int dispError = dispOled_init(this->get_logger(), i2c_device_, &oled_ctx_, display_type, static_cast<uint8_t>(i2c_address));

        if (dispError == 0) {
            RCLCPP_INFO(this->get_logger(), "OLED Display Initialized Successfully.");
            display_initialized_ = true;
            dispOled_clearDisplay(this->get_logger(), &oled_ctx_);
            this->sleep_for(std::chrono::duration<double>(init_delay_s_)); // Allow time for display clear

            // Display initial info (hostname, IP)
            std::string hostname = getPopen("uname -n");
            hostname.pop_back(); // Remove trailing newline from popen
            std::string eth_addr, wlan_addr;
            getIpAddressses(this->get_logger(), eth_addr, wlan_addr, 1);

            dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_HOSTNAME, 0, DISP_WRITE_TEXT_LEFT, hostname.c_str());
            this->sleep_for(std::chrono::duration<double>(update_delay_s_));
            dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_IP_WLAN, 0, DISP_WRITE_TEXT_LEFT, wlan_addr.c_str());
            this->sleep_for(std::chrono::duration<double>(update_delay_s_));
            dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_IP_ETH, 0, DISP_WRITE_TEXT_LEFT, eth_addr.c_str());
            this->sleep_for(std::chrono::duration<double>(update_delay_s_));

        } else {
            RCLCPP_ERROR(this->get_logger(), "OLED Display Initialization Failed! Error code: %d", dispError);
            // Node will continue but display won't work
        }

        // Create subscriber for display commands
        display_subscription_ = this->create_subscription<oled_display_node::msg::DisplayOutput>(
            "display_output", 10, std::bind(&OledDisplayNode::display_callback, this, _1));

        // TODO: Add subscribers for battery_state, motor_power_active, firmware_version if needed
        // battery_subscription_ = this->create_subscription<sensor_msgs::msg::BatteryState>(...)
        // motor_power_subscription_ = this->create_subscription<std_msgs::msg::Bool>(...)
        // firmware_subscription_ = this->create_subscription<std_msgs::msg::String>(...)

        // Create a timer for periodic status updates
        if (update_rate_hz > 0 && display_initialized_) {
             timer_ = this->create_wall_timer(
                 std::chrono::duration<double>(1.0 / update_rate_hz),
                 std::bind(&OledDisplayNode::timer_callback, this));
        }

        RCLCPP_INFO(this->get_logger(), "OLED Display Node started.");
    }

private:
    void display_callback(const oled_display_node::msg::DisplayOutput::SharedPtr msg)
    {
        if (!display_initialized_) {
            RCLCPP_WARN_ONCE(this->get_logger(), "Display not initialized, ignoring display command.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Received display command: Action=%d, Row=%d, Col=%d, Text='%s'",
                    msg->action_type, msg->row, msg->column, msg->text.c_str());

        int segment = msg->column * DISPLAY_CHAR_WIDTH;
        int dispRow = msg->row;
        int maxChars = oled_ctx_.maxColumn; // Max chars per line
        int lineChars = msg->text.length();

        if (msg->num_chars > 0 && msg->num_chars < lineChars) {
            lineChars = msg->num_chars;
        }

        if (lineChars > maxChars) {
            lineChars = maxChars; // Truncate if too long
        }

        std::string text_to_display = msg->text.substr(0, lineChars);

        switch (msg->action_type) {
            case oled_display_node::msg::DisplayOutput::DISPLAY_ALL:
                // Clear the specific line first? Or assume overwrite is fine?
                // Let's clear the line for simplicity, though less efficient.
                {
                    std::string clear_line(maxChars, ' ');
                    dispOled_writeText(this->get_logger(), &oled_ctx_, dispRow, 0, DISP_WRITE_TEXT_LEFT, clear_line.c_str());
                }
                dispOled_writeText(this->get_logger(), &oled_ctx_, dispRow, segment, DISP_WRITE_TEXT_LEFT, text_to_display.c_str());
                RCLCPP_DEBUG(this->get_logger(), "Action: DISPLAY_ALL on row %d", dispRow);
                break;
            case oled_display_node::msg::DisplayOutput::DISPLAY_SUBSTRING:
                dispOled_writeText(this->get_logger(), &oled_ctx_, dispRow, segment, DISP_WRITE_TEXT_LEFT, text_to_display.c_str());
                RCLCPP_DEBUG(this->get_logger(), "Action: DISPLAY_SUBSTRING on row %d, col %d", dispRow, msg->column);
                break;
            case oled_display_node::msg::DisplayOutput::DISPLAY_STARTUP_STRING:
                 RCLCPP_WARN(this->get_logger(), "Action DISPLAY_STARTUP_STRING not implemented.");
                break;
            case oled_display_node::msg::DisplayOutput::DISPLAY_SET_BRIGHTNESS:
                 RCLCPP_WARN(this->get_logger(), "Action SET_BRIGHTNESS not supported by hardware/driver.");
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown action type: %d", msg->action_type);
                break;
        }
    }

    void timer_callback() // Periodic status update
    {
        if (!display_initialized_) return;

        RCLCPP_DEBUG(this->get_logger(), "Timer tick - updating status lines");

        // Update Hostname
        std::string hostname = getPopen("uname -n");
        hostname.pop_back(); // Remove trailing newline
        dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_HOSTNAME, 0, DISP_WRITE_TEXT_LEFT, hostname.c_str());
        this->sleep_for(std::chrono::duration<double>(update_delay_s_));

        // Update IP Addresses
        std::string eth_addr, wlan_addr;
        getIpAddressses(this->get_logger(), eth_addr, wlan_addr, 0);
        dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_IP_WLAN, 0, DISP_WRITE_TEXT_LEFT, wlan_addr.c_str());
        this->sleep_for(std::chrono::duration<double>(update_delay_s_));
        dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_IP_ETH, 0, DISP_WRITE_TEXT_LEFT, eth_addr.c_str());
        this->sleep_for(std::chrono::duration<double>(update_delay_s_));

        // TODO: Update Battery and Motor/FW status lines if those subscribers are added
        // Example:
        // std::stringstream battStream;
        // battStream << "Bat:" << std::fixed << std::setprecision(1) << g_batteryVoltage << "V" << g_batteryPercentage;
        // dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_BATT_VOLTS, 0, DISP_WRITE_TEXT_LEFT, battStream.str().c_str());
        // this->sleep_for(std::chrono::duration<double>(update_delay_s_));

        // std::stringstream infoStream;
        // infoStream << "Mot:" << g_motorPowerActive << " FW:" << g_firmwareVersion;
        // dispOled_writeText(this->get_logger(), &oled_ctx_, DISP_LINE_MOTOR_POWER, 0, DISP_WRITE_TEXT_LEFT, infoStream.str().c_str());
        // this->sleep_for(std::chrono::duration<double>(update_delay_s_));
    }

    // Helper to sleep without busy-waiting
    void sleep_for(std::chrono::duration<double> duration) {
        this->get_clock()->sleep_for(rclcpp::Duration::from_seconds(duration.count()));
    }

    rclcpp::Subscription<oled_display_node::msg::DisplayOutput>::SharedPtr display_subscription_;
    // Add other subscriptions here (battery, motor, firmware)
    rclcpp::TimerBase::SharedPtr timer_;
    std::string i2c_device_;
    dispCtx_t oled_ctx_; // OLED Display context
    bool display_initialized_;
    double init_delay_s_;
    double update_delay_s_;

    // TODO: Add member variables for battery state, motor power, firmware version
    // double g_batteryVoltage = 0.0;
    // std::string g_batteryPercentage = "    ";
    // std::string g_motorPowerActive = "OFF";
    // std::string g_firmwareVersion = "---";
};

// --- Main Function ---
int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OledDisplayNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

// --- Driver Function Implementations (Copied and adapted from ROS1) ---

/*
 * @name                i2c_read
 * ... (rest of documentation copied from ROS1 source)
 */
static int i2c_read(rclcpp::Logger logger, const char *i2cDevFile, uint8_t i2c7bitAddr,
                                        uint8_t *pBuffer, int numBytes, uint8_t chipRegAddr, bool chipRegAddrFlag)
{
    int       fd;                                         // File descriptor
    const int slaveAddress = i2c7bitAddr;                 // Address of the I2C device
    int       retCode   = 0;
    int       bytesRead = 0;
    struct    i2c_msg msgs[2];                        // Low level representation of one segment of an I2C transaction
    struct    i2c_rdwr_ioctl_data msgset[1];          // Set of transaction segments

    if ((fd = open(i2cDevFile, O_RDWR)) < 0) {        // Open port for reading and writing
        RCLCPP_ERROR(logger, "Cannot open I2C dev %s: %s", i2cDevFile, strerror(errno));
        retCode = IO_ERR_DEV_OPEN_FAILED;
        goto exitWithNoClose;
    }

    if (chipRegAddrFlag) {
            msgs[0].addr = slaveAddress;
            msgs[0].flags = 0;                            // Write bit
            msgs[0].len = 1;                              // Slave Address/byte written to I2C slave address
            msgs[0].buf = &chipRegAddr;                   // Internal Chip Register Address
            msgs[1].addr = slaveAddress;
            msgs[1].flags = I2C_M_RD | I2C_M_NOSTART;     // Read bit or Combined transaction bit
            msgs[1].len = numBytes;                       // Number of bytes read
            msgs[1].buf = pBuffer;                        // Output read buffer

            msgset[0].msgs = msgs;
            msgset[0].nmsgs = 2;                          // number of transaction segments (write and read)

            // The ioctl here will execute I2C transaction with kernel enforced lock
            if (ioctl(fd, I2C_RDWR, &msgset) < 0) {
                    RCLCPP_ERROR(logger, "Failed I2C_RDWR ioctl on %s: %s", i2cDevFile, strerror(errno));
                    retCode = IO_ERR_IOCTL_ADDR_SET;
                    goto exitWithFileClose;
            }
    }
    else {
        // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
        if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {    // Set the port options and addr of the dev
                RCLCPP_ERROR(logger, "Failed I2C_SLAVE ioctl on %s: %s", i2cDevFile, strerror(errno));
                retCode = IO_ERR_IOCTL_ADDR_SET;
                goto exitWithFileClose;
        }

            bytesRead = read(fd, pBuffer, numBytes);
            if (bytesRead != numBytes) {                  // Verify that the number of bytes we requested were read
                RCLCPP_ERROR(logger, "Failed to read %d bytes from %s: %s (read %d)", numBytes, i2cDevFile, strerror(errno), bytesRead);
                retCode = IO_ERR_READ_FAILED;
                goto exitWithFileClose;
            }
    }
    exitWithFileClose:
    close(fd);

    exitWithNoClose:

    // Read is odd in that + is num bytes read so make errors negative
    if (retCode == 0) {
            retCode = numBytes;
    } else {
            retCode = retCode * -1;
    }

    return retCode;
}

/*
 * @name                i2c_write
 * ... (rest of documentation copied from ROS1 source)
 */
static int i2c_write(rclcpp::Logger logger, const char *i2cDevFile, uint8_t i2c7bitAddr, uint8_t *pBuffer, int numBytes)
{
    int        fd;                      // File descriptor
    int        retCode = 0;
    const int  slaveAddress = i2c7bitAddr;      // Address of the I2C device

    // Open port for writing
    if ((fd = open(i2cDevFile, O_WRONLY)) < 0) {
        RCLCPP_ERROR(logger, "Cannot open I2C dev %s for write: %s", i2cDevFile, strerror(errno));
        retCode = IO_ERR_DEV_OPEN_FAILED;
        goto exitWithNoClose;
    }

    // The ioctl here will address the I2C slave device making it ready for 1 or more other bytes
    if (ioctl(fd, I2C_SLAVE, slaveAddress) != 0) {  // Set the port options and addr of the dev
        RCLCPP_ERROR(logger, "Failed I2C_SLAVE ioctl on %s: %s", i2cDevFile, strerror(errno));
        retCode = IO_ERR_IOCTL_ADDR_SET;
        goto exitWithFileClose;
    }

    if (write(fd, pBuffer, numBytes) != numBytes) {
            RCLCPP_ERROR(logger, "Failed to write %d bytes to %s: %s", numBytes, i2cDevFile, strerror(errno));
            retCode = IO_ERR_WRITE_FAILED;
            goto exitWithFileClose;
    }

    exitWithFileClose:
    close(fd);

    exitWithNoClose:

    return retCode;
}

/*
 * @name                dispOled_detectDisplayType
 * ... (rest of documentation copied from ROS1 source)
 */
#define MX_DEV_NAME_LEN 32
int dispOled_detectDisplayType(rclcpp::Logger logger, std::string devName, uint8_t i2c7bitAddr, int *dispType)
{
    uint8_t buf[16];
    int     retCode = 0;
    int     retCount = 0;
    int     i = 0;
    int     vote1106 = 0;
    int     vote1306 = 0;
    char    device[MX_DEV_NAME_LEN];
    strncpy(&device[0], devName.c_str(), MX_DEV_NAME_LEN);
    device[(MX_DEV_NAME_LEN-1)] = 0;     // protect against long dev names

    // We have seen incorrect values read sometimes and because this chip relies on
    // a status register in the SH1106 we better read a few times and 'vote'
    for (i=0 ; i < 5 ; i++) {
            // Read the status register at chip addr 0 to decide on chip type - set flag to true
            retCount = i2c_read(logger, &device[0], i2c7bitAddr, &buf[0], 1, 0x00, true);
            if (retCount < 0) {
                    RCLCPP_ERROR(logger, "Error %d in reading OLED status register at 7bit I2CAddr %#x",
                            retCount, i2c7bitAddr);
                    *dispType = DISPLAY_TYPE_NONE;
                    retCode = IO_ERR_READ_FAILED;
            } else if (retCount != 1) {
                    RCLCPP_ERROR(logger, "Cannot read byte from OLED status register at 7bit Addr %#x",
                    i2c7bitAddr);
                    *dispType = DISPLAY_TYPE_NONE;
                    retCode = IO_ERR_READ_LENGTH;;
            } else {
                    RCLCPP_INFO(logger, "Read OLED status register as %#02x on pass %d", buf[0],i);
                    if ((buf[0] & 0x07) == 0x06) {
                            // We found lower 3 bit as a 6 but datasheet does not spec it
                            vote1306++;
                    } else {
                            // Data sheet guarentees lower 3 bits as 0
                            // We are going to vote by assumption that this is a SSD1306
                            vote1106++;
                    }
            }
            usleep(30000); // Use rclcpp::Rate::sleep() or std::this_thread::sleep_for in ROS2 nodes
    }

    // count the votes and set display type
    if (vote1106 > vote1306) {
            *dispType = DISPLAY_TYPE_SH1106;
    } else {
            *dispType = DISPLAY_TYPE_SSD1306;
    }

    return retCode;
}

/*
 * @name                dispOled_initCtx
 * ... (rest of documentation copied from ROS1 source)
 */
int dispOled_initCtx(rclcpp::Logger logger, std::string devName, dispCtx_t *dispCtx, int dispType, uint8_t i2cAddr)
{
    int retCode = 0;

    dispCtx->dispType = dispType;
    dispCtx->i2cAddr = i2cAddr;
    strncpy(&dispCtx->devName[0], devName.c_str(), sizeof(dispCtx->devName) - 1);
    dispCtx->devName[sizeof(dispCtx->devName) - 1] = '\0'; // Ensure null termination

    // Set max lines and horizontal segments for the display in use
    switch (dispType) {
    case DISPLAY_TYPE_SSD1306:
            dispCtx->maxLine      = SSD1306_MAX_LINE;
            dispCtx->maxColumn    = SSD1306_MAX_COLUMN;
            dispCtx->maxVertPixel = SSD1306_MAX_VERT_PIXEL;
            dispCtx->maxHorzPixel = SSD1306_MAX_HORZ_PIXEL;
            dispCtx->horzOffset   = SSD1306_HORZ_OFFSET;
            dispCtx->endHorzPixel = SSD1306_END_HORZ_PIXEL;
            break;
    case DISPLAY_TYPE_SH1106:
            dispCtx->maxLine      = SH1106_MAX_LINE;
            dispCtx->maxColumn    = SH1106_MAX_COLUMN;
            dispCtx->maxVertPixel = SH1106_MAX_VERT_PIXEL;
            dispCtx->maxHorzPixel = SH1106_MAX_HORZ_PIXEL;
            dispCtx->horzOffset   = SH1106_HORZ_OFFSET;
            dispCtx->endHorzPixel = SH1106_END_HORZ_PIXEL;
            break;
    default:
            RCLCPP_ERROR(logger, "Unsupported display type: %d", dispType);
            retCode = IO_ERR_BAD_DISP_CONTEXT;
            break;
    }

    return retCode; // Changed from 0 to retCode
}

/*
 * @name                dispOled_init
 * ... (rest of documentation copied from ROS1 source)
 */
int dispOled_init(rclcpp::Logger logger, std::string devName, dispCtx_t *dispCtx, int displayType, uint8_t i2cAddr)
{
    int retCode = 0;
    int dispType = displayType;

    // If this is called with AUTO we try to autodetect the display
    if (dispType == DISPLAY_TYPE_AUTO) {
            // Auto-detect display. Detects if display present and type of OLED display
            RCLCPP_INFO(logger, "Attempting to auto-detect display type...");
            retCode = dispOled_detectDisplayType(logger, devName, i2cAddr, &dispType);
            if (retCode != 0 || dispType == DISPLAY_TYPE_NONE) {
                    RCLCPP_ERROR(logger, "Auto-detection failed or display not found.");
                    return (retCode != 0) ? retCode : IO_ERR_DEV_OPEN_FAILED; // Return specific error or generic one
            }
            RCLCPP_INFO(logger, "Auto-detected display type: %d", dispType);
    }

    retCode = dispOled_initCtx(logger, devName, dispCtx, dispType, i2cAddr);
    if (retCode != 0) {
        return retCode;
    }

    //Send all the commands to fully initialize the device.
    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
            // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
            RCLCPP_INFO(logger, "Setup for SSD1306 controller on the OLED display.");
            retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, &ssd1306_init_bytes[0], SSD1306_INIT_BYTE_COUNT);
            break;
    case DISPLAY_TYPE_SH1106:
            // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
            RCLCPP_INFO(logger, "Setup for SH1106 controller on the OLED display.");
            retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, &sh1106_init_bytes[0], SH1106_INIT_BYTE_COUNT);
            break;
    default:
            // This case should not be reached if initCtx worked, but added for safety
            RCLCPP_ERROR(logger, "Invalid display type %d during init sequence.", dispCtx->dispType);
            retCode = IO_ERR_BAD_DISP_CONTEXT;
            break;
    }
    if (retCode != 0) {
            RCLCPP_ERROR(logger, "Setup for OLED display failed with error code %d", retCode);
    }

    return retCode;
}

/*
 * @name                dispOled_setCursor
 * ... (rest of documentation copied from ROS1 source)
 */
int dispOled_setCursor(rclcpp::Logger logger, dispCtx_t *dispCtx, int column, int line) {
    int retCode = 0;
    uint8_t cursorSetup[8];

    // Basic bounds check
    if (line < 0 || line > dispCtx->maxLine || column < 0 || column > dispCtx->maxHorzPixel) {
        RCLCPP_WARN(logger, "SetCursor out of bounds: line %d, column %d", line, column);
        return -1; // Indicate invalid parameters
    }

    switch (dispCtx->dispType) {
    case DISPLAY_TYPE_SSD1306:
                    cursorSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                    cursorSetup[1] = OLED_CMD_SET_COLUMN_RANGE;
                    cursorSetup[2] = column;            // Start of printing from left seg as 0
                    cursorSetup[3] = dispCtx->maxHorzPixel;     // last index of printing segments
                    cursorSetup[4] = OLED_CMD_SET_PAGE_RANGE;
                    cursorSetup[5] = line;                      // We assume only one line written to at a time
                    cursorSetup[6] = line;                      // We assume only one line written to at a time

                    // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                    retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, &cursorSetup[0], 7);
                    break; // Added missing break

    case DISPLAY_TYPE_SH1106:
                    // SH1106 has different addressing than SSD1306
                    cursorSetup[0] = OLED_CONTROL_BYTE_CMD_STREAM;
                    cursorSetup[1] = 0xB0 | (line & 0xf);
                    cursorSetup[2] = 0x10 | (((column + dispCtx->horzOffset) & 0xf0) >> 4);  // Upper column address
                    cursorSetup[3] = 0x00 | ((column + dispCtx->horzOffset)  & 0xf);         // Lower column address

                    // We treat the 1st byte sort of like a 'register' but it is really a command stream mode to the chip
                    retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, &cursorSetup[0], 4);
                    break;
    default:
                    RCLCPP_ERROR(logger, "SetCursor called with invalid display type %d", dispCtx->dispType);
                    retCode = IO_ERR_BAD_DISP_CONTEXT;
                    break;
    }

    return retCode;
}

/*
 * @name                dispOled_clearDisplay
 * ... (rest of documentation copied from ROS1 source)
 */
int dispOled_clearDisplay(rclcpp::Logger logger, dispCtx_t *dispCtx) {
    int retCode = 0;

    // Calculate buffer size needed (Control byte + pixels)
    // Need to handle potential offset and unused pixels correctly
    int pixels_to_clear = dispCtx->maxHorzPixel + 1; // Usually 0-127 = 128 pixels
    if (dispCtx->dispType == DISPLAY_TYPE_SH1106) {
        // SH1106 might need offset handling depending on how writeText works
        // Assuming writeText handles offset, we clear 128 pixels.
        pixels_to_clear = 128; // Clear the standard 128 width
    }
    int buffer_size = 1 + pixels_to_clear;
    std::vector<uint8_t> zero_buffer(buffer_size, 0);
    zero_buffer[0] = OLED_CONTROL_BYTE_DATA_STREAM;

    for (uint8_t line = 0; line <= dispCtx->maxLine; line++) {
        retCode |= dispOled_setCursor(logger, dispCtx, 0, line);
        if (retCode != 0) {
            RCLCPP_ERROR(logger, "Failed to set cursor for line %d during clearDisplay", line);
            return retCode;
        }

        // Clear one line
        retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, zero_buffer.data(), buffer_size);
        if (retCode != 0) {
            RCLCPP_ERROR(logger, "Failed to write clear data for line %d", line);
            return retCode;
        }
    }

    // Set cursor back to home position after clearing
    retCode |= dispOled_setCursor(logger, dispCtx, 0, 0);

    return retCode;
}

/*
 * @name                dispOled_writeText
 * ... (rest of documentation copied from ROS1 source)
 */
int dispOled_writeText(rclcpp::Logger logger, dispCtx_t *dispCtx, uint8_t line, uint8_t segment, uint8_t center, const char *textStr) {
    int           retCode = 0;
    const char    *text = textStr;
    size_t        text_len = strlen(text);
    int           startSegment = segment;

    if (!text) {
        RCLCPP_WARN(logger, "writeText called with null string");
        return -1;
    }

    if (line > dispCtx->maxLine) {
        RCLCPP_WARN(logger, "writeText called with invalid line %d (max %d)", line, dispCtx->maxLine);
        return -1;              // out of range line
    }

    // Calculate starting segment based on centering
    if (center != 0) {
        int text_width_pixels = text_len * DISPLAY_CHAR_WIDTH;
        if (text_width_pixels > (dispCtx->maxHorzPixel + 1)) {
            // Text wider than display, start at 0
            startSegment = 0;
        } else {
            startSegment = ((dispCtx->maxHorzPixel + 1) - text_width_pixels) / 2;
        }
    } else {
        // Ensure start segment is within bounds if not centering
        if (startSegment > dispCtx->maxHorzPixel) {
            startSegment = dispCtx->maxHorzPixel;
        }
    }

    // Check if text will exceed display width from the starting segment
    if ((startSegment + (text_len * DISPLAY_CHAR_WIDTH)) > (dispCtx->maxHorzPixel + 1)) {
        // Text exceeds display width, truncate it
        text_len = (dispCtx->maxHorzPixel + 1 - startSegment) / DISPLAY_CHAR_WIDTH;
        if (startSegment > dispCtx->maxHorzPixel) text_len = 0; // Handle case where start is already off screen
        RCLCPP_DEBUG(logger, "Truncating text to %zu chars to fit display width from segment %d", text_len, startSegment);
    }

    if (text_len == 0) {
        RCLCPP_DEBUG(logger, "Text length is zero after truncation/check, nothing to write.");
        return 0; // Nothing to write
    }

    retCode |= dispOled_setCursor(logger, dispCtx, startSegment, line);
    if (retCode != 0) {
        RCLCPP_ERROR(logger, "Failed to set cursor before writing text.");
        return retCode;
    }

    // Prepare buffer for pixel data
    int data_bytes_needed = text_len * DISPLAY_CHAR_WIDTH;
    int buffer_size = 1 + data_bytes_needed; // Control byte + data bytes
    std::vector<uint8_t> dispData(buffer_size);
    dispData[0] = OLED_CONTROL_BYTE_DATA_STREAM;     // Data starts on byte after this 1st one
    int dispDataIdx = 1;

    // Form pixels to send as data by lookup in font table
    for (size_t i = 0; i < text_len; i++) {
        // Ensure character is within font table range (basic ASCII)
        unsigned char char_code = static_cast<unsigned char>(text[i]);
        if (char_code >= 256) { // Basic check, font table size is 256
             RCLCPP_WARN(logger, "Character %c (code %d) out of font range, skipping.", text[i], char_code);
             // Fill with spaces or skip? Let's fill with spaces.
             char_code = ' '; // Replace with space
        }

        // For each column of pixels for this char send a data byte which is one vertical column of pixels
        for (uint8_t charCol = 0; charCol < DISPLAY_CHAR_WIDTH; charCol++) {
            if (dispDataIdx < buffer_size) { // Bounds check
                dispData[dispDataIdx++] = font8x8_basic_tr[char_code][charCol];
            } else {
                RCLCPP_ERROR(logger, "Buffer overflow detected while preparing text data!");
                return -3; // Indicate internal error
            }
        }
    }

    // Write the data to display
    retCode |= i2c_write(logger, &dispCtx->devName[0], dispCtx->i2cAddr, dispData.data(), buffer_size);
    if (retCode != 0) {
        RCLCPP_ERROR(logger, "Failed to write text data to display.");
    }

    return retCode;
}

/*
 * @name    getPopen
 * ... (rest of documentation copied from ROS1 source)
 */
std::string getPopen(std::string input) {
    std::string result("");
    char buffer[128];
    FILE* pipe = popen(input.c_str(), "r");
    if (!pipe) {
        // Consider logging an error here using a logger instance if available
        // For now, return empty string
        return "";
    }
    try {
        while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
            result += buffer;
        }
    } catch (...) {
        pclose(pipe);
        throw; // Re-throw the exception
    }
    pclose(pipe);
    return result;
}

/*
 * @name getIpAddressses
 * ... (rest of documentation copied from ROS1 source)
 */
void getIpAddressses(rclcpp::Logger logger, std::string &ethAddress, std::string &wlanAddress, int logFindings) {
    ethAddress = "No ETH"; // Default values
    wlanAddress = "No WLAN";

    // Use more robust parsing if possible, popen + grep + awk is fragile
    std::string enetIpOutput = getPopen("ip -o -4 addr show dev eth0 | awk '{print $4}'");
    std::string wlanIpOutput = getPopen("ip -o -4 addr show dev wlan0 | awk '{print $4}'");

    // Trim newline and potential CIDR suffix
    if (!enetIpOutput.empty()) {
        enetIpOutput.pop_back(); // Remove trailing newline
        size_t slash_pos = enetIpOutput.find('/');
        if (slash_pos != std::string::npos) {
            ethAddress = enetIpOutput.substr(0, slash_pos);
        } else {
             ethAddress = enetIpOutput; // Should not happen with ip -o -4 addr
        }
    }

    if (!wlanIpOutput.empty()) {
        wlanIpOutput.pop_back(); // Remove trailing newline
        size_t slash_pos = wlanIpOutput.find('/');
        if (slash_pos != std::string::npos) {
            wlanAddress = wlanIpOutput.substr(0, slash_pos);
        } else {
            wlanAddress = wlanIpOutput;
        }
    }

    if (logFindings != 0) {
        RCLCPP_INFO(logger, "ETH0 IP: %s", ethAddress.c_str());
        RCLCPP_INFO(logger, "WLAN0 IP: %s", wlanAddress.c_str());
    }
}

// Note: Removed battery, motor power, firmware callbacks and related global variables.
// These need to be re-implemented using ROS2 subscribers and member variables if needed.
