#include "BleHidController.h"
#include "soc/soc_caps.h"
#include <cstring>

#if SOC_USB_OTG_SUPPORTED && defined(ARDUINO_USB_MODE) && ARDUINO_USB_MODE == 0
#include "USB.h"
#include "USBHID.h"
#define PINBALL_USB_HID_ENABLED 1
#else
#define PINBALL_USB_HID_ENABLED 0
#endif

static constexpr uint32_t USB_DETECTION_WINDOW_MS = 2500;

//@formatter:off
// Combined report map (keyboard + gamepad)
static const uint8_t hidReportMapData[] = {

    // **** Keyboard (Report ID 1) ****

    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x06,               // Usage (Keyboard)
    0xA1, 0x01,               // Collection (Application)
    0x85, REPORT_ID_KEYBOARD, // Report ID

    // Modifiers (Ctrl, Shift, Alt, Meta)
    0x05, 0x07,               // Usage Page (Keyboard)
    0x19, 0xE0,               // Usage Minimum (Left Control)
    0x29, 0xE7,               // Usage Maximum (Right Meta)
    0x15, 0x00,               // Logical Minimum 0
    0x25, 0x01,               // Logical Maximum 1
    0x95, 0x08,               // Report Count 8
    0x75, 0x01,               // Report Size 1 bit
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // 6 simultaneous keys (keycode table)
    0x05, 0x07,               // Usage Page (Keyboard)
    0x19, 0x00,               // Usage Minimum (0 = No events)
    0x29, 0xFF,               // Usage Maximum (0xFF)
    0x15, 0x00,               // Logical Minimum 0
    0x26, 0xFF, 0x00,         // Logical Maximum 0x00FF
    0x95, 0x06,               // Report Count 6
    0x75, 0x08,               // Report Size 8 bits
    0x81, 0x00,               // Input (Data, Array, Absolute)

    // End Collection
    0xC0,

    // **** Gamepad (Report ID 2) ****
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x05,               // Usage (Gamepad)
    0xA1, 0x01,               // Collection (Application)
    0x85, REPORT_ID_GAMEPAD,  // Report ID

    // 16 buttons
    0x05, 0x09,               // Usage Page (Button)
    0x19, 0x01,               // Usage Minimum (Button 1)
    0x29, 0x10,               // Usage Maximum (Button 16)
    0x15, 0x00,               // Logical Minimum 0
    0x25, 0x01,               // Logical Maximum 1
    0x75, 0x01,               // Report Size 1 bit
    0x95, 0x10,               // Report Count 16
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // D-Pad (hat switch)
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x39,               // Usage (Hat switch)
    0x15, 0x00,               // Logical Min 0
    0x25, 0x07,               // Logical Max 7 (8 directions)
    0x75, 0x04,               // Report Size 4 bits
    0x95, 0x01,               // Report Count 1
    0x81, 0x42,               // Input (Data, Variable, Absolute, Null state)
    0x75, 0x04,               // Report Size 4 bits (padding)
    0x95, 0x01,               // Report Count 1
    0x81, 0x03,               // Input (Constant, Variable, Absolute)

    // Left stick: X, Y
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x30,               // Usage X
    0x09, 0x31,               // Usage Y
    0x16, 0x00, 0x80,         // Logical Min -32768
    0x26, 0xFF, 0x7F,         // Logical Max 32767
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // Right stick: Rx, Ry
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x33,               // Usage Rx
    0x09, 0x34,               // Usage Ry
    0x16, 0x00, 0x80,         // Logical Min -32768
    0x26, 0xFF, 0x7F,         // Logical Max 32767
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // Triggers LT/RT: Z, Rz
    0x05, 0x01,               // Usage Page (Generic Desktop)
    0x09, 0x32,               // Usage Z (LT)
    0x09, 0x35,               // Usage Rz (RT)
    0x15, 0x00,               // Logical Min -0
    0x26, 0xFF, 0x03,         // Logical Max 1023
    0x75, 0x10,               // Report Size 16 bits
    0x95, 0x02,               // Report Count 2
    0x81, 0x02,               // Input (Data, Variable, Absolute)

    // End Collection
    0xC0
};
//@formatter:on

#if PINBALL_USB_HID_ENABLED
static const uint8_t usbReportMapData[] = {
    TUD_HID_REPORT_DESC_KEYBOARD(HID_REPORT_ID(REPORT_ID_KEYBOARD)),
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(REPORT_ID_GAMEPAD))
};

static USBHID usbHid;

struct UsbKeyboardReport
{
    uint8_t modifiers;
    uint8_t reserved;
    uint8_t keys[6];
};

struct __attribute__((packed)) UsbGamepadReport
{
    int8_t x;
    int8_t y;
    int8_t z;
    int8_t rz;
    int8_t rx;
    int8_t ry;
    uint8_t hat;
    uint32_t buttons;
};

static int8_t scaleUsbAxis(const int16_t value) {
    const int16_t scaled = value / 256;
    return static_cast<int8_t>(scaled < -127 ? -127 : scaled);
}

class PinballUsbHidDevice final : public USBHIDDevice
{
public:
    uint16_t _onGetDescriptor(uint8_t* buffer) override {
        memcpy(buffer, usbReportMapData, sizeof(usbReportMapData));
        return sizeof(usbReportMapData);
    }
};

static PinballUsbHidDevice usbHidDevice;
#endif

bool BleHidController::_deviceConnected = false;

// Internal callbacks
class BleHidController::ServerCallbacks : public NimBLEServerCallbacks
{
    void onConnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo) override {
        _deviceConnected = true;
        pSrv->updateConnParams(connInfo.getConnHandle(), 6, 7, 0, 600);
    }

    void onDisconnect(NimBLEServer* pSrv, NimBLEConnInfo& connInfo, int reason) override {
        _deviceConnected = false;
        NimBLEDevice::startAdvertising();
    }
};

// Class implementation
BleHidController::BleHidController() = default;


void BleHidController::begin(const char* deviceName, const char* deviceManufacturer, const uint16_t vendorId, const uint16_t productId, const uint16_t version) {
    if (_initialized) return;
    _initialized = true;
    _deviceName = deviceName;
    _deviceManufacturer = deviceManufacturer;
    _vendorId = vendorId;
    _productId = productId;
    _version = version;
    _transportStartedAt = millis();

#if PINBALL_USB_HID_ENABLED
    usbHid.addDevice(&usbHidDevice, sizeof(usbReportMapData));
    usbHid.begin();
    USB.begin();
#else
    startBle();
#endif
}

void BleHidController::updateTransport() {
    if (!_initialized || _bleStarted) return;

#if PINBALL_USB_HID_ENABLED
    if (static_cast<bool>(USB)) return;
    if (millis() - _transportStartedAt < USB_DETECTION_WINDOW_MS) return;
#endif

    startBle();
}

void BleHidController::startBle() {
    if (_bleStarted) return;
    _bleStarted = true;

    NimBLEDevice::init(_deviceName);
    NimBLEDevice::setPower(BLE_TX_POWER);
    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_BOND);
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);

    _server = NimBLEDevice::createServer();

    static ServerCallbacks serverCb;
    _server->setCallbacks(&serverCb);

    _server->advertiseOnDisconnect(false);

    _hidDevice = new NimBLEHIDDevice(_server);
    _hidDevice->setManufacturer(_deviceManufacturer);
    _hidDevice->setPnp(PNP_VENDOR_SRC_USB, _vendorId, _productId, _version);
    _hidDevice->setHidInfo(0x00, 0x01);
    _hidDevice->setBatteryLevel(BATTERY_LEVEL);
    _hidDevice->setReportMap(const_cast<uint8_t*>(hidReportMapData), sizeof(hidReportMapData));

    _kbInputReport = _hidDevice->getInputReport(REPORT_ID_KEYBOARD);
    _gpInputReport = _hidDevice->getInputReport(REPORT_ID_GAMEPAD);

    // _hidDevice->startServices(); // Deprecated: Services are now started by the server when start()

    NimBLEAdvertising* adv = NimBLEDevice::getAdvertising();
    adv->setName(_deviceName);
    adv->addServiceUUID(_hidDevice->getHidService()->getUUID());
    adv->addServiceUUID(_hidDevice->getBatteryService()->getUUID());
    adv->setAppearance(GENERIC_HID); // !HERE Meta Quest 3 doesn't seem to accept HID_GAMEPAD
    adv->enableScanResponse(true);
    adv->start();
}

bool BleHidController::isUsbConnected() const {
#if PINBALL_USB_HID_ENABLED
    return usbHid.ready();
#else
    return false;
#endif
}


// ***************************************************************
// KEYBOARD API
// ***************************************************************

void BleHidController::sendKeyboardState() {
    if (_deviceConnected && _kbInputReport != nullptr) {
        _kbInputReport->setValue(reinterpret_cast<uint8_t*>(&_kbState), sizeof(_kbState));
        (void)_kbInputReport->notify();
    }
#if PINBALL_USB_HID_ENABLED
    if (usbHid.ready()) {
        const UsbKeyboardReport report{
            .modifiers = _kbState.modifiers,
            .reserved = 0,
            .keys = {_kbState.keys[0], _kbState.keys[1], _kbState.keys[2],
                     _kbState.keys[3], _kbState.keys[4], _kbState.keys[5]}
        };
        usbHid.SendReport(REPORT_ID_KEYBOARD, &report, sizeof(report));
    }
#endif
}

/**
 * Add modifier bit and send report
 *
 * @param modifier
 */
void BleHidController::keyModPress(const uint8_t modifier) {
    _kbState.modifiers |= modifier;
    sendKeyboardState();
}

/**
 * Remove modifier bit and send report
 *
 * @param modifier
 */
void BleHidController::keyModRelease(const uint8_t modifier) {
    _kbState.modifiers &= ~modifier;
    sendKeyboardState();
}

/**
 * Add keycode to the first available slot in the keys array and send report
 *
 * @param keycode
 */
void BleHidController::keyPress(const uint8_t keycode) {
    if (keycode == KEY_NONE) return;

    // Verify if key already exists
    for (const unsigned char key : _kbState.keys) {
        if (key == keycode) {
            return;
        }
    }

    for (unsigned char& key : _kbState.keys) {
        if (key == 0x00) {
            key = keycode;
            break;
        }
    }
    sendKeyboardState();
}

/**
 * Remove keycode from the keys array and send report
 *
 * @param keycode
 */
void BleHidController::keyRelease(const uint8_t keycode) {
    // Remove the keycode from the keys array
    for (unsigned char& key : _kbState.keys) {
        if (key == keycode) {
            key = 0x00;
            break;
        }
    }
    sendKeyboardState();
}

/**
 * Clear all active key presses and modifier states and send report
 */
void BleHidController::keyReleaseAll() {
    _kbState = KeyReport{};
    sendKeyboardState();
}


// ***************************************************************
//  GAMEPAD API
// ***************************************************************

void BleHidController::sendGamepadState() {
    if (_deviceConnected && _gpInputReport != nullptr) {
        _gpInputReport->setValue(reinterpret_cast<uint8_t*>(&_gpState), sizeof(_gpState));
        (void)_gpInputReport->notify();
    }
#if PINBALL_USB_HID_ENABLED
    if (usbHid.ready()) {
        const UsbGamepadReport report{
            .x = scaleUsbAxis(_gpState.leftX),
            .y = scaleUsbAxis(_gpState.leftY),
            .z = static_cast<int8_t>((static_cast<uint32_t>(_gpState.lt) * 127) / 1023),
            .rz = static_cast<int8_t>((static_cast<uint32_t>(_gpState.rt) * 127) / 1023),
            .rx = scaleUsbAxis(_gpState.rightX),
            .ry = scaleUsbAxis(_gpState.rightY),
            .hat = static_cast<uint8_t>(_gpState.dpad == DPAD_CENTERED ? 0 : _gpState.dpad + 1),
            .buttons = _gpState.buttons
        };
        usbHid.SendReport(REPORT_ID_GAMEPAD, &report, sizeof(report));
    }
#endif
}

void BleHidController::buttonPress(const uint16_t button) {
    _gpState.buttons |= button;
    sendGamepadState();
}

void BleHidController::buttonRelease(const uint16_t button) {
    _gpState.buttons &= ~button;
    sendGamepadState();
}

void BleHidController::dpadPress(const uint8_t dpad) {
    _gpState.dpad = dpad;
    sendGamepadState();
}

void BleHidController::dpadRelease() {
    _gpState.dpad = DPAD_CENTERED;
    sendGamepadState();
}

void BleHidController::setLeftStick(const int16_t lx, const int16_t ly, const bool sendState) {
    _gpState.leftX = lx;
    _gpState.leftY = ly;
    if (sendState) sendGamepadState();
}

void BleHidController::setRightStick(const int16_t rx, const int16_t ry, const bool sendState) {
    _gpState.rightX = rx;
    _gpState.rightY = ry;
    if (sendState) sendGamepadState();
}

void BleHidController::setLeftTrigger(const uint16_t lt, const bool sendState) {
    _gpState.lt = lt;
    if (sendState) sendGamepadState();
}

void BleHidController::setRightTrigger(const uint16_t rt, const bool sendState) {
    _gpState.rt = rt;
    if (sendState) sendGamepadState();
}

void BleHidController::sendGamepad(uint16_t const buttons, const uint8_t dpad, const int16_t lx, const int16_t ly, const int16_t rx, const int16_t ry, const uint16_t lt, const uint16_t rt) {
    _gpState.buttons = buttons;
    _gpState.dpad    = dpad & 0x0F;
    _gpState.leftX   = lx;
    _gpState.leftY   = ly;
    _gpState.rightX  = rx;
    _gpState.rightY  = ry;
    _gpState.lt      = lt;
    _gpState.rt      = rt;
    sendGamepadState();
}
