/*
  This file is part of the MKRWAN library.
  Copyright (C) 2017  Arduino AG (http://www.arduino.cc/)

  Based on the TinyGSM library https://github.com/vshymanskyy/TinyGSM
  Copyright (c) 2016 Volodymyr Shymanskyy

  MKRWAN library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  MKRWAN library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with MKRWAN library.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"

#ifdef PORTENTA_CARRIER
#undef LORA_RESET
#define LORA_RESET (PD_5)
#undef LORA_BOOT0
#define LORA_BOOT0 (PJ_11)
#endif

#define DEFAULT_JOIN_TIMEOUT 60000L

template <class T, unsigned N>
class SerialFifo
{
public:
    SerialFifo()
    {
        clear();
    }

    void clear()
    {
        _r = 0;
        _w = 0;
    }

    // writing thread/context API
    //-------------------------------------------------------------

    bool writeable(void)
    {
        return free() > 0;
    }

    int free(void)
    {
        int s = _r - _w;
        if (s <= 0)
            s += N;
        return s - 1;
    }

    bool put(const T& c)
    {
        int i = _w;
        int j = i;
        i = _inc(i);
        if (i == _r) // !writeable()
            return false;
        _b[j] = c;
        _w = i;
        return true;
    }

    int put(const T* p, int n, bool t = false)
    {
        int c = n;
        while (c)
        {
            int f;
            while ((f = free()) == 0) // wait for space
            {
                if (!t) return n - c; // no more space and not blocking
                /* nothing / just wait */;
            }
            // check free space
            if (c < f) f = c;
            int w = _w;
            int m = N - w;
            // check wrap
            if (f > m) f = m;
            memcpy(&_b[w], p, f);
            _w = _inc(w, f);
            c -= f;
            p += f;
        }
        return n - c;
    }

    // reading thread/context API
    // --------------------------------------------------------

    bool readable(void)
    {
        return (_r != _w);
    }

    size_t size(void)
    {
        int s = _w - _r;
        if (s < 0)
            s += N;
        return s;
    }

    bool get(T* p)
    {
        int r = _r;
        if (r == _w) // !readable()
            return false;
        *p = _b[r];
        _r = _inc(r);
        return true;
    }

    bool peek(T* p)
    {
        int r = _r;
        if (r == _w) // !readable()
            return false;
        *p = _b[r];
        return true;
    }

    int get(T* p, int n, bool t = false)
    {
        int c = n;
        while (c)
        {
            int f;
            for (;;) // wait for data
            {
                f = size();
                if (f)  break;        // free space
                if (!t) return n - c; // no space and not blocking
                /* nothing / just wait */;
            }
            // check available data
            if (c < f) f = c;
            int r = _r;
            int m = N - r;
            // check wrap
            if (f > m) f = m;
            memcpy(p, &_b[r], f);
            _r = _inc(r, f);
            c -= f;
            p += f;
        }
        return n - c;
    }

private:
    int _inc(int i, int n = 1)
    {
        return (i + n) % N;
    }

    T    _b[N];
    int  _w;
    int  _r;
};

#ifndef YIELD
  #define YIELD() { delay(2); }
#endif

typedef const char* ConstStr;
#define GFP(x) x
#define GF(x)  x

#ifdef LORA_DEBUG
namespace {
  template<typename T>
  static void DBG(T last) {
    LORA_DEBUG.println(last);
  }

  template<typename T, typename... Args>
  static void DBG(T head, Args... tail) {
    LORA_DEBUG.print(head);
    LORA_DEBUG.print(' ');
    DBG(tail...);
  }
}
#else
  #define DBG(...)
#endif

template<class T>
const T& Min(const T& a, const T& b)
{
    return (b < a) ? b : a;
}

template<class T>
const T& Max(const T& a, const T& b)
{
    return (b < a) ? a : b;
}

#if !defined(LORA_RX_BUFFER)
  #define LORA_RX_BUFFER 256
#endif

/* AT Command strings. Commands start with AT */
#define AT_RESET      "+REBOOT"
#define AT_BAND       "+BAND"
#define AT_DEUI       "+DEVEUI"
#define AT_DADDR      "+DEVADDR"
#define AT_APPKEY     "+APPKEY"
#define AT_NWKSKEY    "+NWKSKEY"
#define AT_APPSKEY    "+APPSKEY"
#define AT_APPEUI     "+APPEUI"
#define AT_ADR        "+ADR"
#define AT_TXP        "+RFPOWER"
#define AT_FORMAT     "+DFORMAT"
#define AT_DR         "+DR"
#define AT_DCS        "+DUTYCYCLE"
#define AT_PNM        "+NWK"
#define AT_RX2FQ      "+RX2FQ"
#define AT_RX2DR      "+RX2DR"
#define AT_RX1DL      "+RX1DL"
#define AT_RX2DL      "+RX2DL"
#define AT_JN1DL      "+JN1DL"
#define AT_JN2DL      "+JN2DL"
#define AT_NJM        "+MODE"
#define AT_NWKID      "+IDNWK"
#define AT_FCU        "+FCU"
#define AT_FCD        "+FCD"
#define AT_CLASS      "+CLASS"
#define AT_JOIN       "+JOIN"
#define AT_NJS        "+NJS"
#define AT_SENDB      "+SENDB"
#define AT_SEND       "+SEND"
#define AT_RECVB      "+RECVB"
#define AT_RECV       "+RECV"
#define AT_UTX		  "+UTX"
#define AT_CTX		  "+CTX"
#define AT_PORT       "+PORT"
#define AT_VER        "+VER"
#define AT_DEV        "+DEV"
#define AT_CFM        "+CFM"
#define AT_CFS        "+CFS"
#define AT_SNR        "+SNR"
#define AT_RSSI       "+RSSI"
#define AT_BAT        "+BAT"
#define AT_TRSSI      "+TRSSI"
#define AT_TTONE      "+TTONE"
#define AT_TTLRA      "+TTLRA"
#define AT_TRLRA      "+TRLRA"
#define AT_TCONF      "+TCONF"
#define AT_TOFF       "+TOFF"
#define AT_CERTIF     "+CERTIF"
#define AT_CHANMASK   "+CHANMASK"
#define AT_CHANDEFMASK "+CHANDEFMASK"

#define AT_EVENT	  "+EVENT"
#define AT_UART		  "+UART"
#define AT_FACNEW	  "+FACNEW"
#define AT_SLEEP	  "+SLEEP"
#define AT_MSIZE	  "+MSIZE"
#define AT_EQ		  "="
#define AT_QM		  "?"

#define ARDUINO_LORA_MAXBUFF		64	// hard coded limit in middleware and driver

#define LORA_NL "\r"
static const char LORA_OK[] = "+OK";
static const char LORA_ERROR[] = "+ERR";
static const char LORA_ERROR_PARAM[] = "+ERR_PARAM";
static const char LORA_ERROR_BUSY[] = "+ERR_BUSY";
static const char LORA_ERROR_OVERFLOW[] = "+ERR_PARAM_OVERFLOW";
static const char LORA_ERROR_NO_NETWORK[] = "+ERR_NO_NETWORK";
static const char LORA_ERROR_RX[] = "+ERR_RX";
static const char LORA_ERROR_UNKNOWN[] = "+ERR_UNKNOWN";

static const char ARDUINO_FW_VERSION[] = "ARD-078 1.2.4";
static const char ARDUINO_FW_VERSION_AT[] = "ARD-078 1.2.4";
static const char ARDUINO_FW_IDENTIFIER[] = "ARD-078";

typedef enum {
    AS923 = 0,
    AU915,
    CN470,
    CN779,
    EU433,
    EU868 = 5,
    KR920,
    IN865,
    US915,
    US915_HYBRID,
} _lora_band;

typedef enum {
    RFO = 0,
    PABOOST,
} _rf_mode;

typedef enum {
    ABP = 0,
    OTAA,
} _lora_mode;

typedef enum {
    APP_EUI = 0,
    APP_KEY,
    DEV_EUI,
    DEV_ADDR,
    NWKS_KEY,
    APPS_KEY,
    NWK_ID,
} _lora_property;

typedef enum {
    CLASS_A = 'A',
    CLASS_B,
    CLASS_C,
} _lora_class;

class LoRaModem : public Stream // @suppress("Class has a virtual method and non-virtual destructor")
{

public:
  LoRaModem(__attribute__((unused)) Stream& stream = (Stream&)Serial)
#ifdef SerialLoRa
    : stream(SerialLoRa), lastPollTime(millis()), pollInterval(300000)
#else
    : stream(stream), lastPollTime(millis()), pollInterval(300000)
#endif
    {
	  network_joined = false;
	  mask_size = 1;
	  region = EU868;
	  compat_mode = false;
	  msize = ARDUINO_LORA_MAXBUFF;
    }

public:
  typedef SerialFifo<uint8_t, LORA_RX_BUFFER> RxFifo;

private:
  Stream&       stream;
  bool          network_joined;
  RxFifo        rx;
  RxFifo        tx;
  String        fw_version;
  unsigned long lastPollTime;
  unsigned long pollInterval;
  int           mask_size;
  uint16_t      channelsMask[6];
  String        channel_mask_str;
  _lora_band    region;
  bool			compat_mode;
  size_t		msize;

public:
  virtual int joinOTAA(const char *appEui, const char *appKey, const char *devEui, uint32_t timeout) {
    YIELD();
    rx.clear();
    changeMode(OTAA);
    set(APP_EUI, appEui);
    set(APP_KEY, appKey);
    if (devEui != NULL) {
        set(DEV_EUI, devEui);
    }
    network_joined = join(timeout);
    delay(1000);
    return network_joined;
  }

  virtual int joinOTAA(String appEui, String appKey, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), NULL, timeout);
  }

  virtual int joinOTAA(String appEui, String appKey, String devEui, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), devEui.c_str(), timeout);
  }

  virtual int joinABP(/*const char* nwkId, */const char * devAddr, const char * nwkSKey, const char * appSKey, uint32_t timeout = DEFAULT_JOIN_TIMEOUT) {
    YIELD();
    rx.clear();
    changeMode(ABP);
    //set(NWK_ID, nwkId);
    set(DEV_ADDR, devAddr);
    set(NWKS_KEY, nwkSKey);
    set(APPS_KEY, appSKey);
    network_joined = join(timeout);
    return (getJoinStatus() == 1);
  }

  virtual int joinABP(/*String nwkId, */String devAddr, String nwkSKey, String appSKey) {
    return joinABP(/*nwkId.c_str(), */devAddr.c_str(), nwkSKey.c_str(), appSKey.c_str());
  }

  // Stream compatibility (like UDP)
  void beginPacket() {
    tx.clear();
  }

  int endPacket(bool confirmed = false) {
    uint8_t buffer[LORA_RX_BUFFER];
    memset(buffer, 0, LORA_RX_BUFFER);
    int size = tx.get(buffer, tx.size());
    return modemSend(buffer, size, confirmed);
  }

  size_t write(uint8_t c) {
    return tx.put(c);
  };

  size_t write(const uint8_t *buffer, size_t size) {
    return tx.put(buffer, size);
  };

  template <typename T> inline size_t write(T val) {return write((uint8_t*)&val, sizeof(T));};
  using Print::write;

  int parsePacket() {
    return available();
  }

  virtual int available() {
    YIELD();
    if (!rx.size()) {
      maintain();
    }
    return rx.size(); // + buf_available;
  }

  virtual int read(uint8_t *buf, size_t size) {
    YIELD();
    maintain();
    size_t cnt = 0;
    while (cnt < size) {
      size_t chunk = Min(size-cnt, rx.size());
      if (chunk > 0) {
        rx.get(buf, chunk);
        buf += chunk;
        cnt += chunk;
        continue;
      }
      // TODO: Read directly into user buffer?
      maintain();
      /*
      if (buf_available > 0) {
        modemRead(rx.free());
      } else {
        break;
      }
      */
    }
    return cnt;
  }

  virtual int read() {
    uint8_t c;
    if (read(&c, 1) == 1) {
      return c;
    }
    return -1;
  }

  virtual int peek() {
    uint8_t c;
    if (rx.peek(&c) == true) {
      return c;
    }
    return -1;
  }

  virtual void flush() { stream.flush(); }

  virtual uint8_t connected() {
    if (available()) {
      return true;
    }
    return network_joined;
  }

  virtual operator bool() { return connected(); }

public:

  /*
   * Basic functions
   */
  bool begin(_lora_band band, uint32_t baud = 19200, uint16_t config = SERIAL_8N2) {
#ifdef SerialLoRa
    SerialLoRa.begin(baud, config);
    pinMode(LORA_BOOT0, OUTPUT);
    digitalWrite(LORA_BOOT0, LOW);
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
    digitalWrite(LORA_RESET, LOW);
    delay(200);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
#endif
    region = band;
    if (init()) {
        return configureBand(band);
    } else {
      return begin(band, baud, SERIAL_8N1);
    }
    return false;
  }

  bool init() {
    if (!autoBaud()) {
      return false;
    }
    // populate version field on startup
    version();
    if (isArduinoFW() && !isLatestFW()) {
      DBG("### Please update fw using MKRWANFWUpdate_standalone.ino sketch");
    }
    return true;
  }

  bool configureClass(_lora_class _class) {
    return setValue(GF(AT_CLASS), (char)_class);
  }

  bool configureBand(_lora_band band) {
    if (!setValue(GF(AT_BAND), band)) {
        return false;
    }
    if (band == EU868 && isArduinoFW()) {
        return dutyCycle(true);
    }
    return true;
  }

  int getChannelMaskSize(_lora_band band) {
    switch (band)
    {
      case AS923:
      case CN779:
      case EU433:
      case EU868:
      case KR920:
      case IN865:
        mask_size = 1;
        break;
      case AU915:
      case CN470:
      case US915:
      case US915_HYBRID:
        mask_size = 6;
        break;
      default:
        break;
    }
    return mask_size;
  }

  String getChannelMask() {
    int size = 4*getChannelMaskSize(region);
    sendAT(GF(AT_CHANMASK AT_QM));
	if ((!compat_mode && waitResponse(GF(AT_CHANMASK)) == 1)
			|| (compat_mode && waitResponse() == 1)) {
        channel_mask_str = stream.readStringUntil('\r');
        DBG("### Full channel mask string: ", channel_mask_str);
        sscanf(channel_mask_str.c_str(), "%04hx%04hx%04hx%04hx%04hx%04hx", &channelsMask[0], &channelsMask[1], &channelsMask[2],
                                                    &channelsMask[3], &channelsMask[4], &channelsMask[5]);

        return channel_mask_str.substring(0, size);
    }
    String str = "0";
    return str;
  }

  int isChannelEnabled(int pos) {
	populateChannelsMask();

    int row = pos / 16;
    int col = pos % 16;
    uint16_t channel = (uint16_t)(1 << col);

    channel = ((channelsMask[row] & channel) >> col);

    return channel;
  }

  bool disableChannel(int pos) {
	populateChannelsMask();

    int row = pos / 16;
    int col = pos % 16;
    uint16_t mask = ~(uint16_t)(1 << col);

    channelsMask[row] = channelsMask[row] & mask;

    return sendMask();
  }

  bool enableChannel(int pos) {
	populateChannelsMask();

	int row = pos / 16;
    int col = pos % 16;
    uint16_t mask = (uint16_t)(1 << col);

    channelsMask[row] = channelsMask[row] | mask;

    return sendMask();
  }

  bool sendMask() {
    String newMask;

    /* Convert channel mask into string */
    for (int i = 0; i < 6; i++) {
      char hex[5];
      sprintf(hex, "%04x", channelsMask[i]);
      newMask.concat(hex);
    }

    DBG("### Newmask: ", newMask);

    return sendMask(newMask);
  }

  bool sendMask(String newMask) {
    return setValue(GF(AT_CHANMASK), newMask);
  }

  void setBaud(unsigned long baud) {
    sendAT(GF(AT_UART), baud);
  }

  bool autoBaud(unsigned long timeout = 10000L) {
    for (unsigned long start = millis(); millis() - start < timeout; ) {
      sendAT(GF(""));
      if (waitResponse(200) == 1) {
          delay(100);
          return true;
      }
      delay(100);
    }
    return false;
  }

  String version() {
	fw_version = "";
	int ret = 0;
	sendAT(GF(AT_DEV), AT_QM);
	if ((ret = waitResponse(GF(AT_DEV),GF(LORA_OK))) == 1 || ret == 2) {
		fw_version = stream.readStringUntil('\r');
	}
	sendAT(GF(AT_VER), AT_QM);
	if ((ret = waitResponse(GF(AT_VER),GF(LORA_OK))) == 1 || ret == 2) {
		fw_version += " " + stream.readStringUntil('\r');
	}
    return fw_version;
  }

  String deviceEUI() {
    return getStringValue(GF(AT_DEUI));
  }

  void maintain() {
    while (stream.available()) {
      waitResponse(100);
    }
  }

  void minPollInterval(unsigned long secs) {
    pollInterval = secs * 1000;
  }

  void poll() {
    if (millis() - lastPollTime < pollInterval) return;
    lastPollTime = millis();
    // simply trigger a fake write
    uint8_t dummy = 0;
    modemSend(&dummy, 1, true);
  }

  bool factoryDefault() {
    sendAT(GF(AT_FACNEW));  // Factory
    return waitResponse() == 1;
  }

  /*
   * Power functions
   */

  bool restart() {
    if (!autoBaud()) {
      return false;
    }
    sendAT(GF(AT_RESET));
    if (waitResponse(10000L, AT_EVENT AT_EQ "0,0") != 1) {
      return false;
    }
    delay(1000);
    return init();
  }

  bool power(_rf_mode mode, uint8_t transmitPower) { // transmitPower can be between 0 and 5
	sendAT(GF(AT_TXP AT_EQ), mode,",",transmitPower);
	if ((!compat_mode && waitResponse(GF(AT_TXP)) == 1)
			|| (compat_mode && waitResponse() == 1))
      return false;
    String resp = stream.readStringUntil('\r');
    return true;
  }

  int32_t getPower() {
    return getIntValue(GF(AT_TXP));
  }

#ifdef SerialLoRa
  // Sends the modem into dumb mode, so the Semtech chip can be controlled directly
  // The only way to exit this mode is through a begin()
  void dumb() {
	SerialLoRa.end();
	pinMode(LORA_IRQ_DUMB, OUTPUT);
	digitalWrite(LORA_IRQ_DUMB, LOW);

	// Hardware reset
	pinMode(LORA_BOOT0, OUTPUT);
	digitalWrite(LORA_BOOT0, LOW);
	pinMode(LORA_RESET, OUTPUT);
	digitalWrite(LORA_RESET, HIGH);
	delay(200);
	digitalWrite(LORA_RESET, LOW);
	delay(200);
	digitalWrite(LORA_RESET, HIGH);
	delay(50);

	// You can now use SPI1 and LORA_IRQ_DUMB as CS to interface with the chip
  }
#endif

  bool dutyCycle(bool on) {
    return setValue(GF(AT_DCS), on);
  }

  bool setPort(uint8_t port) {
    return setValue(GF(AT_PORT), port);
  }

  bool publicNetwork(bool publicNetwork) {
    return setValue(GF(AT_PNM), publicNetwork);
  }

  bool sleep(bool on = true) {
    return setValue(GF(AT_SLEEP), on);
  }

  bool format(bool hexMode) {
    return setValue(GF(AT_FORMAT), hexMode);
  }

/*
	DataRate 	Modulation 	SF 	BW 	bit/s
	0 	LoRa 	12 	125 	250
	1 	LoRa 	11 	125 	440
	2 	LoRa 	10 	125 	980
	3 	LoRa 	9 	125 	1'760
	4 	LoRa 	8 	125 	3'125
	5 	LoRa 	7 	125 	5'470
	6 	LoRa 	7 	250 	11'000
*/

  bool dataRate(uint8_t dr) {
    return setValue(GF(AT_DR), dr);
  }

  int getDataRate() {
    return (int)getIntValue(GF(AT_DR));
  }

  bool setADR(bool adr) {
    return setValue(GF(AT_ADR), adr);
  }

  int getADR() {
    return (int)getIntValue(GF(AT_ADR));
  }

  String getDevAddr() {
    return getStringValue(GF(AT_DADDR));
  }

  String getNwkSKey() {
    return getStringValue(GF(AT_NWKSKEY));
  }

  String getAppSKey() {
    return getStringValue(GF(AT_APPSKEY));
  }

  int getRX2DR() {
    return (int)getIntValue(GF(AT_RX2DR));
  }

  bool setRX2DR(uint8_t dr) {
    return setValue(GF(AT_RX2DR),dr);
  }

  int32_t getRX2Freq() {
    return getIntValue(GF(AT_RX2FQ));
  }

  bool setRX2Freq(uint32_t freq) {
    return setValue(GF(AT_RX2FQ),freq);
  }

  bool setFCU(uint16_t fcu) {
    return setValue(GF(AT_FCU), fcu);
  }

  int32_t getFCU() {
    return getIntValue(GF(AT_FCU));
  }

  bool setFCD(uint16_t fcd) {
    return setValue(GF(AT_FCD), fcd);
  }

  int32_t getFCD() {
    return getIntValue(GF(AT_FCD));
  }

  int32_t getRSSI() {
    return getIntValue(GF(AT_RSSI));
  }

  int32_t getSNR() {
    return getIntValue(GF(AT_SNR));
  }

  bool getMsgConfirmed() {
    return getIntValue(GF(AT_CFS)) == 1;
  }

private:

  bool isArduinoFW() {
    return (fw_version.indexOf(ARDUINO_FW_IDENTIFIER) >= 0);
  }

  bool isLatestFW() {
	compat_mode = (fw_version.compareTo(ARDUINO_FW_VERSION_AT) < 0);
    return (fw_version == ARDUINO_FW_VERSION);
  }

  bool changeMode(_lora_mode mode) {
    return setValue(GF(AT_NJM), mode);
  }

  bool join(uint32_t timeout) {
    sendAT(GF(AT_JOIN));
    sendAT();
    if (waitResponse(timeout, AT_EVENT AT_EQ "1,1") != 1) {
      return false;
    }
    (void)streamSkipUntil('\r');
    return true;
  }

  bool set(_lora_property prop, const char* value) {
    switch (prop) {
        case APP_EUI:
        	return setValue(GF(AT_APPEUI), value);
        case APP_KEY:
        	return setValue(GF(AT_APPKEY), value);
        case DEV_EUI:
        	return setValue(GF(AT_DEUI), value);
        case DEV_ADDR:
        	return setValue(GF(AT_DADDR), value);
        case NWKS_KEY:
        	return setValue(GF(AT_NWKSKEY), value);
        case NWK_ID:
            return setValue(GF(AT_NWKID), value);
        case APPS_KEY:
        	return setValue(GF(AT_APPSKEY), value);
        default:
            return false;
    }
  }

  /**
   * @brief transmit uplink
   * 
   * @param buff data to transmit
   * @param len length of the buffer
   * @param confirmed true = transmit confirmed uplink
   * @return int a positive number indicate success and is the number of bytes transmitted
   *             -1 indicates a timeout error
   *             -2 indicates LORA_ERROR
   *             -3 indicates LORA_ERROR_PARAM
   *             -4 indicates LORA_ERROR_BUSY
   *             -5 indicates LORA_ERROR_OVERFLOW
   *             -6 indicates LORA_ERROR_NO_NETWORK
   *             -7 indicates LORA_ERROR_RX
   *             -8 indicates LORA_ERROR_UNKNOWN
   *             -20 packet exceeds max length
   *             
   */
  int modemSend(const void* buff, size_t len, bool confirmed) {

    size_t max_len = modemGetMaxSize();
    if (len > max_len) {
        return -20;
    }

    if (confirmed) {
        sendAT(GF(AT_CTX " "), len);
    } else {
        sendAT(GF(AT_UTX " "), len);
    }

    stream.write((uint8_t*)buff, len);

    int8_t rc = waitResponse();
    if (rc == 1) {            ///< OK
      return len;
    } else if ( rc > 1 ) {    ///< LORA ERROR
      return -rc;
    } else {                  ///< timeout
      return -1;
    }
  }

  size_t modemGetMaxSize() {
    if (isArduinoFW()) {
      return ARDUINO_LORA_MAXBUFF;
    }

    int size = getIntValue(GF(AT_MSIZE));
    if (size > 0){
    	msize = (size_t)size;
    	return msize;
    }
    return 0;
  }

  bool getJoinStatus() {
    return (getIntValue(GF(AT_NJS)));
  }

  /* Utilities */
  template<typename T>
  void streamWrite(T last) {
    stream.print(last);
  }

  template<typename T, typename... Args>
  void streamWrite(T head, Args... tail) {
    stream.print(head);
    streamWrite(tail...);
  }

  bool streamSkipUntil(char c, unsigned long timeout = 1000L) {
    unsigned long startMillis = millis();
	do {
	  if (stream.available()) {
		if (stream.read() == c)
	      return true;
	  }
    } while (millis() - startMillis < timeout);
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., LORA_NL);
    stream.flush();
    YIELD();
    DBG("### AT:", cmd...);
  }

  int char2int(char input)
  {
    if(input >= '0' && input <= '9')
      return input - '0';
    if(input >= 'A' && input <= 'F')
      return input - 'A' + 10;
    if(input >= 'a' && input <= 'f')
      return input - 'a' + 10;
    return 0;
  }

  void populateChannelsMask(){
	//Populate channelsMask array
	int max_retry = 3;
	for (int retry = 0; retry < max_retry; retry++) {
	  String mask = getChannelMask();
	  if (mask != "0") {
		break;
	  }
	}
  }

  /**
   * @brief wait for a response from the modem.
   * 
   * @param timeout the time in milliseconds to wait for a response
   * @param data a string containing the response to wait for
   * @param r1 response string defaults to LORA_OK
   * @param r2 response string defaults to LORA_ERROR
   * @param r3 response string defaults to LORA_ERROR_PARAM
   * @param r4 response string defaults to LORA_ERROR_BUSY
   * @param r5 response string defaults to LORA_ERROR_OVERFLOW
   * @param r6 response string defaults to LORA_ERROR_NO_NETWORK
   * @param r7 response string defaults to LORA_ERROR_RX
   * @param r8 response string defaults to LORA_ERROR_UNKNOWN
   * @return int8_t   n if the response = r<n>
   *                  99 if the response ends with "+RECV=", "+RECVB="
   *                  -1 if timeout
   */
  int8_t waitResponse(uint32_t timeout, String& data,
                       ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
                       ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    data.reserve(msize);
    int8_t index = -1;
    int length = 0;
    int a = 0;
    unsigned long startMillis = millis();
    do {
      YIELD();
      while (stream.available() > 0) {
        a = stream.peek();
        if (a < 0) continue;
        if (a == '=' || a == '\r'
        		|| (a == '+' && data.length() > 0)) {
			DBG("### Data string:", data);
			if (r1 && data.endsWith(r1)) {
			  index = 1;
			  goto finish;
			} else if (r2 && data.endsWith(r2)) {
			  index = 2;
			  goto finish;
			} else if (r3 && data.endsWith(r3)) {
			  index = 3;
			  goto finish;
			} else if (r4 && data.endsWith(r4)) {
			  index = 4;
			  goto finish;
			} else if (r5 && data.endsWith(r5)) {
			  index = 5;
			  goto finish;
			} else if (r6 && data.endsWith(r6)) {
			  index = 6;
			  goto finish;
			} else if (r7 && data.endsWith(r7)) {
			  index = 7;
			  goto finish;
			} else if (r8 && data.endsWith(r8)) {
			  index = 8;
			  goto finish;
			} else if ((data.endsWith(AT_RECV)
					|| data.endsWith(AT_RECVB)) && a == '=') {
			  (void)stream.readStringUntil(',').toInt();
			  length = stream.readStringUntil('\r').toInt();
			  (void)streamSkipUntil('\n');
			  (void)streamSkipUntil('\n');
			  if ((uint16_t)length >= msize){
				  DBG("### Data string too long:", data);
				  data = "";
				  length = 0;
				  continue;
			  }
			  if (data.endsWith(AT_RECVB)){ // Binary receive
				  char Hi = 0;
				  for (int i = 0; i < length*2;) {
					if (stream.available()) {
						if (!(i%2))
							Hi=char2int(stream.read()) * 0x10;
						else
							rx.put(char2int(stream.read()) + Hi);
						i++;
					}
				  }
			  }
			  else	// String receive
				  for (int i = 0; i < length;) {
					if (stream.available()) {
						rx.put(stream.read());
						i++;
					}
				  }
			  data = "";
			  length = 0;
			  continue;
			}
        }
        data += (char)stream.read();
        length++;
        if ((uint16_t)length >= msize){
        	DBG("### Data string too long:", data);
        	return index;
        }
      }
    } while (millis() - startMillis < timeout);
finish:
	if (a != '+') // no follow-up command, get terminator from buffer
		(void)stream.read();

    if (!index) {
      data.trim();
      if (data.length()) {
        DBG("### Unhandled:", data);
      }
      data = "";
    }
    return index;
  }

  int8_t waitResponse(uint32_t timeout,
                       ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
                       ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  int8_t waitResponse(ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
					  ConstStr r3=GFP(LORA_ERROR_PARAM), ConstStr r4=GFP(LORA_ERROR_BUSY), ConstStr r5=GFP(LORA_ERROR_OVERFLOW),
					  ConstStr r6=GFP(LORA_ERROR_NO_NETWORK), ConstStr r7=GFP(LORA_ERROR_RX), ConstStr r8=GFP(LORA_ERROR_UNKNOWN))
  {
    return waitResponse(1000, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  String getStringValue(ConstStr cmd){
	String value = "";
	sendAT(cmd, AT_QM);
	if ((!compat_mode && waitResponse(cmd) == 1)
			|| (compat_mode && waitResponse() == 1)) {
		value = stream.readStringUntil('\r');
	}
	return value;
  }

  int32_t getIntValue(ConstStr cmd){
	int32_t value = -1;
	sendAT(cmd, AT_QM);
	if ((!compat_mode && waitResponse(cmd) == 1)
			|| (compat_mode && waitResponse() == 1)) {
		value = stream.readStringUntil('\r').toInt();
	}
	return value;
  }

  template<typename T, typename U>
  bool setValue(T cmd, U value) {
	sendAT(cmd, AT_EQ, value);
	return (waitResponse() == 1);
  }

};
