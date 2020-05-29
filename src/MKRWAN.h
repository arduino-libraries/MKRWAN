/*
  This file is part of the MKR WAN library.
  Copyright (C) 2017  Arduino AG (http://www.arduino.cc/)

  Based on the TinyGSM library https://github.com/vshymanskyy/TinyGSM
  Copyright (c) 2016 Volodymyr Shymanskyy

  MKR WAN library is free software: you can redistribute it and/or modify
  it under the terms of the GNU Lesser General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  MKR WAN library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public License
  along with MKR WAN library.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "Arduino.h"
#include "IPAddress.h"

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
  #define YIELD() { delay(0); }
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

#define LORA_NL "\n"
static const char LORA_OK[] = "OK\r\n";
static const char LORA_ERROR[] = "ERROR\r\n";
static const char LORA_ERROR_PARAM[] = "PARAM_ERROR\r\n";
static const char LORA_ERROR_BUSY[] = "+ERR_BUSY\r";
static const char LORA_ERROR_OVERFLOW[] = "+ERR_PARAM_OVERFLOW\r";
static const char LORA_ERROR_NO_NETWORK[] = "+ERR_NO_NETWORK\r";
static const char LORA_ERROR_RX[] = "+ERR_RX\r";
static const char LORA_ERROR_UNKNOWN[] = "+ERR_UNKNOWN\r";

static const char ARDUINO_FW_VERSION[] = "ARD-078 1.1.9";
static const char ARDUINO_FW_IDENTIFIER[] = "ARD-078";

typedef enum {
    AS923 = 0,
    AU915 = 1,
    CN470 = 2, 
    CN779 = 3,
    EU433 = 4,
    EU868 = 5,
    KR920 = 6,
    IN865 = 7,
    US915 = 8,
    RU864 = 9,
} _lora_band;

typedef enum {
    RFO = 0,
    PABOOST,
} _rf_mode;

typedef enum {
    ABP = 0,
    OTAA = 1,
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

struct fw_version_LoRa_s {
  String        app_version;
  String        mac_version;
};

IPAddress     app_addr;
IPAddress     mac_addr;

class LoRaModem : public Stream
{

public:
  LoRaModem(__attribute__((unused)) Stream& stream = (Stream&)Serial)
#ifdef SerialLoRa
    : stream(SerialLoRa), lastPollTime(millis()), pollInterval(300000)
#else
    : stream(stream), lastPollTime(millis()), pollInterval(300000)
#endif
    {}

public:
  typedef SerialFifo<uint8_t, LORA_RX_BUFFER> RxFifo;
  uint8_t portToJoin;

private:
  Stream&       stream;
  bool          network_joined;
  RxFifo        rx;
  RxFifo        tx;
  String        fw_version;
  unsigned long lastPollTime;
  unsigned long pollInterval;

public:
  virtual int joinOTAA(const char *appEui, const char *appKey, const char *devEui = NULL) {
    YIELD();
    rx.clear();
    changeMode(OTAA);
    set(APP_EUI, appEui);
    set(APP_KEY, appKey);
    if (devEui != NULL) {
        set(DEV_EUI, devEui);
    }
    network_joined = join();
    return (getJoinStatus() == 1);
  }

  virtual int joinOTAA(String appEui, String appKey) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), NULL);
  }

  virtual int joinOTAA(String appEui, String appKey, String devEui) {
    return joinOTAA(appEui.c_str(), appKey.c_str(), devEui.c_str());
  }

  virtual int joinABP(/*const char* nwkId, */const char * devAddr, const char * nwkSKey, const char * appSKey) {
    YIELD();
    rx.clear();
    changeMode(ABP);
    //set(NWK_ID, nwkId);
    set(DEV_ADDR, devAddr);
    set(NWKS_KEY, nwkSKey);
    set(APPS_KEY, appSKey);
    network_joined = join();
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
  bool begin(_lora_band band) {
#ifdef SerialLoRa
    SerialLoRa.begin(9600);
    pinMode(LORA_BOOT0, OUTPUT);
    digitalWrite(LORA_BOOT0, LOW);
    pinMode(LORA_RESET, OUTPUT);
    digitalWrite(LORA_RESET, HIGH);
    delay(200);
    digitalWrite(LORA_RESET, LOW);
    delay(200);
    digitalWrite(LORA_RESET, HIGH);
#endif
    if (init()) {
        return configureBand(band);
    }
    return false;
  }

  bool init() {
    String        init_msg;
    // populate version field on startup

    waitResponse(init_msg);
    return true;
  }

  bool configureClass(_lora_class _class) {
    sendAT(GF("+CLASS="), (char)_class);
    if (waitResponse() != 1) {
        return false;
    }
    return true;
  }

  bool configureBand(_lora_band band) {
    sendAT(GF("+BAND="), band);
    if (waitResponse() != 1) {
        return false;
    }
    if (band == EU868) {    
        return dutyCycle(true);
    }
  
    return true;
  }

  String getTConf() {
    String tconf_read;
    tconf_read="GetTConf Fail";

    sendAT(GF("+TCONF=?"));
    if (waitResponse(tconf_read) == 1) {
      tconf_read=tconf_read.substring(0, tconf_read.lastIndexOf('\r', tconf_read.indexOf("OK"))); 
    }
    return tconf_read;
  }

  String setTConf(String params) {
    String tconf;
    tconf="SetTConf Fail";
    
    //DBG("Debug in TConf. Parameters:", params);
    params.trim();
    sendAT(GF("+TCONF="), params.c_str());
    if (waitResponse(1000,tconf) == 1) {
      DBG("Waitresponse ok");
    }
    else {
      DBG("FAIL");
    }
    tconf = getTConf();
    return tconf;
  }

  bool enTtone() {
    sendAT(GF("+TTONE"));
    if (waitResponse() == 1) {
          return true;
    }
    return false;
  }

  int getrxfreq() {
    return getValue("RX2FQ").toInt();
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

  IPAddress version() {

    // TODO: split firmware version in actual versions
    String        app_version_str;
    String        mac_version_str;
    String        app_num_str;
    String        mac_num_str;

    sendAT(GF("+VER=?"));   
    if (waitResponse(fw_version) == 1) {
        app_version_str=fw_version.substring(0,fw_version.indexOf('\r'));
        mac_version_str=fw_version.substring(fw_version.indexOf('\n')+1, fw_version.lastIndexOf('\r', fw_version.indexOf("OK"))); 
    }
    app_num_str=app_version_str.substring(app_version_str.indexOf('=')+2, app_version_str.lastIndexOf('\r'));
    mac_num_str=mac_version_str.substring(mac_version_str.indexOf('=')+2, mac_version_str.lastIndexOf('\r'));

    if (app_addr.fromString(app_num_str)){
      DBG("App num: ", app_addr);
    }
    if (mac_addr.fromString(mac_num_str)){
      DBG("Mac num: ", mac_addr);
    }

    return app_addr;
  }

  String deviceEUI() {
    return getValue("DEUI");
  }

  String applicationKey() {
    return getValue("APPKEY");
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

  /*
   * Power functions
   */

  bool restart() {
    String rst_msg;
    if (!autoBaud()) {
      return false;
    }
    sendAT(GF("Z"));
    waitResponse(rst_msg);
    delay(1000);
    return init();
  }

  bool power(uint8_t transmitPower) { // transmitPower can be between 0 and 5
    sendAT(GF("+TXP="),transmitPower);
    if (waitResponse() != 1) {
      return false;
    }
    return true;
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
    sendAT(GF("+DCS="), on);   
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  bool setPort(uint8_t port) {
    portToJoin=port;
    return true;
  }

  bool publicNetwork(bool publicNetwork) {
    sendAT(GF("+PNM="), publicNetwork);
    if (waitResponse() != 1) {
      return false;
    }
    return true;
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
    sendAT(GF("+DR="), dr);
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  int getDataRate() {
    return getValue("DR").toInt();
  }

  bool setADR(bool adr) {
    sendAT(GF("+ADR="), adr);
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  int getADR() {
    return getValue("ADR").toInt();
  }

  bool setCFM(bool cfm) {
    sendAT(GF("+CFM="), cfm);
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  int getCFM() {
    return getValue("CFM").toInt();
  }

  int getCFS() {
    return getValue("CFS").toInt();
  }

  String getDevAddr() {
    return getValue("DADDR");
  }

  String getNwkSKey() {
    return getValue("NWKSKEY");
  }

  String getAppSKey() {
    return getValue("APPKEY");
  }


private:

  bool isArduinoFW() {
    return (fw_version.indexOf(ARDUINO_FW_IDENTIFIER) >= 0);
  }

  bool changeMode(_lora_mode mode) {
    sendAT(GF("+NJM="),mode);  
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  bool join() {
    sendAT(GF("+JOIN"));
    if (waitResponse(60000L, "JOINED") != 1) {
      return false;
    }
    return true;
  }

  bool set(_lora_property prop, const char* value) {

    char* real_val = (char*)value;

    // fw 1.3.1 requires keys to be in "mac address" format
    if (strstr(value, ":") == NULL) {
      String mac;
      int i = 0;
      while (1) {
        mac += value[i++];
        mac += value[i++];
        if (value[i] != '\0') {
          mac += ":";
        } else {
          break;
        }
      }
      real_val = (char*)mac.c_str();
    }

    switch (prop) {
        case APP_EUI:
            sendAT(GF("+APPEUI="), real_val);
            break;
        case APP_KEY:
            sendAT(GF("+APPKEY="), real_val);
            break;
        case DEV_EUI:
            sendAT(GF("+DEUI="), real_val);
            break;
        case DEV_ADDR:
            sendAT(GF("+DADDR="), real_val);
            break;
        case NWKS_KEY:
            sendAT(GF("+NWKSKEY="), real_val);
            break;
        case NWK_ID:
            sendAT(GF("+NWKID="), real_val);
            break;
        case APPS_KEY:
            sendAT(GF("+APPSKEY="), real_val);
            break;
        default:
            return false;
    }
    if (waitResponse() != 1) {
      return false;
    }
    return true;
  }

  /**
   * @brief transmit uplink
   * 
   * @param buff data to transmit`
   * @param len length of the buffer`
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
  int modemSend(uint8_t* buff, size_t len, bool confirmed) {

    String msg_buf;
    // Convert buff to hex
    for (size_t i=0; i<len; i++) {
      if (buff[i] < 10) {
        msg_buf += "0";
      }
      msg_buf += String(buff[i], HEX);
    }

    setCFM(confirmed);

    sendAT(GF("+SENDB="), portToJoin, ":", msg_buf);

    int8_t rc = waitResponse( GFP(LORA_OK), GFP(LORA_ERROR), GFP(LORA_ERROR_PARAM), GFP(LORA_ERROR_BUSY), GFP(LORA_ERROR_OVERFLOW), GFP(LORA_ERROR_NO_NETWORK), GFP(LORA_ERROR_RX), GFP(LORA_ERROR_UNKNOWN) );

    if (confirmed && rc == 1) {
      // need both OK and shitty confirmation string
      const char* confirmation = "confirmed message transmission";
      rc = waitResponse(5000, GFP(confirmation));
    }

    if (rc == 1) {            ///< OK
      return len;
    } else if ( rc > 1 ) {    ///< LORA ERROR
      return -rc;
    } else {                  ///< timeout
      return -1;
    }
  }

  bool getJoinStatus() {
    String njs;
    sendAT(GF("+NJS=?"));

    if (waitResponse(njs) == 1) {
      njs=njs.substring(0,njs.lastIndexOf('\r', njs.indexOf("OK")));
      njs.trim();
      if (njs == "1") {
        return true;
      }
    }
    return false;
  }

  String getValue(String val) {
      String temp;
      sendAT(GF("+" + val + "=?"));
      if (waitResponse(temp) == 1) {
        temp=temp.substring(0,temp.lastIndexOf('\r', temp.indexOf("OK")));
        return temp;
     }
  return "";
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

  int streamRead() { return stream.read(); }

  bool streamSkipUntil(char c) { //TODO: timeout
    while (true) {
      while (!stream.available()) {}
      if (stream.read() == c)
        return true;
    }
    return false;
  }

  template<typename... Args>
  void sendAT(Args... cmd) {
    streamWrite("AT", cmd..., LORA_NL);
    stream.flush();
    YIELD();
    DBG("### AT:", cmd...);
  }

  uint8_t toHex(const char* str) {
    char temp[3];
    temp[0] = str[0];
    temp[1] = str[1];
    temp[2] = '\0';
    return strtol(temp, NULL, 16);
  }

  // TODO: Optimize this!
  /**
   * @brief wait for a response from the modem.
   * 
   * @param timeout the time in milliseconds to wait for a response
   * @param data a string containing the response to wait for
   * @param r1 response string
   * @param r2 response string
   * @param r3 response string
   * @param r4 response string
   * @param r5 response string
   * @param r6 response string
   * @param r7 response string
   * @param r8 response string
   * @return int8_t   n if the response = r<n>
   *                  99 if the response ends with "+RECV="
   *                  -1 if timeout
   */
  int8_t waitResponse(uint32_t timeout, String& data,
                       ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=NULL, ConstStr r4=NULL, ConstStr r5=NULL,
                       ConstStr r6=NULL, ConstStr r7=NULL, ConstStr r8=NULL)
  {
    data.reserve(64);
    int8_t index = -1;
    unsigned long startMillis = millis();
    do {
      YIELD();
      while (stream.available() > 0) {
        int a = streamRead();
        if (a < 0) continue;
        data += (char)a;
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
        } else if (data.endsWith("+EVT:")) {
          data = "";
          int port = stream.readStringUntil(':').toInt();
          data = stream.readStringUntil('\r');
          for (size_t i=0; i<data.length(); i+=2) {
            rx.put(toHex(&(data.c_str()[i])));
          }
          // consume buffer; contains band and power info
          data = stream.readStringUntil('\r');
        }
      }
    } while (millis() - startMillis < timeout);
finish:
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
                       ConstStr r3=NULL, ConstStr r4=NULL, ConstStr r5=NULL,
                       ConstStr r6=NULL, ConstStr r7=NULL, ConstStr r8=NULL)
  {
    String data;
    return waitResponse(timeout, data, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  int8_t waitResponse(ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=NULL, ConstStr r4=NULL, ConstStr r5=NULL,
                       ConstStr r6=NULL, ConstStr r7=NULL, ConstStr r8=NULL)
  {
    return waitResponse(1000, r1, r2, r3, r4, r5, r6, r7, r8);
  }

  int8_t waitResponse(String& data, ConstStr r1=GFP(LORA_OK), ConstStr r2=GFP(LORA_ERROR),
                       ConstStr r3=NULL, ConstStr r4=NULL, ConstStr r5=NULL,
                       ConstStr r6=NULL, ConstStr r7=NULL, ConstStr r8=NULL)
  {
    return waitResponse(1000, data, r1, r2, r3, r4, r5, r6, r7, r8);
  }                    
};
