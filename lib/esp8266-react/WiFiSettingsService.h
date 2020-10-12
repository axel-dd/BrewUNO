#ifndef WiFiSettingsService_h
#define WiFiSettingsService_h

#include <SettingsService.h>
#include <IPAddress.h>

#define WIFI_SETTINGS_FILE "/config/wifiSettings.json"
#define WIFI_SETTINGS_SERVICE_PATH "/rest/wifiSettings"
#define WIFI_RECONNECTION_DELAY 1000 * 10
#define WIFI_RECONNECTION_ATTEMPT 5

#define DEFAULT_WIFI_SSID "ssid"
#define DEFAULT_WIFI_PASSWORD "password"
#define DEFAULT_WIFI_HOSTNAME "BrewUno"

class WiFiSettingsService : public AdminSettingsService {

  public:

    WiFiSettingsService(AsyncWebServer* server, FS* fs);
    ~WiFiSettingsService();

    void begin();
    void loop();

  protected:

    void readFromJsonObject(JsonObject& root);
    void writeToJsonObject(JsonObject& root);
    void onConfigUpdated();

  private:
    // connection settings
    String _ssid;
    String _password;
    String _hostname;
    bool _staticIPConfig;

    // for the mangement delay loop
    unsigned long _lastConnectionAttempt;
    unsigned long _lastConnectionAttemptNumber;

    // optional configuration for static IP address
    IPAddress _localIP;
    IPAddress _gatewayIP;
    IPAddress _subnetMask;
    IPAddress _dnsIP1;
    IPAddress _dnsIP2;

#if defined(ESP8266)
    WiFiEventHandler _onStationModeDisconnectedHandler;
    void onStationModeDisconnected(const WiFiEventStationModeDisconnected& event);
#elif defined(ESP_PLATFORM)
    void onStationModeDisconnected(WiFiEvent_t event, WiFiEventInfo_t info);
#endif
    
    void readIP(JsonObject& root, String key, IPAddress& _ip);
    void writeIP(JsonObject& root, String key, IPAddress& _ip);
    void reconfigureWiFiConnection();
    void manageSTA();

};

#endif // end WiFiSettingsService_h
