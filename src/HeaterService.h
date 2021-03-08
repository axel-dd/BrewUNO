#ifndef HeaterService_h
#define HeaterService_h

#include <TemperatureService.h>
#include <ActiveStatus.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <enum.h>
#include <BrewSettingsService.h>

struct HeaterServiceStatus
{
  double PWM;
  double PWMPercentage;
  boolean PIDActing;
};

class HeaterService
{
public:
  HeaterService(TemperatureService *temperatureService,
                ActiveStatus *activeStatus,
                BrewSettingsService *brewSettingsService) : _temperatureService(temperatureService),
                                                            _activeStatus(activeStatus),
                                                            _brewSettingsService(brewSettingsService)

  {
  }

  HeaterServiceStatus Compute(double input, double target, double heaterPercentage)
  {
    HeaterServiceStatus status;
    uint8_t _heaterBus = GetBus();
    SetUP();
    if (StopCompute())
    {
      status.PIDActing = false;
      status.PWM = 0;
      status.PWMPercentage = 0;
      TurnOff();
      return status;
    }

    SetPidParameters(input, target);

    if (_activeStatus->PIDSettingsUpdated)
    {
      _activeStatus->PIDSettingsUpdated = false;
      StartPID(_brewSettingsService->KP, _brewSettingsService->KI, _brewSettingsService->KD);
      Serial.println("BrewSettings Updated: " + String(_brewSettingsService->KP) + "/" + String(_brewSettingsService->KI) + "/" + String(_brewSettingsService->KD));
      return status;
    }

    // check if you can use PWM or interval heating (for example when having a relay or induction-cooker)
    if (IsIntervalHeatingOn())
    {
      // count to 100 and reset to 0 to do something like 100%
      if (stepcounter > 99){
        stepcounter = 0;
      } else if (millis() > (time + 100)) { // slow the counter down to not switch too fast
        stepcounter++;
        time = millis();
      }

      status.PIDActing = false;
      if (_activeStatus->ActiveStep == boil) {
        // overwrite heaterPercentage with BoilPowerPercentage in boil-mode
        heaterPercentage = _brewSettingsService->BoilPowerPercentage;
      }
      
      if (((_activeStatus->ActiveStep == boil) && (GetPidInput() < GetPidSetPoint())) // below cook temp always on
            || ((_activeStatus->ActiveStep == boil) && (GetPidInput() > GetPidSetPoint()) && (stepcounter < heaterPercentage)) // above cook temp do interval
            || ((GetPidInput() < (GetPidSetPoint() + 0.1)) && (stepcounter < heaterPercentage))) { // below step temp in mashing do interval
        status.PWM = 1023;
        status.PWMPercentage = 100;
        digitalWrite(_heaterBus, 1);
      } else {
        status.PWM = 0;
        status.PWMPercentage = 0;
        digitalWrite(_heaterBus, 0);
      }
      // when interval heating we can leave here
      return status;
    }
      
    // boil mode
    if (_activeStatus->ActiveStep == boil)
    {
      status.PIDActing = false;
      status.PWM = ((1023 * _brewSettingsService->BoilPowerPercentage) / 100);
      status.PWMPercentage = (status.PWM * 100) / 1023;
      analogWrite(_heaterBus, InvertedPWM() ? abs(status.PWM - 1023) : status.PWM);
      return status;
    }

    if (_activeStatus->FullPower)
      heaterPercentage = 100;

    if (GetPidSetPoint() - GetPidInput() > _brewSettingsService->PIDStart)
    {
      status.PIDActing = false;
      status.PWM = ((1023 * heaterPercentage) / 100);
      status.PWMPercentage = (status.PWM * 100) / 1023;
      analogWrite(_heaterBus, status.PWM);
      return status;
    }

    // to prevent pid overshoot
    if (GetPidInput() > GetPidSetPoint() + 0.1)
    {
      status.PWM = 0;
      status.PWMPercentage = 0;
      status.PIDActing = false;
      analogWrite(_heaterBus, _activeStatus->PWM);
      StartPID(_brewSettingsService->KP, _brewSettingsService->KI, _brewSettingsService->KD);
      return status;
    }

    PidCompute();

    // default behaviour
    int maxPWM = ((1023 * heaterPercentage) / 100);
    status.PWM = GetPidOutput() > maxPWM ? maxPWM : GetPidOutput();
    status.PWMPercentage = (status.PWM * 100) / 1023;
    analogWrite(_heaterBus, status.PWM);

    status.PIDActing = status.PWM > 0;
    return status;
  }

protected:
  virtual void SetUP();
  virtual boolean StopCompute();
  virtual void StartPID(double kp, double ki, double kd);
  virtual void PidCompute();
  virtual double GetPidOutput();
  virtual double GetPidInput();
  virtual double GetPidSetPoint();
  virtual uint8_t GetBus();
  virtual void TurnOff();
  virtual bool InvertedPWM();
  virtual void SetPidParameters(double input, double setpoint);
  virtual bool IsIntervalHeatingOn();

  TemperatureService *_temperatureService;
  ActiveStatus *_activeStatus;
  BrewSettingsService *_brewSettingsService;
  PID *_kettlePID;
  
  uint8_t stepcounter;
  unsigned long time = 0;
};
#endif