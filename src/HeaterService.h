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

    // check if you can use PWM or interval heating (for example when having a relay or induction-cooker)
    if (IsIntervalHeatingOn())
    {
      //Serial.print("stepcounter = "); Serial.println(stepcounter);
      docheck = false;
      if (stepcounter > 100){
        stepcounter = 0;
        docheck = false;
        //Serial.print("docheck changed to false");
      } else if (millis() > (time + 300)) {
        stepcounter++;
        time = millis();
        docheck = true;
        //Serial.print("docheck changed to false");
      }
    }

    SetPidParameters(input, target);

    if (_activeStatus->PIDSettingsUpdated)
    {
      _activeStatus->PIDSettingsUpdated = false;
      StartPID(_brewSettingsService->KP, _brewSettingsService->KI, _brewSettingsService->KD);
      Serial.println("BrewSettings Updated: " + String(_brewSettingsService->KP) + "/" + String(_brewSettingsService->KI) + "/" + String(_brewSettingsService->KD));
      return status;
    }

    // boil mode
    if (_activeStatus->ActiveStep == boil)
    {
      status.PIDActing = false;
      if (IsIntervalHeatingOn())
      {
        if (docheck) // slow down check interval for interval heating
        {
          if (stepcounter > _brewSettingsService->BoilPowerPercentage) { 
            status.PWM = 0;
            status.PWMPercentage = 0;
            digitalWrite(_heaterBus, 0); 
          } else {
            status.PWM = 1023;
            status.PWMPercentage = 100;
            digitalWrite(_heaterBus, 1); 
          }  
        }
      } else {
        status.PWM = ((1023 * _brewSettingsService->BoilPowerPercentage) / 100);
        status.PWMPercentage = (status.PWM * 100) / 1023;
        analogWrite(_heaterBus, InvertedPWM() ? abs(status.PWM - 1023) : status.PWM);
      }
      return status;
    }

    if (_activeStatus->FullPower)
      heaterPercentage = 100;

    if (GetPidSetPoint() - GetPidInput() > _brewSettingsService->PIDStart)
    {
      status.PIDActing = false;
      
      if (IsIntervalHeatingOn())
      { 
        if (docheck) // slow down check interval for interval heating
        { 
          // do interval only after reaching cook temp
          if ((GetPidInput() < GetPidSetPoint()) && (stepcounter < heaterPercentage)) { 
            status.PWM = 1023;
            status.PWMPercentage = 100;
            digitalWrite(_heaterBus, 1); 
          } else {
            status.PWM = 0;
            status.PWMPercentage = 0;
            digitalWrite(_heaterBus, 0); 
          }
        }
      } else {
        status.PWM = ((1023 * heaterPercentage) / 100);
        status.PWMPercentage = (status.PWM * 100) / 1023;
        analogWrite(_heaterBus, status.PWM);
      }
      return status;
    }

    // to prevent pid overshoot
    if (GetPidInput() > GetPidSetPoint() + 0.1)
    {
      status.PWM = 0;
      status.PWMPercentage = 0;
      status.PIDActing = false;
      if (IsIntervalHeatingOn())
      {
        digitalWrite(_heaterBus, 0); 
      } else {
        analogWrite(_heaterBus, _activeStatus->PWM);
      }
      StartPID(_brewSettingsService->KP, _brewSettingsService->KI, _brewSettingsService->KD);
      return status;
    }

    PidCompute();

    // default behaviour
    int maxPWM = ((1023 * heaterPercentage) / 100);
    status.PWM = GetPidOutput() > maxPWM ? maxPWM : GetPidOutput();
    status.PWMPercentage = (status.PWM * 100) / 1023;
    if (IsIntervalHeatingOn())
    {
      if (docheck) // slow down check interval for interval heating
      {  
        if (stepcounter > status.PWMPercentage) { 
          status.PWM = 0;
          status.PWMPercentage = 0;
          digitalWrite(_heaterBus, 0); 
        } else {
          status.PWM = 1023;
          status.PWMPercentage = 100;
          digitalWrite(_heaterBus, 1); 
        }
      } 
    } else {
      analogWrite(_heaterBus, status.PWM);
    }

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
  bool docheck;
};
#endif