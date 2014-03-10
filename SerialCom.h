//************************************************************************************
// general config parameter access routines
//************************************************************************************

// types of config parameters
enum confType {
  BOOL,
  INT8,
  INT16,
  INT32,
  UINT8,
  UINT16,
  UINT32
};

#define CONFIGNAME_MAX_LEN 17
typedef struct configDef {
  char name[CONFIGNAME_MAX_LEN];  // name of config parameter
  confType type;                  // type of config parameters
  void * address;                 // address of config parameter
  void (* updateFunction)(void);  // function is called when parameter update happens
} t_configDef;

t_configDef configDef;

// access decriptor as arry of bytes as well
typedef union {
  t_configDef   c;
  char          bytes[sizeof(t_configDef)];
} t_configUnion;

t_configUnion configUnion;

//
// list of all config parameters
// to be accessed by par command
//
// descriptor is stored in PROGMEN to preserve RAM space
// see http://www.arduino.cc/en/Reference/PROGMEM
// and http://jeelabs.org/2011/05/23/saving-ram-space/
const t_configDef PROGMEM configListPGM[] = {
  {"vers",             UINT8, &config.vers,             NULL},

  {"pitchKp",        INT32, &config.pitchKp,            &initPIDs},
  {"pitchKi",        INT32, &config.pitchKi,            &initPIDs},
  {"pitchKd",        INT32, &config.pitchKd,            &initPIDs},
  {"rollKp",         INT32, &config.rollKp,             &initPIDs},
  {"rollKi",         INT32, &config.rollKi,             &initPIDs},
  {"rollKd",         INT32, &config.rollKd,             &initPIDs},
  {"timeConstant",   INT16, &config.accTimeConstant,    &initIMU},
  {"mpuLPF",         INT8,  &config.mpuLPF,             &initMPUlpf},
  
  {"angleOffsetPitch", INT16, &config.angleOffsetPitch, NULL},
  {"angleOffsetRoll",  INT16, &config.angleOffsetRoll,  NULL},
  
  {"dirMotorPitch",    INT8,  &config.dirMotorPitch,    NULL},
  {"dirMotorRoll",     INT8,  &config.dirMotorRoll,     NULL},
  {"motorNumberPitch", UINT8, &config.motorNumberPitch, NULL},
  {"motorNumberRoll",  UINT8, &config.motorNumberRoll,  NULL},
  {"maxPWMmotorPitch", UINT8, &config.maxPWMmotorPitch, &recalcMotorStuff},
  {"maxPWMmotorRoll",  UINT8, &config.maxPWMmotorRoll,  &recalcMotorStuff},

  {"minRCPitch",       INT8,  &config.minRCPitch,       NULL},
  {"maxRCPitch",       INT8,  &config.maxRCPitch,       NULL},
  {"minRCRoll",        INT8,  &config.minRCRoll,        NULL},
  {"maxRCRoll",        INT8,  &config.maxRCRoll,        NULL},
  {"rcGain",           INT16, &config.rcGain,           NULL},
  {"rcLPF",            INT16, &config.rcLPF,            &initRC},

  {"rcMid",            INT16, &config.rcMid,            NULL},
  {"rcAbsolute",       BOOL,  &config.rcAbsolute,       NULL},
  
  {"majorAxis",        UINT8, &config.majorAxis,        &initSensorOrientation},

  {"axisReverseZ",     BOOL,  &config.axisReverseZ,      &initSensorOrientation},
  {"axisSwapXY",       BOOL,  &config.axisSwapXY,        &initSensorOrientation},
  
  {"", BOOL, NULL, NULL} // terminating NULL required !!
};

// read bytes from program memory
void getPGMstring (PGM_P s, char * d, int numBytes) {
  for (int i=0; i<numBytes; i++) {
    *d++ = pgm_read_byte(s++);
  }
}

// find Config Definition for named parameter
t_configDef * getConfigDef(char * name) {
  bool found = false;  
  t_configDef * p = (t_configDef *)configListPGM;

  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    if (strncmp(configUnion.c.name, name, CONFIGNAME_MAX_LEN) == 0) {
      found = true;
      break;
   }
   p++; 
  }
  if (found) 
      return &configUnion.c;
  else 
      return NULL;
}

// print single parameter value
void printConfig(t_configDef * def) {
  if (def != NULL) {
    Serial.print(def->name);
    Serial.print(F(" "));
    switch (def->type) {
      case BOOL   : Serial.print(*(bool *)(def->address)); break;
      case UINT8  : Serial.print(*(uint8_t *)(def->address)); break;
      case UINT16 : Serial.print(*(uint16_t *)(def->address)); break;
      case UINT32 : Serial.print(*(uint32_t *)(def->address)); break;
      case INT8   : Serial.print(*(int8_t *)(def->address)); break;
      case INT16  : Serial.print(*(int16_t *)(def->address)); break;
      case INT32  : Serial.print(*(int32_t *)(def->address)); break;
    }
    Serial.println("");
  } else {
    Serial.println(F("ERROR: illegal parameter"));    
  }
}

// write single parameter with value
void writeConfig(t_configDef * def, int32_t val) {
  if (def != NULL) {
    switch (def->type) {
      case BOOL   : *(bool *)(def->address)     = val; break;
      case UINT8  : *(uint8_t *)(def->address)  = val; break;
      case UINT16 : *(uint16_t *)(def->address) = val; break;
      case UINT32 : *(uint32_t *)(def->address) = val; break;
      case INT8   : *(int8_t *)(def->address)   = val; break;
      case INT16  : *(int16_t *)(def->address)  = val; break;
      case INT32  : *(int32_t *)(def->address)  = val; break;
    }
    // call update function
    if (def->updateFunction != NULL) def->updateFunction();
  } else {
    Serial.println(F("ERROR: illegal parameter"));    
  }
}

// print all parameters
void printConfigAll(t_configDef * p) {
  while (true) {
    getPGMstring ((PGM_P)p, configUnion.bytes, sizeof(configDef)); // read structure from program memory
    if (configUnion.c.address == NULL) break;
    printConfig(&configUnion.c);
    p++; 
  }
  Serial.println(F("done."));
}

//******************************************************************************
// general parameter modification function
//      par                           print all parameters
//      par <parameter_name>          print parameter <parameter_name>
//      par <parameter_name> <value>  set parameter <parameter_name>=<value>
//*****************************************************************************
void parameterMod() {

  char * paraName = NULL;
  char * paraValue = NULL;
  
  int32_t val = 0;

  if ((paraName = sCmd.next()) == NULL) {
    // no command parameter, print all config parameters
    printConfigAll((t_configDef *)configListPGM);
  } else if ((paraValue = sCmd.next()) == NULL) {
    // one parameter, print single parameter
    printConfig(getConfigDef(paraName));
  } else {
    // two parameters, set specified parameter
    val = atol(paraValue);
    writeConfig(getConfigDef(paraName), val);
  }
}
//************************************************************************************

void updateAllParameters() {
  recalcMotorStuff();
  initPIDs();
  initIMU();
  initMPUlpf();
  initSensorOrientation();
  initRCPins();
  initRC();
}

void setDefaultParametersAndUpdate() {
  setDefaultParameters();
  updateAllParameters();
}

void writeEEPROM() {
  config.crc16 = crc16((uint8_t*)&config, sizeof(config)-2); // set proper CRC 
  EEPROM_writeAnything(0, config);
}

void readEEPROM() {
  wdt_reset();
  EEPROM_readAnything(0, config); 
  if (config.crc16 == crc16((uint8_t*)&config, sizeof(config)-2)) { 
    updateAllParameters();
  } else {
    wdt_reset();
    // crc failed intialize directly here, as readEEPROM is void
    Serial.print(F("EEPROM CRC failed, initialize EEPROM"));
    setDefaultParameters();
    writeEEPROM();
  }
}

void transmitActiveConfig()
{
  Serial.println(config.vers);
  Serial.println(config.pitchKp);
  Serial.println(config.pitchKi);
  Serial.println(config.pitchKd);
  Serial.println(config.rollKp);
  Serial.println(config.rollKi);
  Serial.println(config.rollKd);
  Serial.println(config.accTimeConstant);
  Serial.println(config.nPolesMotorPitch);
  Serial.println(config.nPolesMotorRoll);
  Serial.println(config.dirMotorPitch);
  Serial.println(config.dirMotorRoll);
  Serial.println(config.motorNumberPitch);
  Serial.println(config.motorNumberRoll);
  Serial.println(config.maxPWMmotorPitch);
  Serial.println(config.maxPWMmotorRoll);
}

void gyroRecalibrate() {
  // Set voltage on all motor phases to zero 
  softStart = 0;
  mpu.setDLPFMode(MPU6050_DLPF_BW_5);  // experimental AHa: set to slow mode during calibration
  gyroOffsetCalibration();
  initMPUlpf();
  Serial.println(F("recalibration: done"));
}

void printHelpUsage() {
  Serial.println(F("This gives you a list of all commands with usage:"));
  Serial.println(F("Explanations are in brackets(), use integer values only !"));
  Serial.println();
  Serial.println(F("these are the preferred commands, use them for new GUIs !!"));
  Serial.println();
  Serial.println(F("SD    (Set Defaults)"));
  Serial.println(F("WE    (Writes active config to eeprom)"));
  Serial.println(F("RE    (Restores values from eeprom to active config)"));  
  Serial.println(F("GC    (Recalibrates the Gyro Offsets)"));
  Serial.println(F("par <parName> <parValue>   (general parameter read/set command)"));
  Serial.println(F("    example usage:"));
  Serial.println(F("       par                     ... list all config parameters"));
  Serial.println(F("       par pitchKi             ... list pitchKi"));
  Serial.println(F("       par pitchKi 12000       ... set pitchKi to 12000"));
  Serial.println();
  Serial.println(F("HE     (print this output)"));
  Serial.println();
  Serial.println(F("Note: command input is case-insensitive, commands are accepted in both upper/lower case"));
}

void unrecognized(const char *command)  {
  Serial.println(F("What? type in HE for Help ..."));
}

void setSerialProtocol() {
  // Setup callbacks for SerialCommand commands
  sCmd.addCommand("sd", setDefaultParametersAndUpdate);   
  sCmd.addCommand("we", writeEEPROM);   
  sCmd.addCommand("re", readEEPROM); 
  sCmd.addCommand("par", parameterMod);
  sCmd.addCommand("gc", gyroRecalibrate);

  sCmd.addCommand("he", printHelpUsage);
  sCmd.setDefaultHandler(unrecognized);      // Handler for command that isn't matched  (says "What?")
}

