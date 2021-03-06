/* sdhal_file_log_handler -- 
 *
 * Copyright 2019 Jean-philippe GOI
 * 
 * This file is part of GnuVario-E.
 *
 * ToneHAL is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ToneHAL is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

/*********************************************************************************/
/*                                                                               */
/*                   sdhal_file_log_handler                                      */
/*                                                                               */
/*  version    Date        Description                                           */
/*    1.0      25/11/19                                                          */
/*    1.1      29/11/19    Modif changement librairie sdfat                      */
/*                                                                               */
/*********************************************************************************/

#include "mpp-console.h"
#include "mpp-sdhal_log_handler.h"
//#define LOG_DEBUG
#include "mpp_local.h"

#include <HardwareConfig.h>

#if !defined ESP8266
// Not implented on ESP8266 as SD filesystem does not support renaming

#define FILE_MAX_NAME   15

#ifdef SDFAT_LIB
SdFatFileLogHandler::SdFatFileLogHandler(LogFormatter *formatter, char *filename, size_t maxSize, int backupCount) :
#else
SdFatFileLogHandler::SdFatFileLogHandler(LogFormatter *formatter, SdCardHAL *sd, char *filename, size_t maxSize, int backupCount) :
#endif //SDFAT_LIB
LogHandler(formatter, "SdFatFileLogHandler")
{
#ifdef SDFAT_LIB
#else
  log_sd = sd;
#endif //SDFAT_LIB
  log_name = filename;
  log_size = maxSize;
  log_backup = backupCount;
}

#if defined(ESP32) && not defined (SDFAT_LIB) 

File SdFatFileLogHandler::openLog(void)
{
  File f;

  if (log_sd->exists(log_name)) {
    debug_print("openLog: %s exists\n", log_name);
    f = log_sd->open(log_name, FILE_WRITE);  //FILE_APPEND);
    if (!f) {
      debug_print("openLog: %s error\n", log_name);
      return f;
    }
  }
  else {
    debug_print("openLog: %s missing\n", log_name);
    f = log_sd->open(log_name, FILE_WRITE);
    if (!f) {
      debug_print("openLog: %s error\n", log_name);
      return f;
    }
    return f;
  }
  if (f.size() > log_size) {
    debug_print("openLog %s %ld > %ld\n", log_name, f.size(), log_size);
    f.close();
    logRotate();
    debug_print("openLog: create %s\n", log_name);
    File f = log_sd->open(log_name, FILE_WRITE);
    if (!f) {
      debug_print("openLog: %s error\n", log_name);
      return f;
    }
    debug_print("%s: created\n", log_name);
    return f;
  }
  return f;
}

#else

SdFile SdFatFileLogHandler::openLog(void)
{
  SdFile f;
	bool retour = f.open(log_name, O_WRONLY | O_CREAT);
   if (!retour) {
    debug_print("openLog: %s: error 0x%x\n", log_name, 0); //SDHAL_SD.card()->errorCode());
    return f;
  }
  f.seekEnd();
  if (f.fileSize() > log_size) {
    debug_print("openLog %s %ld > %ld\n", log_name, f.fileSize(), log_size);
    f.close();
    logRotate();
    SdFile f;
		bool retour = f.open(log_name, O_WRONLY | O_CREAT);
    if (!retour) {
      debug_print("openLog: %s error 0x%x\n", log_name, 0); //log_sd->card()->errorCode());
      return f;
    }
    f.seekEnd();
    return f;
  }
  return f;
}

#endif

#if defined(ESP32) && not defined(SDFAT_LIB)

int SdFatFileLogHandler::logRotate(void)
{
  int count = 0, fno, flast = 0;
  size_t size;

  File dir = SDHAL.open("/");
  if (!dir) {
    debug_print("/: error\n");
    return 0;
  }
  dir.rewindDirectory();
  while (true) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    char fname[FILE_MAX_NAME];
    strcpy(fname, entry.name());
    if (!strncmp(fname, log_name, strlen(log_name))) {
      debug_print("found %s %ld\n", fname, entry.size());
      char *p = strrchr(fname, '.');
      char *ext = p+1;
      fno = atoi(ext);
      if (fno != 0) {
        if (fno > flast) {
          flast = fno;
        }
      }
    }
    entry.close();
    yield();
  }
  dir.close();
  klog("SdFatFileLogHandler::logRotate: %d files\n", flast);
  if (flast != 0) {
    for (fno = flast ; fno >= log_backup ; fno--) {
      char name[FILE_MAX_NAME];
      sprintf(name, "%s.%d", log_name, fno);
      if (SDHAL.exists(name)) {
        debug_print("%s: delete\n", name);
        if (SDHAL.remove(name) != true) {
          debug_print("%s: cannot delete\n", name);
        }
        debug_print("%s: successfully deleted\n", name);
      }
    }
    for (fno = flast ; fno > 0 ; fno--) {
      char name[FILE_MAX_NAME];
      char newname[FILE_MAX_NAME];
      sprintf(name, "%s.%d", log_name, fno);
      sprintf(newname, "%s.%d", log_name, fno+1);
      debug_print("%s: rename to %s\n", name, newname);
      if (log_sd->rename(name, newname) != true) {
        debug_print("%s: cannot rename to %s\n", name, newname);
      }
      debug_print("%s: successfully renamed to %s\n", name, newname);
    }
  }
  char newname[FILE_MAX_NAME];
  sprintf(newname, "%s.%d", log_name, fno+1);
  debug_print("%s: rename to %s\n", log_name, newname);
  if (log_sd->rename(log_name, newname) != true) {
    debug_print("%s: cannot rename to %s\n", log_name, newname);
  }
  debug_print("%s: successfully renamed to %s\n", log_name, newname);
  return 0;
}

#else

int SdFatFileLogHandler::logRotate(void)
{
  int count = 0, fno, flast = 0;
  size_t size;
  SdFile file;
  
	SdFile dir;
	bool retour = dir.open("/", O_RDONLY);
  if (!retour) {
    debug_print("/: error\n");
    return 0;
  }
  while (file.openNext(&dir, O_RDONLY)) {
    char fname[FILE_MAX_NAME];
    file.getName(fname, FILE_MAX_NAME);
    if (!strncmp(fname, log_name, strlen(log_name))) {
      debug_print("found %s %ld\n", fname, size);
      char *p = strrchr(fname, '.');
      char *ext = p+1;
      fno = atoi(ext);
      if (fno != 0) {
        if (fno > flast) {
          flast = fno;
        }
      }
    }
  }
  file.close();
  klog("SdFatFileLogHandler::logRotate: %d files\n", flast);
  if (flast != 0) {
    for (fno = flast ; fno >= log_backup ; fno--) {
      char name[FILE_MAX_NAME];
      sprintf(name, "%s.%d", log_name, fno);
      debug_print("%s: delete\n", name);
      if (SDHAL_SD.remove(name) != true) {
        debug_print("%s: cannot delete\n", name);
      }
      debug_print("%s: successfully deleted\n", name);
      flast--;
    }
    for (fno = flast ; fno > 0 ; fno--) {
      char name[FILE_MAX_NAME];
      char newname[FILE_MAX_NAME];
      sprintf(name, "%s.%d", log_name, fno);
      sprintf(newname, "%s.%d", log_name, fno+1);
      debug_print("%s: rename to %s\n", name, newname);
      if (SDHAL_SD.rename(name, newname) != true) {
        debug_print("%s: cannot rename to %s\n", name, newname);
      }
      debug_print("%s: successfully renamed to %s\n", name, newname);
    }
  }
  char newname[FILE_MAX_NAME];
  sprintf(newname, "%s.%d", log_name, fno+1);
  debug_print("%s: rename to %s\n", log_name, newname);
  if (SDHAL_SD.rename(log_name, newname) != true) {
    debug_print("%s: cannot rename to %s\n", log_name, newname);
  }
  debug_print("%s: successfully renamed to %s\n", log_name, newname);
  return 0;
}

#endif

void SdFatFileLogHandler::send(const char *msg)
{
#if not defined(SDFAT_LIB)
  File f = openLog();
  if (f) {
#else
  SdFile f = openLog();
  if (f.isOpen()) {
#endif
    size_t len = strlen(msg);
    size_t n = f.println(msg);
    f.close();
    debug_print("%s: %s\n", log_name, msg);
    debug_print("%s: %d/%d bytes written\n", log_name, n, len);
  }
}

void SdFatFileLogHandler::sendRaw(const char *msg)
{
#if not defined (SDFAT_LIB)
  File f = openLog();
  if (f) {
#else
  SdFile f;
  f = openLog();
  if (f.isOpen()) {
#endif
    size_t len = strlen(msg);
    size_t n = f.print(msg);
    f.close();
    debug_print("%s: %s\n", log_name, msg);
    debug_print("%s: %d/%d bytes written\n", log_name, n, len);
  }
}

#endif
